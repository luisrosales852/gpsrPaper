"""Generate and replay paired GPSR-Sim benchmark batches. See README for commands."""

import argparse
import hashlib
import json
import math
import random
import re
import subprocess
import sys
from pathlib import Path
from statistics import mean, stdev
from types import SimpleNamespace

from gpsr_merger.recovery import RecoveryPolicy
from sim.recovery_trials import go, say, _interleaved
from sim.run_sim import HERE, run_once
from sim.skill_model import SkillModel, fit_skill_model
from sim.world import World

METHODS = {
    "one-by-one": ("one-by-one", RecoveryPolicy.SKIP_ONLY),
    "back-to-back": ("sequential", RecoveryPolicy.SKIP_ONLY),
    "greedy": ("merged", RecoveryPolicy.SKIP_ONLY),
    **{policy.value: ("merged", policy) for policy in RecoveryPolicy},
    "skip-defer+replan": ("merged", RecoveryPolicy.SKIP_DEFER),
}


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def parse_utterance(text):
    """Deterministic parser for the four sampled official command forms."""
    def site(name):
        return name.replace(" ", "_")

    count = re.fullmatch(r"tell me how many (.+) there are on the (.+)", text)
    if count:
        target, location = count.groups()
        return [go(site(location)), {"action": "count", "target_to_count": target},
                go("start_location"), say(target)]
    guide = re.fullmatch(r"(?:guide|escort|take|lead) (\w+) from the (.+) to the (.+)", text)
    if guide:
        person, start, end = guide.groups()
        return [go(site(start)), {"action": "find_person_by_name", "name": person},
                {"action": "guide_person_to", "destination_room": site(end)}]
    fetch = re.fullmatch(r"(?:take|get|grasp|fetch) (?:a|an) (.+?) from the (.+?) and (.+)", text)
    if fetch:
        obj, start, followup = fetch.groups()
        prefix = [go(site(start)), {"action": "pick_object", "object_to_pick": obj}]
        place = re.fullmatch(r"(?:place|put) it on the (.+)", followup)
        if place:
            return prefix + [go(site(place[1])), {"action": "place_object"}]
        give = re.fullmatch(r"(?:bring|give|deliver) it to (\w+) in the (.+)", followup)
        if give:
            return prefix + [go(site(give[2])), {"action": "find_person_by_name", "name": give[1]},
                             {"action": "give_object"}]
    raise ValueError(f"unsupported benchmark utterance: {text!r}")


def language_step(adapter, request):
    response = subprocess.run([str(adapter.resolve())], input=json.dumps(request), text=True,
                              capture_output=True, check=True, timeout=120)
    return json.loads(response.stdout)


def generate(generator_root, apartment, seed, batches, language_adapter=None):
    """Stratify by supported goal family; bind the official grammar to the apartment."""
    if batches < 1:
        raise ValueError("batches must be positive")
    sys.path.insert(0, str(generator_root.resolve() / "src"))
    from robocupathome_generator.gpsr_commands import CommandGenerator

    class BoundGenerator(CommandGenerator):
        def insert_placeholders(self, ph):
            key = ph.strip("{}")
            if key in self.bindings:
                return self.bindings[key].replace("_", " ")
            return super().insert_placeholders(ph)

        def _sample_followup(self, key, category):
            return self.followup

    gen = BoundGenerator(None)
    world = World(apartment)
    rng = random.Random(seed)
    state = random.getstate()
    try:
        random.seed(seed)  # The upstream grammar uses the module-level RNG.
        corpus = []
        for i in range(batches):
            obj = rng.choice([o for o in world.objects if o.kind in ("cup", "book", "plate")])
            guide, recipient = rng.sample(world.people, 2)
            site = world.nearest_site(recipient.home)
            source = world.nearest_site(guide.home)
            drop = rng.choice([x for x in ("counter", "bookshelf", "side_table") if x != obj.site])
            dest = rng.choice([x for x in ("bedroom", "garage", "office")
                               if world.room_of(source) != x])
            at = rng.choice(("side_table", "counter", "dishwasher", "bookshelf"))
            target = rng.choice(("apples", "cups", "books", "plates"))
            jobs = [
                ("countObjOnPlcmt", None, {"plurCat": target, "plcmtLoc": at},
                 {"type": "report", "site": at, "target": target}),
                ("guideNameFromBeacToBeac", None,
                 {"name": guide.name, "loc": source, "loc_room": dest},
                 {"type": "escort", "person": guide.name, "source": source, "goal": dest}),
            ]
            if i % 2:
                jobs.append(("takeObjFromPlcmt", "deliverObjToNameAtBeac",
                             {"obj_singCat": obj.kind, "plcmtLoc": obj.site, "name": recipient.name,
                              "room": world.room_of(site) or site},
                             {"type": "handover", "object": obj.name, "source": obj.site,
                              "person": recipient.name, "goal": world.room_of(site) or site}))
            else:
                jobs.append(("takeObjFromPlcmt", "placeObjOnPlcmt",
                             {"obj_singCat": obj.kind, "plcmtLoc": obj.site, "plcmtLoc2": drop},
                             {"type": "delivery", "object": obj.name, "source": obj.site, "goal": drop}))
            rng.shuffle(jobs)
            sites = {o.name: o.site for o in world.objects}
            records = []
            for template, followup, slots, goal in jobs:
                gen.bindings, gen.followup = slots, followup
                gen.object_cmd_list = [(template, 1)]
                utterance = gen.generate_command_start("objects")
                if goal["type"] == "report":
                    n = sum(world.observes_site(at, sites[o.name]) for o in world.objects_matching(target))
                    goal["answer"] = f"{n} {target}"
                elif goal["type"] in ("delivery", "handover"):
                    sites[obj.name] = goal["goal"]
                alternatives = []
                spoken = utterance
                if language_adapter:
                    alternatives = language_step(language_adapter, {"operation": "rephrase", "text": utterance,
                                                                     "seed": seed + i})
                    if (not isinstance(alternatives, list) or len(alternatives) > 3
                            or any(not isinstance(t, str) or not t.strip() for t in alternatives)):
                        raise ValueError("rephrase must return up to three nonempty strings")
                    spoken = rng.choice(alternatives or [utterance])
                    actions = language_step(language_adapter, {"operation": "parse", "text": spoken})
                    if not isinstance(actions, list) or any(not isinstance(a, dict) or not a.get("action")
                                                            for a in actions):
                        raise ValueError("parse must return a list of action objects")
                else:
                    actions = parse_utterance(spoken)
                records.append(dict(template=template, followup=followup, slots=slots, goal=goal,
                                    utterance=utterance, rephrasings=alternatives, spoken=spoken, actions=actions))
            corpus.append({"name": f"batch-{i:04}", "records": records})
    finally:
        random.setstate(state)
    rev = subprocess.check_output(["git", "-C", str(generator_root), "rev-parse", "HEAD"], text=True).strip()
    return {"generator_revision": rev, "seed": seed, "parser": ({"adapter": language_adapter.name,
                        "sha256": hashlib.sha256(language_adapter.read_bytes()).hexdigest()}
                       if language_adapter else "four-family English grammar v1"),
            "apartment_sha256": hashlib.sha256(apartment.read_bytes()).hexdigest(), "batches": corpus}


def violations(events):
    """Audit gripper ownership and provenance of spoken answers in the execution trace."""
    owner = None
    facts = {}
    errors = {"gripper": 0, "answer_context": 0}
    for e in events:
        cmd, kind = e["cmd"], e["kind"]
        if e["phase"] == "start":
            if kind == "pick_object" and e["held"] is not None:
                errors["gripper"] += 1
            elif kind in ("place_object", "give_object") and (e["held"] is None or owner != cmd):
                errors["gripper"] += 1
            continue
        if kind == "pick_object" and e["held"] is not None:
            owner = cmd
        if e["held"] is None:
            owner = None
        if e["status"] != "EXECUTION_SUCCESS" or e["purpose"] == "reacquire":
            continue
        history = facts.setdefault(cmd, {})
        if kind == "say_with_context":
            expected = "; ".join(history[idx] for idx in sorted(history) if idx < e["idx"])
            if e["result"].partition(": ")[2] != expected:
                errors["answer_context"] += 1
        elif kind != "go_to" and e["result"]:
            history[e["idx"]] = e["result"]
    return errors


def score(records, log, world):
    summary = log["summary"]
    events = [e for e in log["events"] if e["phase"] == "end" and e["status"] == "EXECUTION_SUCCESS"]
    full, partial = [], []
    sites = {o.name: o.site for o in world.objects}
    expected_answers = {}
    for cmd, record in enumerate(records):
        g = record["goal"]
        work = [e for e in events if e["cmd"] == cmd]
        if g["type"] == "report":
            # Evaluate the issue-order question using the mutations earlier
            # commands actually achieved, including a failed delivery.
            count = sum(world.observes_site(g["site"], sites[o.name])
                        for o in world.objects_matching(g["target"]))
            answer = f"{count} {g['target']}"
            expected_answers[cmd] = answer
            observed = any(e["kind"] == "count" and e["result"] == answer for e in work)
            solved = any(e["kind"] == "say_with_context" and e["result"].partition(": ")[2] == answer
                         and math.dist((e["x"], e["y"]), world.locator("start_location")) < 0.01
                         for e in work)
        elif g["type"] == "escort":
            observed = any(e["kind"] == "find_person_by_name" and e["result"] == g["person"] for e in work)
            xy = summary["people_xy"][g["person"]]
            solved = observed and any(e["kind"] == "guide_person_to" for e in work) and any(
                z["name"] == g["goal"] and z["rect"][0] <= xy[0] <= z["rect"][2]
                and z["rect"][1] <= xy[1] <= z["rect"][3] for z in world.zones)
        else:
            obj = next(o for o in world.objects if o.name == g["object"])
            actual = summary["objects_xyz"][obj.name]
            observed = any(e["kind"] == "pick_object" and e["held"] == obj.kind for e in work)
            if g["type"] == "handover":
                solved = (any(e["kind"] == "give_object" and e["result"] == f"{obj.kind} to {g['person']}"
                              for e in work) and math.dist(actual[:2], summary["people_xy"][g["person"]]) < 0.01
                          and abs(actual[2] - 1.05) < 0.01)
            else:
                x, y, yaw = world.pose(g["goal"])
                drop = (x + 0.65 * math.cos(yaw), y + 0.65 * math.sin(yaw), 0.75)
                solved = math.dist(actual, drop) < 0.01
        for event in log["events"]:
            if event["cmd"] == cmd and event["phase"] == "end":
                sites.update(event.get("site_changes", {}))
        if solved:
            full.append(cmd)
        elif observed:
            partial.append(cmd)
    return dict(completed=full, partial=partial, expected_answers=expected_answers, violations=violations(log["events"]), units=len(full) + 0.5 * len(partial),
                elapsed_s=summary["elapsed_s"], distance_m=summary["actual_distance_m"], holding_s=summary["holding_time_s"],
                returned=summary["returned"], interleaved=_interleaved(log), replans=len(summary["replans"]))


def evaluate(manifest, apartments, out, repeats, rates, skills=None, holding_penalty=0.0,
             travel_exposure_only=False):
    if repeats < 1 or not manifest["batches"] or not rates:
        raise ValueError("provide at least one batch, rate and replicate")
    if skills and len(rates) > 1:
        raise ValueError("use one fallback rate with a fitted per-skill model")
    rows = []
    for layout, apartment in enumerate(apartments):
        world = World(apartment)
        for i, batch in enumerate(manifest["batches"]):
            records = batch["records"]
            scenario = {"name": batch["name"], "origin": world.locator("start_location"),
                        "commands": [r["actions"] for r in records]}
            for rate in rates:
                for trial in range(repeats):
                    seed = f"{manifest['seed']}/{layout}/{i}/{trial}"
                    paired = []
                    for method, (mode, policy) in METHODS.items():
                        args = SimpleNamespace(apartment=apartment, width=832, height=720, fps=30,
                            speed=3, camera="topdown", retries=2, budget=420, fail=[], no_video=True,
                            print_tree=False, ordering_baseline=True, recovery_policy=policy, replan=method.endswith("+replan"),
                            greedy=method == "greedy", skill_model=SkillModel(seed, rate, skills),
                            travel_exposure_only=travel_exposure_only,
                            holding_penalty=holding_penalty if mode == "merged" and method != "greedy" else 0.0)
                        directory = out / f"layout-{layout}" / batch["name"] / f"{rate}-{trial}" / method
                        run_once(scenario, mode, str(directory), args)
                        log = json.loads((directory / "run.json").read_text())
                        paired.append(dict(batch=i, layout=layout, seed=seed, rate=rate, method=method,
                                           **score(records, log, world)))
                    baseline = next(r for r in paired if r["method"] == "back-to-back")
                    for r in paired:
                        r["bonus_evidence"] = (r["interleaved"] and len(r["completed"]) == 3 and r["returned"]
                            and len(baseline["completed"]) == 3 and baseline["returned"]
                            and (r["elapsed_s"] < baseline["elapsed_s"] or r["distance_m"] < baseline["distance_m"]))
                    rows.extend(paired)
    summary = []
    for rate in rates:
        for method in METHODS:
            group = [r for r in rows if r["rate"] == rate and r["method"] == method]
            metrics = {"pilot_units": [r["units"] for r in group],
                       "violations": [sum(r["violations"].values()) for r in group],
                       "solved_fraction": [len(r["completed"]) / 3 for r in group],
                       "bonus_evidence_fraction": [int(r["bonus_evidence"]) for r in group]}
            for field in ("elapsed_s", "distance_m", "holding_s", "replans"):
                metrics[field] = [r[field] for r in group]
            summary.append({"rate": rate, "method": method, "runs": len(group),
                            **{key: {"mean": mean(values), "sd": stdev(values) if len(values) > 1 else None}
                               for key, values in metrics.items()}})
    save(out / "results.json", {"layouts": [hashlib.sha256(p.read_bytes()).hexdigest() for p in apartments],
                                "manifest_sha256": hashlib.sha256(json.dumps(manifest, sort_keys=True).encode()).hexdigest(),
                                "holding_penalty": holding_penalty, "travel_exposure_only": travel_exposure_only,
                                "skills": skills, "summary": summary, "rows": rows})
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    gen = sub.add_parser("generate")
    gen.add_argument("--generator", type=Path, required=True)
    gen.add_argument("--seed", type=int, default=2027)
    gen.add_argument("--batches", type=int, default=8)
    gen.add_argument("--language-adapter", type=Path)
    gen.add_argument("--output", type=Path, required=True)
    gen.add_argument("--apartment", type=Path, default=Path(HERE)/"apartment.json")
    run = sub.add_parser("run")
    run.add_argument("manifest", type=Path)
    run.add_argument("--apartment", type=Path, action="append")
    run.add_argument("--out", type=Path, required=True)
    run.add_argument("--rates", type=float, nargs="+", default=[0, .1, .2, .4])
    run.add_argument("--repeats", type=int, default=1)
    run.add_argument("--skills", type=Path)
    run.add_argument("--travel-exposure-only", action="store_true")
    run.add_argument("--holding-penalty", type=float, default=0.0)
    fit = sub.add_parser("fit")
    fit.add_argument("observations", type=Path)
    fit.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "generate":
        save(args.output, generate(args.generator, args.apartment, args.seed, args.batches, args.language_adapter))
    elif args.command == "fit":
        save(args.output, fit_skill_model(json.loads(args.observations.read_text())))
    else:
        rows = evaluate(json.loads(args.manifest.read_text()), args.apartment or [Path(HERE)/"apartment.json", Path(HERE)/"apartment_partitioned.json"],
                        args.out, args.repeats, args.rates,
                        json.loads(args.skills.read_text()) if args.skills else None, args.holding_penalty,
                        args.travel_exposure_only)
        print(f"{len(rows)} paired method runs written to {args.out}")


if __name__ == "__main__":
    main()
