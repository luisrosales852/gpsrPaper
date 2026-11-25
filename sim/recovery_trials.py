"""Run paired three-command recovery trials and check the execution evidence.

    python -m sim.recovery_trials --out out/recovery
"""

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path

from gpsr_merger.recovery import RecoveryPolicy
from gpsr_merger.timeouts import GLOBAL_BUDGET_S
from sim.run_sim import HERE, run_once
from sim.world import World


def go(place):
    return {"action": "go_to", "location_to_go": place}


def say(subject):
    return {"action": "say_with_context", "user_instruction": subject}


def fetch(obj, start, goal):
    return [go(start), {"action": "pick_object", "object_to_pick": obj},
            go(goal), {"action": "place_object"}]


CUPS = [go("kitchen"), {"action": "count", "target_to_count": "cups"},
        go("start_location"), say("the cups")]
TOM = [go("bookshelf"), {"action": "find_person_by_name", "name": "Tom"},
       {"action": "get_person_info", "info_type": "shirt_colour"},
       go("start_location"), say("Tom's shirt")]
INFORMATION = [CUPS, TOM, fetch("book", "bedroom", "garage")]
MANIPULATION = [fetch("book", "bedroom", "bookshelf"),
                fetch("cup", "kitchen", "dishwasher"), TOM]


@dataclass(frozen=True)
class Trial:
    name: str
    commands: list
    failures: tuple = ()
    start_elapsed: float = 0.0

    def scenario(self):
        return {"name": self.name, "origin": [0.0, 0.0], "commands": self.commands,
                "elapsed_before_execution_s": self.start_elapsed}


TRIALS = (
    Trial("nominal", INFORMATION),
    Trial("transient-information", INFORMATION, ("0.1:1",)),
    Trial("deferred-target", INFORMATION, ("1.2:2",)),
    Trial("persistent-information", INFORMATION, ("1.2:3",)),
    Trial("failed-pick-shared-navigation", MANIPULATION, ("0.1:3",)),
    Trial("failed-release", MANIPULATION, ("0.3:3",)),
    Trial("insufficient-time", INFORMATION, ("1.2:2",), 300.0),
)


def _key(event):
    return event["cmd"], event["idx"]


def _score(trial, log, world):
    """Score reported facts and final deliveries against fixture ground truth."""
    summary = log["summary"]
    successes = [e for e in log["events"] if e["phase"] == "end"
                 and e["status"] == "EXECUTION_SUCCESS" and e["purpose"] != "reacquire"]
    by_key = {_key(e): e for e in successes}
    completed, partial, recovered = [], [], []
    for cmd, actions in enumerate(trial.commands):
        last = len(actions) - 1
        final = by_key.get((cmd, last))
        if actions[-1]["action"] == "place_object":
            obj = actions[1]["object_to_pick"]
            place = world.pose(actions[2]["location_to_go"])
            actual = summary["objects_xyz"][obj]
            assert not final or (final["held"] is None and final["result"] == obj)
            # The placement handler puts the object 0.65 m ahead of the goal pose.
            drop_xy = [place[0] + 0.65 * math.cos(place[2]),
                       place[1] + 0.65 * math.sin(place[2])]
            full = math.dist(drop_xy, actual[:2]) <= 0.003 and abs(actual[2] - 0.75) <= 0.001
            intermediate = by_key.get((cmd, 1), {}).get("held") == obj
        else:
            expected = "1 cups" if actions == CUPS else "Tom; yellow"
            answer = f"About {actions[-1]['user_instruction']}: {expected}"
            assert not final or final["result"] == answer, final
            full = final is not None
            info = by_key.get((cmd, 1 if actions == CUPS else 2), {})
            intermediate = info.get("result") == ("1 cups" if actions == CUPS else "yellow")
        if full:
            completed.append(cmd)
            if final and final["branch"] == "deferred":
                recovered.append(cmd)
        elif intermediate:
            partial.append(cmd)

    return {
        "completed_goals": len(completed),
        "partial_credit": 0.5 * len(partial),
        "goals_recovered_by_defer": len(recovered),
        "completed_commands": completed,
        "partial_commands": partial,
        "elapsed_s": summary["elapsed_s"],
        "distance_m": summary["actual_distance_m"],
        "returned": summary["returned"],
    }


def _verify(trial, log, world):
    expected_places = {}
    for i, command in enumerate(trial.commands):
        place = (0.0, 0.0)
        for j, action in enumerate(command):
            if action["action"] == "go_to":
                place = world.pose(action["location_to_go"])[:2]
            expected_places[i, j] = place

    summary = log["summary"]
    assert summary["elapsed_s"] <= GLOBAL_BUDGET_S
    if summary["returned"]:
        assert summary["final_xy"] == [0.0, 0.0]
    held_by = None
    events = log["events"]
    for start, end in zip(events[::2], events[1::2], strict=True):
        assert (start["phase"], end["phase"]) == ("start", "end") and _key(start) == _key(end)
        if start["kind"] == "go_to":
            continue
        cmd, _ = _key(start)
        assert math.dist((start["x"], start["y"]), expected_places[_key(start)]) < 0.003, start
        if start["kind"] == "pick_object":
            assert start["held"] is None, start
            if end["held"] is not None:
                held_by = cmd
        if start["kind"] == "place_object":
            assert start["held"] is not None and held_by == cmd, start
            if end["held"] is None:
                held_by = None

    reports = log["failure_reports"]
    assert len({_key(e) for e in reports}) == len(reports)
    for report in reports:
        assert report["text"].startswith(f"Command {report['cmd'] + 1}: ")
        assert (report["x"], report["y"]) == (0.0, 0.0)


def _interleaved(log):
    work = [e["cmd"] for e in log["events"]
            if e["phase"] == "end" and e["status"] == "EXECUTION_SUCCESS"
            and e["kind"] != "go_to" and e["purpose"] != "reacquire"]
    return any(cmd != work[i - 1] and cmd in work[:i] for i, cmd in enumerate(work) if i)


def _run(trial, policy, directory, mode="merged"):
    args = argparse.Namespace(
        apartment=str(Path(HERE) / "apartment.json"), width=832, height=720,
        fps=30, speed=3.0, camera="topdown", retries=2, budget=GLOBAL_BUDGET_S,
        fail=trial.failures, no_video=True, print_tree=False, recovery_policy=policy,
    )
    run_once(trial.scenario(), mode, str(directory), args)
    log = json.loads((directory / "run.json").read_text())
    world = World(args.apartment)
    _verify(trial, log, world)
    return log, _score(trial, log, world)


def run_trials(out):
    out.mkdir(parents=True, exist_ok=True)
    _, baseline = _run(TRIALS[0], RecoveryPolicy.SEQUENTIAL_FALLBACK, out / "nominal-back-to-back",
                       mode="sequential")
    rows = []
    for trial in TRIALS:
        for policy in RecoveryPolicy:
            log, score = _run(trial, policy, out / trial.name / policy.value)
            meaningful = (trial.name == "nominal" and _interleaved(log)
                          and score["completed_goals"] == 3 and score["returned"]
                          and baseline["completed_goals"] == 3 and baseline["returned"]
                          and (score["elapsed_s"] < baseline["elapsed_s"]
                               or score["distance_m"] < baseline["distance_m"]))
            rows.append({"trial": trial.name, "policy": policy.value, **score,
                         "plan_demonstration_observed": False,
                         "simulated_bonus_evidence": meaningful,
                         "competition_bonus_awarded": None})
    (out / "results.json").write_text(json.dumps({"nominal_back_to_back": baseline,
                                                  "trials": rows}, indent=2) + "\n")
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("out/recovery"))
    for row in run_trials(parser.parse_args().out):
        print(f"{row['trial']:30} {row['policy']:20} {row['completed_goals']}  "
              f"{row['partial_credit']:.1f}  {row['elapsed_s']:6.1f}  "
              f"{row['distance_m']:5.1f}  {row['goals_recovered_by_defer']}  "
              f"{'returned' if row['returned'] else 'return incomplete'}")


if __name__ == "__main__":
    main()
