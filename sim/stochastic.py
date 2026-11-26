"""Small-batch stochastic scheduling oracle for the structured benchmark.

Enumerate segment outcomes and solve the finite, acyclic MDP by backward
induction. Compare policies under identical transitions and terminal rewards.
"""

import argparse
import json
import math
from collections import defaultdict
from functools import lru_cache
from pathlib import Path

from gpsr_merger.merger import _action_effects, _annotate_effects, _build_needs, _decompose, _room_table
from sim.benchmark import save
from sim.handlers import DURATION
from sim.recovery_trials import HERE
from sim.robot import LIN_SPEED
from sim.run_sim import planning_router
from sim.world import World
from tests._merger_helpers import build_commands

OBSERVATIONS = {"find_person", "find_person_by_name", "get_person_info", "count", "get_visual_info"}
ACQUIRE = {"count", "pick_object", "find_person", "find_person_by_name", "get_person_info"}


def compare(records, world, failure_rate, skills=None, budget=420, seconds_per_goal=120):
    """Minimize expected elapsed seconds minus reward for completed/partial goals."""
    if not 0 <= failure_rate <= 1 or budget <= 0 or seconds_per_goal <= 0:
        raise ValueError("invalid failure rate, budget or reward")
    skills = skills or {}
    commands = build_commands([r["actions"] for r in records])
    home = "start_location"
    segments = [_decompose(c, command) for c, command in enumerate(commands)]
    for segs, cmd in zip(segments, commands):
        _annotate_effects(segs, cmd, home)
    needs = _build_needs(segments, _room_table([s for cmd in segments for s in cmd], world.room_of))
    lengths = tuple(map(len, segments))
    router = planning_router(world)

    @lru_cache(None)
    def drive(start, end):
        distance = router.distance(world.locator(start), world.locator(end))
        return budget + 1 if distance is None else math.ceil(distance / LIN_SPEED)

    @lru_cache(None)
    def attempts(kind, nominal):
        model = skills.get(kind, {})
        p = 0.0 if nominal else model.get("failure_rate", failure_rate)
        durations = defaultdict(float)
        for ok, mass, field, default in ((True, 1-p, "success_s", DURATION.get(kind, 1.2)),
                                         (False, p, "failure_s", 0.6)):
            samples = model.get(field) or [default]
            for duration in samples:
                durations[ok, math.ceil(duration)] += mass / len(samples)
        return [(ok, t, prob) for (ok, t), prob in durations.items() if prob]

    @lru_cache(None)
    def outcomes(c, j, nominal):
        """(probability, elapsed, exit site, success, gripper change, earned partial)."""
        segment = segments[c][j]
        entry = segment.location or home
        live = {(0, entry, None, False): 1.0}
        finished = defaultdict(float)
        for idx in segment.indices:
            action = commands[c].commands[idx]
            kind = action.action
            if kind == "go_to":
                continue
            next_live = defaultdict(float)
            for (elapsed, place, grip, partial), mass in live.items():
                dest = _action_effects(action, place)[2] or place
                leg = drive(place, dest) if dest != place else 0
                for ok, seconds, prob in attempts(kind, nominal):
                    retry = attempts(kind, nominal) if not ok and kind in OBSERVATIONS else [(ok, 0, 1.0)]
                    for final_ok, extra, weight in retry:
                        t = elapsed + seconds + extra
                        joint = mass * prob * weight
                        if not final_ok:
                            finished[t, place, False, grip, partial] += joint
                            continue
                        g = grip
                        if kind == "pick_object":
                            g = c
                        elif kind in ("give_object", "place_object"):
                            g = -1
                        next_live[t + leg, dest, g, partial or kind in ACQUIRE] += joint
            live = next_live
        for (t, place, grip, partial), mass in live.items():
            finished[t, place, True, grip, partial] += mass
        return [(p, *outcome) for outcome, p in finished.items()]

    def eligible(k, holder):
        for c, j in enumerate(k):
            if j == lengths[c] or any(k[d] < n for d, n in needs[c][j]):
                continue
            if holder not in (-1, c) and segments[c][j].ends_holding is not None:
                continue
            yield c

    @lru_cache(None)
    def solve(state, policy):
        k, place, holder, partials, remaining = state
        choices = list(eligible(k, holder))
        if remaining == 0 or not choices:
            return min(remaining, drive(place, home)), None
        nominal = policy == "nominal"
        if policy == "replan":
            choices = [solve(state, "nominal")[1]]
        best = (math.inf, None)
        for c in choices:
            j = k[c]
            hop = drive(place, segments[c][j].location or home)
            q = 0.0
            for p, seconds, end, ok, grip, partial in outcomes(c, j, nominal):
                elapsed = seconds + hop
                if elapsed > remaining:
                    q += p * remaining
                    continue
                progress = list(k)
                progress[c] = j + 1 if ok else lengths[c]
                mask = partials | ((1 << c) if partial else 0)
                before = 0.5 if partials & (1 << c) else 0.0
                after = 1.0 if ok and progress[c] == lengths[c] else (0.5 if mask & (1 << c) else 0.0)
                nxt = (tuple(progress), end, holder if grip is None else grip, mask, remaining - elapsed)
                q += p * (elapsed - seconds_per_goal * (after - before) + solve(nxt, policy)[0])
            best = min(best, (q, c))
        return best

    start = ((0,) * len(commands), home, -1, 0, budget)
    exact, first_exact = solve(start, "exact")
    replan, first_replan = solve(start, "replan")
    return {"exact_cost": exact, "replan_cost": replan, "gap_seconds": replan - exact,
            "exact_first": first_exact, "replan_first": first_replan}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("manifest", type=Path)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--apartment", type=Path, default=Path(HERE)/"apartment.json")
    p.add_argument("--rates", type=float, nargs="+", default=[0, .1, .2, .4])
    p.add_argument("--skills", type=Path)
    p.add_argument("--budget", type=int, default=420)
    p.add_argument("--seconds-per-goal", type=float, default=120)
    args = p.parse_args()
    rows = []
    world = World(args.apartment)
    skills = json.loads(args.skills.read_text()) if args.skills else {}
    for batch in json.loads(args.manifest.read_text())["batches"]:
        for rate in args.rates:
            rows.append({"batch": batch["name"], "rate": rate,
                         **compare(batch["records"], world, rate, skills, args.budget, args.seconds_per_goal)})
    save(args.out, {"budget": args.budget, "seconds_per_goal": args.seconds_per_goal, "rows": rows})


if __name__ == "__main__":
    main()
