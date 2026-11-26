"""Run a deterministic remaining-time fixture, including the final return."""
import argparse
import json
from pathlib import Path

from gpsr_merger.endgame import plan_endgame
from tests._merger_helpers import build_commands


def replay(fixture, remaining_s, bin_s):
    legs = {}
    for a, b, seconds in fixture["symmetric_travel_s"]:
        legs[a, b] = legs[b, a] = seconds
    plan = plan_endgame(build_commands(fixture["commands"]), start=fixture["start"],
                        home=fixture["home"], travel_s=lambda a, b: legs.get((a, b)),
                        initial_holder=fixture.get("initial_holder"),
                        skill_durations_s=fixture["skill_durations_s"],
                        bin_s=bin_s, remaining_s=remaining_s)
    row = {"remaining_s": remaining_s, "bin_s": bin_s, "return_feasible": plan is not None}
    if plan is not None:
        row.update(segment_order=plan.segment_order, completed=plan.completed,
                   elapsed_s=plan.elapsed_s, return_s=plan.return_s, return_to=plan.return_to,
                   steps=[step.action.action for step in plan.actions] + ["return_to_instruction_point"])
    return row


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fixture", type=Path)
    parser.add_argument("--remaining", type=float, nargs="+")
    parser.add_argument("--bins", type=float, nargs="+", default=[1])
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    fixture = json.loads(args.fixture.read_text())
    rows = [replay(fixture, seconds, resolution)
            for seconds in args.remaining or [fixture["remaining_s"]] for resolution in args.bins]
    report = json.dumps(rows, indent=2, allow_nan=False)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(report + "\n")
    print(report)
