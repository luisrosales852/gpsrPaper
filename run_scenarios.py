#!/usr/bin/env python3
"""Run the merger against every scenario in scenarios/merger.json.

Pure Python: no ROS, and no py_trees (merger.py alone has no third-party
deps). This is the smoke test that the extraction from home2 is intact.
"""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gpsr_merger.merger import merge  # noqa: E402
from tests._merger_helpers import (  # noqa: E402
    build_commands,
    describe_plan,
    evaluate_expectations,
    make_locator,
)

SCENARIOS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenarios", "merger.json")


def main() -> int:
    verbose = "-v" in sys.argv
    with open(SCENARIOS) as f:
        scenarios = json.load(f)

    passed = 0
    for sc in scenarios:
        commands = build_commands(sc["commands"])
        kwargs = {}
        if "coords" in sc:
            kwargs["locator"] = make_locator(sc["coords"])
        if "origin" in sc:
            kwargs["origin"] = tuple(sc["origin"]) if sc["origin"] is not None else None

        plan = None
        try:
            plan = merge(commands, **kwargs)
            ok, failures = evaluate_expectations(plan, commands, sc.get("expect", {}))
        except Exception as e:  # noqa: BLE001
            ok, failures = False, [f"EXCEPTION: {e}"]

        passed += ok
        print(f"{'PASS' if ok else 'FAIL'}  {sc['name']}" + ("" if ok else f"   <- {'; '.join(failures)}"))
        if verbose and plan is not None:
            print(f"      {describe_plan(plan)}")

    print(f"\n{passed}/{len(scenarios)} scenarios passed")
    return 0 if passed == len(scenarios) else 1


if __name__ == "__main__":
    sys.exit(main())
