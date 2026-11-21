#!/usr/bin/env python3
"""Travel vs distance carried as the holding penalty grows (paper's lambda).

    python3 bench_holding.py            # plain table
    python3 bench_holding.py --latex    # rows for main.tex
"""

import argparse
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gpsr_merger.merger import _euclidean_distance, merge  # noqa: E402
from tests._merger_helpers import build_commands, make_locator, make_room_of, planned_travel  # noqa: E402
from tests.test_dependencies import COORDS, ROOMS, _random_batch  # noqa: E402

PENALTIES = [0.0, 0.25, 0.5, 1.0, 2.0]
N_BATCHES = 1000
SEED = 20261121


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--latex", action="store_true")
    args = ap.parse_args()

    rng = random.Random(SEED)
    locator = make_locator(COORDS)
    room_of = make_room_of(ROOMS)
    batches = []
    while len(batches) < N_BATCHES:
        cmds_json = _random_batch(rng)
        if any(a["action"] == "pick_object" for c in cmds_json for a in c):
            batches.append((build_commands(cmds_json), (rng.uniform(0, 9), rng.uniform(-3, 3))))

    def measure(actions, commands, origin):
        distance = _euclidean_distance(locator, origin)
        travel = planned_travel(actions, commands, distance, True)
        carried = planned_travel(actions, commands, distance, True, holding_penalty=1.0) - travel
        return travel, carried

    rows = []
    seq = [measure([pa for per in merge(c, locator=locator, origin=o, room_of=room_of).fallback
                    for pa in per], c, o) for c, o in batches]
    rows.append(("sequential", seq))
    for penalty in PENALTIES:
        rows.append((penalty, [measure(merge(c, locator=locator, origin=o, room_of=room_of,
                                             holding_penalty=penalty).actions, c, o)
                               for c, o in batches]))

    print_rows = []
    for label, results in rows:
        travel = sum(t for t, _ in results) / len(results)
        carried = sum(c for _, c in results) / len(results)
        print_rows.append((label, travel, carried))

    if args.latex:
        for label, travel, carried in print_rows:
            name = "Sequential" if label == "sequential" else f"$\\lambda={label:g}$"
            print(f"{name} & {travel:.2f} & {carried:.2f} \\\\")
        return 0

    print(f"{N_BATCHES} random GPSR-shaped batches with a pick, dependencies on\n")
    print(f"{'':>12} {'travel m':>9} {'carried m':>10}")
    for label, travel, carried in print_rows:
        name = "sequential" if label == "sequential" else f"lambda={label:g}"
        print(f"{name:>12} {travel:>9.2f} {carried:>10.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
