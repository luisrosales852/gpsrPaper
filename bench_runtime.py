#!/usr/bin/env python3
"""Planner runtime on random GPSR-shaped batches (paper Table "Planner runtime").

    python3 bench_runtime.py            # plain table
    python3 bench_runtime.py --latex    # rows for main.tex
"""

import argparse
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gpsr_merger.merger import (  # noqa: E402
    _STATE_CAP,
    _build_holding_after,
    _decompose,
    _euclidean_distance,
    _optimal_schedule,
    _state_count,
    merge,
)
from tests._merger_helpers import build_commands, make_locator  # noqa: E402

# Segments per command. The first row is the largest GPSR batch (n=3, L_c<=4).
CONFIGS = [
    (4, 4, 4),
    (3, 3, 3, 3),
    (5, 5, 4, 4),
    (5, 5, 4, 4, 4),
    (5, 5, 4, 4, 4, 4),
    (4,) * 8,
]
N_PLACES = 12
SEED = 20261120


def _command(rng, n_segs):
    """n_segs go_to-led segments, with a pick ... place pair and info actions."""
    pick_at = rng.randrange(n_segs)
    place_at = rng.randrange(pick_at, n_segs)
    acts = []
    for j in range(n_segs):
        acts.append({"action": "go_to", "location_to_go": f"P{rng.randrange(N_PLACES)}"})
        if j == pick_at:
            acts.append({"action": "pick_object", "object_to_pick": "x"})
        if j == place_at:
            acts.append({"action": "place_object"})
        if rng.random() < 0.5:
            acts.append({"action": "count", "target_to_count": "y"})
    return acts


def _time_dp(commands, distance, repeats):
    per_cmd = [_decompose(c, cmd) for c, cmd in enumerate(commands)]
    flat = [s for segs in per_cmd for s in segs]
    seg_ids, gi = [], 0
    for segs in per_cmd:
        seg_ids.append(list(range(gi, gi + len(segs))))
        gi += len(segs)
    holding_after = _build_holding_after(per_cmd)
    best = float("inf")
    for _ in range(repeats):
        t0 = time.perf_counter()
        _optimal_schedule(flat, seg_ids, holding_after, distance, True)
        best = min(best, time.perf_counter() - t0)
    return best


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--latex", action="store_true")
    ap.add_argument("--repeats", type=int, default=3)
    args = ap.parse_args()

    rng = random.Random(SEED)
    coords = {f"P{i}": [rng.uniform(0, 12), rng.uniform(0, 8)] for i in range(N_PLACES)}
    locator = make_locator(coords)
    origin = (0.0, 0.0)
    distance = _euclidean_distance(locator, origin)

    rows = []
    for lens in CONFIGS:
        commands = build_commands([_command(rng, L) for L in lens])
        prod = _state_count(lens) // len(lens)
        capped = _state_count(lens) > _STATE_CAP
        t = _time_dp(commands, distance, 1 if capped else args.repeats)
        t0 = time.perf_counter()
        merge(commands, locator=locator, origin=origin)
        t_merge = time.perf_counter() - t0
        rows.append((sum(lens), len(lens), prod, 2 ** sum(lens), t, capped, t_merge))

    if args.latex:
        for M, n, prod, subset, t, capped, _ in rows:
            mark = "$^\\dagger$" if capped else ""
            print(f"{M} & {n} & {prod:,} & {subset:,} & {t * 1e3:.1f}{mark} \\\\".replace(",", "\\,"))
        return 0

    print(f"state cap: n * prod(L_c+1) <= {_STATE_CAP:,}\n")
    print(f"{'M':>3} {'n':>2} {'prod(L+1)':>10} {'2^M':>14} {'DP ms':>8} {'merge() ms':>11}  merge() path (deps on)")
    for M, n, prod, subset, t, capped, t_merge in rows:
        path = "sequential (above cap)" if capped else "DP"
        print(f"{M:>3} {n:>2} {prod:>10,} {subset:>14,} {t * 1e3:>8.1f} {t_merge * 1e3:>11.1f}  {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
