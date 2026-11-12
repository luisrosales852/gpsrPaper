"""Progress-vector DP vs the Held-Karp oracle: same optimal cost.

    python3 -m tests.test_progress_dp      # or: pytest tests/test_progress_dp.py
"""

import json
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gpsr_merger.merger import (  # noqa: E402
    _build_holding_after,
    _cost_matrix,
    _decompose,
    _euclidean_distance,
    _optimal_schedule,
)
from tests._heldkarp_oracle import heldkarp_schedule  # noqa: E402
from tests._merger_helpers import build_commands, make_distance, make_locator  # noqa: E402

SCENARIOS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scenarios", "merger.json")
ORACLE_MAX_M = 12  # Held-Karp is O(2^M * M^2); keep the oracle fast
N_RANDOM = 400
SEED = 20261111


def _prepare(commands):
    per_cmd = [_decompose(c, cmd) for c, cmd in enumerate(commands)]
    flat = [s for segs in per_cmd for s in segs]
    seg_ids, gi = [], 0
    for segs in per_cmd:
        seg_ids.append(list(range(gi, gi + len(segs))))
        gi += len(segs)
    return flat, seg_ids, _build_holding_after(per_cmd)


def _travel(schedule, flat, distance, has_origin):
    seed, edge = _cost_matrix(flat, distance, has_origin)
    if not schedule:
        return 0.0
    return seed[schedule[0]] + sum(edge[a][b] for a, b in zip(schedule, schedule[1:]))


def _feasible(schedule, flat, holding_after):
    n = len(holding_after)
    k = [0] * n
    expected = {}
    for u, s in enumerate(flat):
        expected.setdefault(s.cmd, []).append(u)
    for u in schedule:
        c = flat[u].cmd
        if expected[c][k[c]] != u:
            return False
        holders = [d for d in range(n) if holding_after[d][k[d]]]
        if holders and holders[0] != c and flat[u].ends_holding is not None:
            return False
        k[c] += 1
    return sorted(schedule) == list(range(len(flat)))


def _compare(commands, distance, has_origin):
    flat, seg_ids, holding_after = _prepare(commands)
    new = _optimal_schedule(flat, seg_ids, holding_after, distance, has_origin)
    old = heldkarp_schedule(flat, holding_after, distance, has_origin)
    assert _feasible(new, flat, holding_after), "progress DP returned an infeasible schedule"
    if old is None:
        return None
    c_new = _travel(new, flat, distance, has_origin)
    c_old = _travel(old, flat, distance, has_origin)
    assert abs(c_new - c_old) < 1e-3, f"cost mismatch: progress={c_new} heldkarp={c_old}"
    return new == old


# ---------------------------------------------------------------------------
# Random batches
# ---------------------------------------------------------------------------
LOCS = [f"L{i}" for i in range(8)]


def _random_batch(rng):
    coords = {name: [rng.uniform(0, 12), rng.uniform(0, 8)] for name in LOCS}
    names = LOCS + ["unknown_place"]  # not in coords
    while True:
        n = rng.randint(2, 4)
        lens = [rng.randint(1, 4) for _ in range(n)]
        if sum(lens) <= ORACLE_MAX_M:
            break
    commands = []
    for L in lens:
        pick_at = place_at = None
        if rng.random() < 0.6:
            pick_at = rng.randrange(L)
            place_at = rng.randrange(pick_at, L)
        acts = []
        for j in range(L):
            acts.append({"action": "go_to", "location_to_go": rng.choice(names)})
            if j == pick_at:
                acts.append({"action": "pick_object", "object_to_pick": "obj"})
            if j == place_at:
                acts.append({"action": "place_object"})
            if rng.random() < 0.5:
                acts.append({"action": "count", "target_to_count": "x"})
        commands.append(acts)
    origin = None if rng.random() < 0.3 else (rng.uniform(0, 12), rng.uniform(0, 8))
    return commands, coords, origin


def test_random_batches_match_heldkarp():
    rng = random.Random(SEED)
    same = 0
    for _ in range(N_RANDOM):
        cmds_json, coords, origin = _random_batch(rng)
        distance = _euclidean_distance(make_locator(coords), origin)
        same += bool(_compare(build_commands(cmds_json), distance, origin is not None))
    print(f"random: {same}/{N_RANDOM} identical schedules (rest are exact-cost ties)")


def test_scenarios_match_heldkarp():
    with open(SCENARIOS) as f:
        scenarios = json.load(f)
    checked = 0
    for sc in scenarios:
        commands = build_commands(sc["commands"])
        flat, _, _ = _prepare(commands)
        if len(flat) > ORACLE_MAX_M:
            continue
        origin = tuple(sc["origin"]) if sc.get("origin") is not None else None
        if "distance_overrides" in sc:
            distance = make_distance(sc.get("coords", {}), sc["distance_overrides"], origin)
        else:
            distance = _euclidean_distance(make_locator(sc.get("coords", {})), origin)
        _compare(commands, distance, origin is not None)
        checked += 1
    assert checked > 20


def test_gpsr_batch_is_fast():
    rng = random.Random(1)
    cmds = [
        sum(([{"action": "go_to", "location_to_go": f"L{rng.randrange(8)}"},
              {"action": "count", "target_to_count": "x"}] for _ in range(4)), [])
        for _ in range(3)
    ]
    coords = {f"L{i}": [rng.uniform(0, 12), rng.uniform(0, 8)] for i in range(8)}
    flat, seg_ids, holding_after = _prepare(build_commands(cmds))
    distance = _euclidean_distance(make_locator(coords), None)
    t0 = time.perf_counter()
    _optimal_schedule(flat, seg_ids, holding_after, distance, False)
    assert time.perf_counter() - t0 < 0.05


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS  {name}")
