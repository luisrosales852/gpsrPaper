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
    _flatten,
    _gripper_holder,
    _optimal_schedule,
    merge,
)
from tests._heldkarp_oracle import heldkarp_schedule  # noqa: E402
from tests._merger_helpers import (  # noqa: E402
    build_commands, make_distance, make_locator, planned_travel)

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


LOCS = [f"L{i}" for i in range(8)]


def _random_batch(rng):
    coords = {name: [rng.uniform(0, 12), rng.uniform(0, 8)] for name in LOCS}
    names = LOCS + ["unknown_place"]  # not in coords
    while True:
        n = rng.randint(2, 4)
        lens = [rng.randint(1, 4) for _ in range(n)]
        heads = [rng.random() < 0.2 for _ in range(n)]  # actions before the first go_to
        if sum(lens) + sum(heads) <= ORACLE_MAX_M:
            break
    commands = []
    for L, head in zip(lens, heads):
        pick_at = place_at = None
        if rng.random() < 0.6:
            pick_at = rng.randrange(L)
            place_at = rng.randrange(pick_at, L)
        acts = [{"action": "answer_question"}] if head else []
        for j in range(L):
            acts.append({"action": "go_to", "location_to_go": rng.choice(names)})
            if j == pick_at:
                acts.append({"action": "pick_object", "object_to_pick": "obj"})
            if j == place_at:
                acts.append({"action": "place_object"})
            if rng.random() < 0.5:
                acts.append({"action": "count", "target_to_count": "x"})
            if rng.random() < 0.25:  # escort: the segment exits somewhere else
                dest = rng.choice(names + [None])
                if rng.random() < 0.5:
                    acts.append({"action": "guide_person_to", "destination_room": dest})
                else:
                    acts.append({"action": "follow_person_until", "destination": dest})
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


_HEADS = [[], [{"action": "answer_question"}], [{"action": "pick_object", "object_to_pick": "o"}],
          [{"action": "find_person"}, {"action": "follow_person_until", "destination": "L1"}]]


def _named_start_batch(rng):
    """2-3 commands, mostly starting before their first go_to, M <= 7."""
    while True:
        commands = []
        for _ in range(rng.randint(2, 3)):
            head = list(rng.choice(_HEADS))
            holding = bool(head) and head[0]["action"] == "pick_object"
            acts = head
            for j in range(rng.randint(1, 2)):
                acts.append({"action": "go_to", "location_to_go": rng.choice(LOCS)})
                if holding and (j == 1 or rng.random() < 0.5):
                    acts.append({"action": "place_object"})
                    holding = False
                elif not holding and rng.random() < 0.3:
                    acts.append({"action": "pick_object", "object_to_pick": "o"})
                    holding = True
                if rng.random() < 0.3:
                    acts.append({"action": "guide_person_to", "destination_room": rng.choice(LOCS)})
            if holding:
                acts.append({"action": "place_object"})
            commands.append(acts)
        flat, _, _ = _prepare(build_commands(commands))
        if len(flat) <= 7 and any(c[0]["action"] != "go_to" for c in commands):
            return commands


def _asymmetric(coords, pose):
    """Euclidean + 15% one way: catches a return priced in the wrong direction."""
    def d(a, b):
        pa = pose if a is None else coords[a]
        pb = coords[b]
        e = ((pa[0] - pb[0]) ** 2 + (pa[1] - pb[1]) ** 2) ** 0.5
        return e * (1.15 if (a or "") < b else 1.0)
    return d


def _feasible_schedules(flat, seg_ids, holding_after):
    n = len(seg_ids)

    def rec(k, out):
        if all(k[c] == len(seg_ids[c]) for c in range(n)):
            yield out
            return
        holder = _gripper_holder(k, holding_after)
        for c in range(n):
            if k[c] == len(seg_ids[c]):
                continue
            u = seg_ids[c][k[c]]
            if holder is not None and holder != c and flat[u].ends_holding is not None:
                continue
            yield from rec(k[:c] + (k[c] + 1,) + k[c + 1:], out + [u])

    yield from rec(tuple([0] * n), [])


def test_named_start_is_priced_as_driven():
    """With origin_name, the robot's pose off the named start and an asymmetric
    metric, the merged plan drives no more than the best feasible order, where
    each order is scored by the distance its flattened plan really drives."""
    rng = random.Random(SEED + 3)
    for _ in range(300):
        cmds_json = _named_start_batch(rng)
        coords = {name: [rng.uniform(0, 12), rng.uniform(0, 8)] for name in LOCS}
        coords["S"] = [rng.uniform(0, 12), rng.uniform(0, 8)]
        pose = (coords["S"][0] + rng.uniform(-1.5, 1.5), coords["S"][1] + rng.uniform(-1.5, 1.5))
        distance = _asymmetric(coords, pose)
        commands = build_commands(cmds_json)
        flat, seg_ids, holding_after = _prepare(commands)
        driven = lambda acts: planned_travel(acts, commands, distance, True, "S")  # noqa: E731
        best = min(driven(_flatten(s, flat, commands, "S"))
                   for s in _feasible_schedules(flat, seg_ids, holding_after))
        plan = merge(commands, origin=pose, distance=distance, dependencies=False, origin_name="S")
        assert driven(plan.actions) <= best + 1e-6, (driven(plan.actions), best, cmds_json)


def test_holding_penalty_is_exact():
    """With holding_penalty > 0, the merged plan is the best feasible order
    under travel + penalty x distance driven while holding."""
    rng = random.Random(SEED + 4)
    changed = 0
    for _ in range(300):
        cmds_json = _named_start_batch(rng)
        coords = {name: [rng.uniform(0, 12), rng.uniform(0, 8)] for name in LOCS}
        coords["S"] = [rng.uniform(0, 12), rng.uniform(0, 8)]
        pose = (coords["S"][0] + rng.uniform(-1.5, 1.5), coords["S"][1] + rng.uniform(-1.5, 1.5))
        distance = _asymmetric(coords, pose)
        penalty = rng.choice([0.25, 0.5, 1.0, 2.0])
        commands = build_commands(cmds_json)
        flat, seg_ids, holding_after = _prepare(commands)
        cost = lambda acts: planned_travel(acts, commands, distance, True, "S", penalty)  # noqa: E731
        best = min(cost(_flatten(s, flat, commands, "S"))
                   for s in _feasible_schedules(flat, seg_ids, holding_after))
        kw = dict(origin=pose, distance=distance, dependencies=False, origin_name="S")
        plan = merge(commands, holding_penalty=penalty, **kw)
        assert cost(plan.actions) <= best + 1e-6, (cost(plan.actions), best, penalty, cmds_json)
        changed += cost(merge(commands, **kw).actions) > best + 1e-6
    assert changed > 0  # the penalty does change some plans


def test_return_is_priced_exactly():
    """With return_to, the merged plan is the best feasible order counting the
    drive back, weighted like a hop (holding penalty, navigation overhead)."""
    rng = random.Random(SEED + 6)
    changed = 0
    for _ in range(300):
        cmds_json = _named_start_batch(rng)
        if cmds_json[0][-1]["action"] == "place_object" and rng.random() < 0.3:
            cmds_json[0].pop()  # command 0 drives home holding
        coords = {name: [rng.uniform(0, 12), rng.uniform(0, 8)] for name in [*LOCS, "S"]}
        pose = (coords["S"][0] + rng.uniform(-1.5, 1.5), coords["S"][1] + rng.uniform(-1.5, 1.5))
        distance = _asymmetric(coords, pose)
        penalty, overhead = rng.choice([0.0, 0.5]), rng.choice([0.0, 2.0])
        commands = build_commands(cmds_json)
        flat, seg_ids, holding_after = _prepare(commands)

        def cost(actions):
            return planned_travel(actions, commands, distance, True, "S", penalty,
                                  navigation_overhead=overhead, return_to="S")

        best = min(cost(_flatten(s, flat, commands, "S"))
                   for s in _feasible_schedules(flat, seg_ids, holding_after))
        kw = dict(origin=pose, distance=distance, dependencies=False, origin_name="S",
                  holding_penalty=penalty, navigation_overhead=overhead)
        plan = merge(commands, return_to="S", **kw)
        assert cost(plan.actions) <= best + 1e-6, (cost(plan.actions), best, cmds_json)
        changed += cost(merge(commands, **kw).actions) > best + 1e-6
    assert changed > 0  # counting the return does change some plans


def test_navigation_overhead_matches_brute_force():
    rng = random.Random(SEED + 5)
    changed = 0
    for _ in range(150):
        coords = {name: [rng.uniform(0, 12), rng.uniform(0, 8)] for name in [*LOCS, "S"]}
        pose = (coords["S"][0] + 0.5, coords["S"][1])
        distance = _asymmetric(coords, pose)
        commands = build_commands(_named_start_batch(rng))
        flat, seg_ids, holding_after = _prepare(commands)
        orders = [_flatten(order, flat, commands, "S")
                  for order in _feasible_schedules(flat, seg_ids, holding_after)]
        kw = dict(origin=pose, distance=distance, dependencies=False, origin_name="S",
                  holding_penalty=0.5)
        base = merge(commands, **kw)
        overhead = 3.0

        def cost(actions):
            return planned_travel(actions, commands, distance, True, "S", 0.5,
                                  navigation_overhead=overhead)

        best = min(map(cost, orders))
        plan = merge(commands, **kw, navigation_overhead=overhead)
        assert cost(plan.actions) <= best + 1e-6
        assert plan.fallback == base.fallback
        changed += cost(base.actions) > best + 1e-6
    assert changed > 0


def test_navigation_overhead_favors_one_visit_to_a_shared_stop():
    commands = build_commands([
        [{"action": "go_to", "location_to_go": place} for place in stops]
        for stops in (("A", "B"), ("C", "B"))
    ])
    locator = make_locator({"A": [0, 0], "B": [5, 0], "C": [10, 0]})
    options = dict(locator=locator, origin=(0, 0), origin_name="A")
    default = merge(commands, **options)
    grouped = merge(commands, navigation_overhead=2.0, **options)
    assert [a.location for a in default.actions] == ["B", "C", "B"]
    assert [a.location for a in grouped.actions] == ["C", "B"]
    assert sorted(grouped.actions[-1].navigation_goals) == [(0, 1), (1, 1)]


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
