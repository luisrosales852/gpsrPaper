"""Cross-command dependencies: the merged plan is conflict-equivalent to
running the commands one by one.

Pure Python — no py_trees, no venv needed:

    python3 -m tests.test_dependencies      # or: pytest tests/test_dependencies.py

Random batches are GPSR-shaped (pick/deliver, count, describe, find, guide,
follow) over a small apartment, so different commands keep touching the same
places. On each batch:

  * the progress-vector DP and the Held-Karp oracle, both given the same
    dependency table, reach the same optimal cost;
  * the plan passes ``conflict_order_preserved`` (an action-level re-check
    independent of the segment-level table) and the other invariants;
  * dependencies never make travel *shorter* (they only remove options).

It also reports how often travel-only interleaving (dependencies=False)
breaks conflict order — the "answer errors" the abstract talks about.
"""

import json
import os
import random
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gpsr_merger.merger import (  # noqa: E402
    _annotate_effects,
    _build_holding_after,
    _build_needs,
    _cost_matrix,
    _decompose,
    _euclidean_distance,
    _optimal_schedule,
    _room_table,
    merge,
)
from tests._heldkarp_oracle import heldkarp_schedule  # noqa: E402
from tests._merger_helpers import (  # noqa: E402
    build_commands,
    conflict_order_preserved,
    gripper_invariant_holds,
    make_locator,
    make_room_of,
    merge_kwargs,
    non_goto_actions_preserved,
    per_command_order_preserved,
)

SCENARIOS = os.path.join(ROOT, "scenarios", "merger.json")
VISUAL = os.path.join(ROOT, "sim", "scenarios_visual.json")
N_RANDOM = 300
ORACLE_MAX_M = 11
SEED = 20261112

# A small two-room flat: placements and beacons inside rooms, plus the rooms.
ROOMS = {
    "kitchen table": "kitchen", "counter": "kitchen", "trash bin": "kitchen",
    "couch": "living room", "side table": "living room", "bookshelf": "living room",
}
PLACES = list(ROOMS) + ["kitchen", "living room", "start"]
COORDS = {
    "kitchen table": [0, 0], "counter": [1.5, 1], "trash bin": [-1, 2], "kitchen": [0.5, 1],
    "couch": [8, 0], "side table": [9, 2], "bookshelf": [7, 3], "living room": [8, 1.5],
    "start": [4, -3],
}


def _g(place):
    return {"action": "go_to", "location_to_go": place}


def _random_command(rng):
    """One GPSR-shaped command as 1-3 segments."""
    kind = rng.choice(["fetch", "count", "describe", "find_tell", "guide", "follow"])
    if kind == "fetch":  # take X from P and deliver / place / trash it
        src = rng.choice(["kitchen table", "counter", "side table", "bookshelf", "kitchen"])
        dst = rng.choice(["start", "couch", "side table", "trash bin", "kitchen table"])
        tail = {"action": rng.choice(["give_object", "place_object"])}
        return [_g(src), {"action": "pick_object", "object_to_pick": "x"}, _g(dst), tail]
    if kind == "count":
        place = rng.choice(PLACES[:-1])
        target = rng.choice(["cups", "people"])
        return [_g(place), {"action": "count", "target_to_count": target},
                _g("start"), {"action": "say_with_context", "user_instruction": "count"}]
    if kind == "describe":
        place = rng.choice(["kitchen table", "counter", "side table", "bookshelf"])
        return [_g(place), {"action": "get_visual_info", "measure": "biggest"},
                _g("start"), {"action": "say_with_context", "user_instruction": "biggest"}]
    if kind == "find_tell":
        place = rng.choice(["couch", "living room", "kitchen", "bookshelf"])
        return [_g(place), {"action": "find_person", "attribute_value": "waving"},
                {"action": "get_person_info", "info_type": "name"},
                _g("start"), {"action": "say_with_context", "user_instruction": "name"}]
    if kind == "guide":
        src = rng.choice(["couch", "counter", "bookshelf"])
        dst = rng.choice(["kitchen", "living room"])
        return [_g(src), {"action": "find_person_by_name", "name": "Jane"},
                {"action": "guide_person_to", "destination_room": dst}]
    # follow, sometimes with no destination
    dest = rng.choice(["kitchen", "living room", None])
    follow = {"action": "follow_person_until"}
    if dest:
        follow["destination"] = dest
    return [_g(rng.choice(["couch", "kitchen table"])), {"action": "find_person"}, follow]


def _random_batch(rng):
    while True:
        cmds = [_random_command(rng) for _ in range(rng.randint(2, 4))]
        n_segs = sum(sum(1 for a in c if a["action"] == "go_to") for c in cmds)
        if n_segs <= ORACLE_MAX_M:
            return cmds


def _plan_parts(commands, room_of):
    per_cmd = [_decompose(c, cmd) for c, cmd in enumerate(commands)]
    for segs, cmd in zip(per_cmd, commands):
        _annotate_effects(segs, cmd)
    flat = [s for segs in per_cmd for s in segs]
    seg_ids, gi = [], 0
    for segs in per_cmd:
        seg_ids.append(list(range(gi, gi + len(segs))))
        gi += len(segs)
    needs = _build_needs(per_cmd, _room_table(flat, room_of))
    return flat, seg_ids, _build_holding_after(per_cmd), needs


def _travel(schedule, flat, distance, has_origin):
    seed, edge = _cost_matrix(flat, distance, has_origin)
    return seed[schedule[0]] + sum(edge[a][b] for a, b in zip(schedule, schedule[1:]))


def _check_invariants(plan, commands, room_of):
    A = plan.actions
    assert conflict_order_preserved(A, commands, room_of), "conflict order broken"
    assert per_command_order_preserved(A, len(commands))
    assert non_goto_actions_preserved(plan, commands)


def test_random_batches():
    rng = random.Random(SEED)
    room_of = make_room_of(ROOMS)
    locator = make_locator(COORDS)
    with_conflicts = travel_only_broken = 0
    extra_travel = []
    for _ in range(N_RANDOM):
        cmds_json = _random_batch(rng)
        commands = build_commands(cmds_json)
        origin = (rng.uniform(0, 9), rng.uniform(-3, 3))
        distance = _euclidean_distance(locator, origin)

        # 1. DP == oracle under the same dependency table
        flat, seg_ids, holding_after, needs = _plan_parts(commands, room_of)
        new = _optimal_schedule(flat, seg_ids, holding_after, distance, True, needs)
        old = heldkarp_schedule(flat, holding_after, distance, True, needs)
        if old is not None:
            c_new, c_old = _travel(new, flat, distance, True), _travel(old, flat, distance, True)
            assert abs(c_new - c_old) < 1e-3, (c_new, c_old, cmds_json)

        # 2. merge(): invariants hold, and dependencies never shorten travel
        with_deps = merge(commands, locator=locator, origin=origin, room_of=room_of)
        travel_only = merge(commands, locator=locator, origin=origin, room_of=room_of,
                            dependencies=False)
        if gripper_invariant_holds(with_deps.actions):
            _check_invariants(with_deps, commands, room_of)

        if any(req for per_cmd in needs for req in per_cmd):
            with_conflicts += 1
            if not conflict_order_preserved(travel_only.actions, commands, room_of):
                travel_only_broken += 1
        d_dep = _plan_travel(with_deps, locator, origin)
        d_free = _plan_travel(travel_only, locator, origin)
        assert d_dep >= d_free - 1e-6, (d_dep, d_free, cmds_json)
        extra_travel.append(d_dep - d_free)

    print(f"random: {with_conflicts}/{N_RANDOM} batches have a cross-command conflict; "
          f"travel-only breaks conflict order in {travel_only_broken} of them; "
          f"dependencies cost {sum(extra_travel) / len(extra_travel):.2f} m extra on average")
    assert with_conflicts > N_RANDOM // 4  # the generator really does make conflicts
    assert travel_only_broken > 0          # ...and travel-only really does break them


def _plan_travel(plan, locator, origin):
    last, total = origin, 0.0
    for pa in plan.actions:
        if pa.action.action == "go_to":
            xy = locator(pa.action.location_to_go)
            if xy is not None:
                if last is not None:
                    total += ((xy[0] - last[0]) ** 2 + (xy[1] - last[1]) ** 2) ** 0.5
                last = xy
    return total


def test_every_scenario_with_dependencies_keeps_conflict_order():
    with open(SCENARIOS) as f:
        scenarios = json.load(f)
    for sc in scenarios:
        kwargs = merge_kwargs(sc)
        if kwargs.get("dependencies") is False:
            continue
        commands = build_commands(sc["commands"])
        plan = merge(commands, **kwargs)
        assert conflict_order_preserved(plan.actions, commands, kwargs.get("room_of")), sc["name"]


def test_conflict_free_showcases_keep_their_plan():
    """No shared places in the sim showcases -> dependencies change nothing."""
    with open(VISUAL) as f:
        visual = json.load(f)
    for sc in visual:
        commands = build_commands(sc["commands"])
        origin = tuple(sc["origin"])
        a = merge(commands, origin=origin)
        b = merge(commands, origin=origin, dependencies=False)
        key = lambda p: [(x.source_cmd, x.source_idx) for x in p.actions]  # noqa: E731
        assert key(a) == key(b), sc["name"]


def test_needs_only_point_at_earlier_commands():
    """Constraints only ever say 'wait for an earlier-issued command', which
    is why per-command sequential order always satisfies them."""
    rng = random.Random(SEED + 1)
    for _ in range(100):
        commands = build_commands(_random_batch(rng))
        _, _, _, needs = _plan_parts(commands, make_room_of(ROOMS))
        for c, per_cmd in enumerate(needs):
            for req in per_cmd:
                assert all(d < c for d, _ in req)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS  {name}")
