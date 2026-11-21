"""Cross-command dependencies on random GPSR-shaped batches.

    python3 -m tests.test_dependencies      # or: pytest tests/test_dependencies.py
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
    planned_travel,
    merge_kwargs,
    non_goto_actions_preserved,
    per_command_order_preserved,
)

SCENARIOS = os.path.join(ROOT, "scenarios", "merger.json")
VISUAL = os.path.join(ROOT, "sim", "scenarios_visual.json")
N_RANDOM = 300
ORACLE_MAX_M = 11
SEED = 20261112

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
    kind = rng.choice(["fetch", "count", "describe", "find_tell", "guide", "follow", "here"])
    if kind == "here":  # no leading go_to: starts where the robot is
        return [{"action": "find_person", "attribute_value": "waving"},
                {"action": "get_person_info", "info_type": "name"},
                _g(rng.choice(PLACES[:-1])),
                {"action": "say_with_context", "user_instruction": "name"}]
    if kind == "fetch":
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
    dest = rng.choice(["kitchen", "living room", None])
    follow = {"action": "follow_person_until"}
    if dest:
        follow["destination"] = dest
    return [_g(rng.choice(["couch", "kitchen table"])), {"action": "find_person"}, follow]


def _random_batch(rng):
    while True:
        cmds = [_random_command(rng) for _ in range(rng.randint(2, 4))]
        n_segs = sum(sum(1 for a in c if a["action"] == "go_to") + (c[0]["action"] != "go_to")
                     for c in cmds)
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

        flat, seg_ids, holding_after, needs = _plan_parts(commands, room_of)
        new = _optimal_schedule(flat, seg_ids, holding_after, distance, True, needs)
        old = heldkarp_schedule(flat, holding_after, distance, True, needs)
        if old is not None:
            c_new, c_old = _travel(new, flat, distance, True), _travel(old, flat, distance, True)
            assert abs(c_new - c_old) < 1e-3, (c_new, c_old, cmds_json)

        with_deps = merge(commands, locator=locator, origin=origin, room_of=room_of)
        travel_only = merge(commands, locator=locator, origin=origin, room_of=room_of,
                            dependencies=False)
        if gripper_invariant_holds(with_deps.actions):
            _check_invariants(with_deps, commands, room_of)

        if any(req for per_cmd in needs for req in per_cmd):
            with_conflicts += 1
            if not conflict_order_preserved(travel_only.actions, commands, room_of):
                travel_only_broken += 1
        d_dep = planned_travel(with_deps.actions, commands, distance, origin is not None)
        d_free = planned_travel(travel_only.actions, commands, distance, origin is not None)
        assert d_dep >= d_free - 1e-6, (d_dep, d_free, cmds_json)
        # The cost the DP minimised equals the travel re-derived from the plan.
        assert abs(_travel(new, flat, distance, True) - d_dep) < 1e-6, cmds_json
        extra_travel.append(d_dep - d_free)

    print(f"random: {with_conflicts}/{N_RANDOM} batches have a cross-command conflict; "
          f"travel-only breaks conflict order in {travel_only_broken} of them; "
          f"dependencies cost {sum(extra_travel) / len(extra_travel):.2f} m extra on average")
    assert with_conflicts > N_RANDOM // 4
    assert travel_only_broken > 0


def test_every_scenario_with_dependencies_keeps_conflict_order():
    with open(SCENARIOS) as f:
        scenarios = json.load(f)
    for sc in scenarios:
        kwargs = merge_kwargs(sc)
        if kwargs.get("dependencies") is False:
            continue
        commands = build_commands(sc["commands"])
        plan = merge(commands, **kwargs)
        assert conflict_order_preserved(plan.actions, commands, kwargs.get("room_of"),
                                        kwargs.get("origin_name")), sc["name"]


def test_conflict_free_showcases_keep_their_plan():
    with open(VISUAL) as f:
        visual = json.load(f)
    for sc in visual:
        commands = build_commands(sc["commands"])
        origin = tuple(sc["origin"])
        a = merge(commands, origin=origin)
        b = merge(commands, origin=origin, dependencies=False)
        key = lambda p: [(x.source_cmd, x.source_idx) for x in p.actions]  # noqa: E731
        assert key(a) == key(b), sc["name"]


def _positions(actions):
    """Where the robot is before each action: a name, "@start" or None (unknown)."""
    pos, out = "@start", []
    for pa in actions:
        out.append(pos)
        kind = pa.action.action
        if kind == "go_to":
            pos = pa.action.location_to_go
        elif kind in ("guide_person_to", "follow_person_until"):
            dest = getattr(pa.action, "destination_room" if kind == "guide_person_to" else "destination", None)
            pos = dest if dest else None
    return out


def _check_inserted_go_tos(plan, cmds_json):
    """Inserted go_to(origin) only where the robot is elsewhere, right before the
    command's first action; each headless command's fallback starts with one."""
    A = plan.actions
    n = 0
    for i, (pa, pos) in enumerate(zip(A, _positions(A))):
        if pa.source_idx != -1:
            continue
        n += 1
        assert pos not in ("@start", "origin"), (cmds_json, i)  # never a go_to to where it is
        nxt = A[i + 1]
        assert nxt.source_cmd == pa.source_cmd and nxt.source_idx == 0, (cmds_json, i)
    for c, fb in enumerate(plan.fallback):
        headless = cmds_json[c][0]["action"] != "go_to"
        assert (fb[0].source_idx == -1) == headless, (cmds_json, c)
    return n


def test_origin_name_only_adds_the_drive_back():
    """Travel-only, naming the known start never changes the schedule or its cost;
    it only inserts a go_to back to it. With dependencies, acting at the start
    instead of at an unknown place only removes constraints, so travel never grows."""
    rng = random.Random(SEED + 2)
    room_of = make_room_of(ROOMS)
    inserted = improved = 0
    saved = 0.0
    work = lambda p: [(x.source_cmd, x.source_idx) for x in p.actions  # noqa: E731
                      if x.action.action != "go_to"]
    key = lambda p: [(x.source_cmd, x.source_idx) for x in p.actions]  # noqa: E731
    for _ in range(N_RANDOM):
        cmds_json = _random_batch(rng)
        commands = build_commands(cmds_json)
        origin = (rng.uniform(0, 9), rng.uniform(-3, 3))
        locator = make_locator({**COORDS, "origin": list(origin)})
        distance = _euclidean_distance(locator, origin)
        kw = dict(locator=locator, origin=origin, room_of=room_of)
        travel = lambda p: planned_travel(p.actions, commands, distance, True)  # noqa: E731

        plain = merge(commands, dependencies=False, **kw)
        named = merge(commands, dependencies=False, origin_name="origin", **kw)
        assert work(plain) == work(named), cmds_json
        assert abs(travel(plain) - travel(named)) < 1e-6, cmds_json
        inserted += _check_inserted_go_tos(named, cmds_json)

        plain = merge(commands, **kw)
        named = merge(commands, origin_name="origin", **kw)
        assert travel(named) <= travel(plain) + 1e-6, cmds_json
        assert conflict_order_preserved(named.actions, commands, room_of, "origin"), cmds_json
        assert per_command_order_preserved(named.actions, len(commands))
        assert non_goto_actions_preserved(named, commands)
        inserted += _check_inserted_go_tos(named, cmds_json)
        if travel(named) < travel(plain) - 1e-6:
            improved += 1
            saved += travel(plain) - travel(named)

        # origin=None + a resolvable name plans exactly like the known origin.
        unknown = merge(commands, locator=locator, origin=None, room_of=room_of, origin_name="origin")
        assert key(unknown) == key(named), cmds_json
    print(f"origin_name: {inserted} inserted go_tos; acting at the start shortens "
          f"{improved}/{N_RANDOM} plans, by {saved / max(improved, 1):.2f} m on average")
    assert inserted > 0 and improved > 0


def test_needs_only_point_at_earlier_commands():
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
