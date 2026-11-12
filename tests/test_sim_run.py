"""Assertions on a real simulated run.

These check the **event log**, never pixels. Each mirrors a static invariant
from ``tests/_merger_helpers.py``, but witnesses it at execution time in a world
with geometry, a gripper that can only hold one thing, and perception that fails
when the robot is not actually near its target.

Runs headless (no rendering), so it is fast enough for CI:

    .venv/bin/pytest tests/test_sim_run.py -q
    .venv/bin/python -m tests.test_sim_run      # same checks, no pytest needed
"""

import math
import os
import sys
import tempfile
from argparse import Namespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gpsr_merger.timeouts import GLOBAL_BUDGET_S  # noqa: E402
from sim.routing import (  # noqa: E402
    PERSON_RADIUS, ROBOT_RADIUS, VisGraphRouter, furniture_rects)
from sim.run_sim import load_scenario, run_once  # noqa: E402
from sim.world import World  # noqa: E402

ALL_KINDS = {
    "go_to", "pick_object", "place_object", "give_object", "find_person",
    "find_person_by_name", "get_person_info", "follow_person_until",
    "guide_person_to", "count", "get_visual_info", "answer_question",
    "say_with_context",
}

APARTMENT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         "sim", "apartment.json")
# Every run the collision tests sweep: both showcases, both orderings, plus one
# with an injected failure so the fallback branch's movements are covered too.
COLLISION_RUNS = [(sc, mode, None) for sc in ("showcase_all_leaves", "showcase_merge")
                  for mode in ("merged", "sequential")] + [
                  ("showcase_all_leaves", "merged", ("pick_object:1", "0.3:3"))]
# Slack for trail sampling (TRAIL_DT at cruise speed is ~4 cm, and the planner
# keeps MARGIN_M / PERSON_MARGIN_M on top of the true footprint).
_TOL = 1e-3

_cache = {}


def _args(out, fail=None):
    return Namespace(apartment=APARTMENT,
        width=920, height=720, fps=30, speed=2.5, camera="topdown",
        retries=2, budget=GLOBAL_BUDGET_S, fail=list(fail) if fail else None, no_video=True,
        print_tree=False, out=out)


def run(mode, scenario="showcase_all_leaves", fail=None):
    """One headless run per (scenario, mode, fail), memoised across tests."""
    key = (scenario, mode, fail)
    if key not in _cache:
        sc = load_scenario(scenario)
        with tempfile.TemporaryDirectory() as td:
            summary = run_once(sc, mode, os.path.join(td, mode), _args(td, fail))
            import json
            with open(os.path.join(td, mode, "run.json")) as f:
                _cache[key] = (summary, json.load(f), sc)
    return _cache[key]


def _ends(log):
    return [e for e in log["events"] if e["phase"] == "end"]


def test_every_non_goto_action_executed_exactly_once():
    """The formal version of 'the merger never drops a command'."""
    summary, log, sc = run("merged")
    for ci, actions in enumerate(sc["commands"]):
        want = {i for i, a in enumerate(actions) if a["action"] != "go_to"}
        got = [(e["cmd"], e["idx"]) for e in _ends(log)
               if e["cmd"] == ci and e["kind"] != "go_to"]
        assert sorted(i for _, i in got) == sorted(want), f"command {ci}: {got} != {want}"
        assert len(got) == len(set(got)), f"command {ci} executed an action twice"


def test_gripper_single_item_invariant_in_execution():
    """Sim-side witness: the gripper never holds two things, and every pick is
    released. ``pick_object`` hard-fails on a double pick, so a violating
    schedule would show up as a FAILURE status here, not just a failed assert."""
    summary, log, _ = run("merged")
    held = None
    for e in _ends(log):
        if e["kind"] == "pick_object" and e["status"] == "EXECUTION_SUCCESS":
            assert held is None, f"double pick at t={e['t']} (already holding {held})"
            held = e["cmd"]
        if e["kind"] in ("give_object", "place_object") and e["status"] == "EXECUTION_SUCCESS":
            assert held is not None, f"release with empty gripper at t={e['t']}"
            assert held == e["cmd"], "a pick was released by a different command"
            held = None
    assert held is None, "run ended still holding an object"


def test_per_command_order_preserved():
    summary, log, sc = run("merged")
    last = {}
    for e in _ends(log):
        assert e["idx"] >= last.get(e["cmd"], -1), f"c{e['cmd']} ran {e['idx']} out of order"
        last[e["cmd"]] = e["idx"]


def test_all_thirteen_leaf_behaviours_exercised():
    """The machine-checkable form of 'every leaf behaviour is showcased'."""
    summary, log, _ = run("merged")
    got = {e["kind"] for e in _ends(log)}
    assert got == ALL_KINDS, f"missing: {ALL_KINDS - got}; unexpected: {got - ALL_KINDS}"


def test_run_completes_without_falling_back():
    summary, log, _ = run("merged")
    assert summary["tree_status"] == "Status.SUCCESS"
    assert summary["actions_succeeded"] == summary["actions_executed"]
    assert all(e["branch"] == "interleaved" for e in log["events"])


def test_merged_travels_less_than_sequential():
    m, _, _ = run("merged")
    s, _, _ = run("sequential")
    assert m["actual_distance_m"] < s["actual_distance_m"], (
        f"merged {m['actual_distance_m']} m !< sequential {s['actual_distance_m']} m")
    assert m["planned_cost_m"] < s["planned_cost_m"]


def test_showcase_merge_finishes_sooner():
    """The demo scenario must earn its place: same actions, all succeed, and the
    merged run is clearly faster end-to-end — not just shorter on paper.
    Measured with collision-free routing: ~26% less time, ~39% less driving."""
    m, _, _ = run("merged", "showcase_merge")
    s, _, _ = run("sequential", "showcase_merge")
    for x in (m, s):
        assert x["actions_succeeded"] == x["actions_executed"] == m["actions_executed"]
    assert m["task_time_s"] <= 0.85 * s["task_time_s"], (m["task_time_s"], s["task_time_s"])
    assert m["actual_distance_m"] <= 0.70 * s["actual_distance_m"], (
        m["actual_distance_m"], s["actual_distance_m"])


def test_cost_model_gap_is_attributable_to_non_goto_displacement():
    """The planner's path-length model only accounts for ``go_to`` hops, so any
    gap between planned cost and driven distance must come from leaves that move
    the robot without being a ``go_to`` — follow/guide, and the pick stand-off."""
    summary, log, _ = run("merged")
    ev = log["events"]
    non_goto = 0.0
    for s, e in zip(ev[0::2], ev[1::2]):
        assert s["phase"] == "start" and e["phase"] == "end" and s["idx"] == e["idx"]
        if e["kind"] != "go_to":
            non_goto += e["distance_m"] - s["distance_m"]
    assert summary["cost_model_gap_m"] > 0.5, "expected a measurable gap in this scenario"
    assert non_goto > 0.5
    kinds = {e["kind"] for s, e in zip(ev[0::2], ev[1::2])
             if e["distance_m"] - s["distance_m"] > 0.01 and e["kind"] != "go_to"}
    assert kinds <= {"follow_person_until", "guide_person_to", "pick_object"}, kinds


def test_planned_cost_is_the_driven_metric():
    """``run_sim`` hands the router's path length to ``merge(distance=...)``, so
    when only ``go_to`` moves the robot, what the planner minimised is what the
    robot drove. showcase_merge has no follow/guide; the residue is the pick
    stand-off only."""
    for mode in ("merged", "sequential"):
        s, _, _ = run(mode, "showcase_merge")
        assert abs(s["cost_model_gap_m"]) < 0.02 * s["actual_distance_m"], (
            mode, s["cost_model_gap_m"], s["actual_distance_m"])


def test_every_location_is_in_free_space():
    """No named location may sit inside furniture, a wall or a person's
    footprint (people at their spawn positions) — otherwise the router would
    silently move the goal."""
    world = World(APARTMENT)
    router = VisGraphRouter(world, [p.home for p in world.people])
    bad = {k: v[:2] for k, v in world.locations.items() if router.free_point(v[:2]) != v[:2]}
    assert not bad, f"locations inside an obstacle: {bad}"


def _rect_gap(x, y, r):
    dx = max(r.x0 - x, 0.0, x - r.x1)
    dy = max(r.y0 - y, 0.0, y - r.y1)
    return math.hypot(dx, dy)


def test_robot_never_overlaps_furniture_or_walls():
    """Every trail sample keeps the robot's whole footprint (ROBOT_RADIUS, no
    safety margin) off every piece of furniture and inside the walls."""
    world = World(APARTMENT)
    rects = furniture_rects(world.furniture)
    (bx0, by0), (bx1, by1) = world.bounds
    for sc, mode, fail in COLLISION_RUNS:
        _, log, _ = run(mode, sc, fail)
        for i, (x, y, _c) in enumerate(log["trail"]):
            wall = min(x - bx0, bx1 - x, y - by0, by1 - y)
            assert wall >= ROBOT_RADIUS - _TOL, (sc, mode, fail, i, (x, y), "wall", wall)
            for r, f in zip(rects, world.furniture):
                g = _rect_gap(x, y, r)
                assert g >= ROBOT_RADIUS - _TOL, (sc, mode, fail, i, (x, y), f["name"], g)


def test_robot_keeps_clear_of_people():
    """Robot and person footprints never overlap, escorted person included."""
    world = World(APARTMENT)
    for sc, mode, fail in COLLISION_RUNS:
        _, log, _ = run(mode, sc, fail)
        assert len(log["people_trail"]) == len(log["trail"])
        for i, ((x, y, _c), people) in enumerate(zip(log["trail"], log["people_trail"])):
            for p, (px, py) in zip(world.people, people):
                d = math.hypot(x - px, y - py)
                assert d >= ROBOT_RADIUS + PERSON_RADIUS - _TOL, (
                    sc, mode, fail, i, (x, y), p.name, round(d, 3))


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS  {name}")
            except AssertionError as exc:
                fails += 1
                print(f"FAIL  {name}\n        {exc}")
    print(f"\n{'all checks passed' if not fails else f'{fails} check(s) failed'}")
    sys.exit(1 if fails else 0)
