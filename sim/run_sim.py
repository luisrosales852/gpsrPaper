#!/usr/bin/env python3
"""Run a GPSR batch in the MuJoCo apartment and record it.

The point of this script is what it does *not* do: it never re-implements
scheduling or execution. It calls ``gpsr_merger.merger.merge`` for the plan and
``gpsr_merger.bt_builder.build_tree`` for the executor, exactly as
``reference/gpsr_task_manager.GPSRTM._execute_interleaved_batch`` does on the
robot, and only swaps the subtask managers for simulated ones.

    python -m sim.run_sim --mode merged --dry-run
    python -m sim.run_sim --mode merged
    python -m sim.run_sim --mode both          # -> out/side_by_side.mp4 (sim/compose.py)
"""

import argparse
import json
import math
import os
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple

import mujoco
import py_trees

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gpsr_merger.bt_builder import ExecutionHooks, RecoveryPolicy, build_tree, render_tree_ascii  # noqa: E402
from gpsr_merger.merger import InterleavedPlan, merge  # noqa: E402
from gpsr_merger.status import Status  # noqa: E402
from gpsr_merger.timeouts import GLOBAL_BUDGET_S  # noqa: E402
from sim.compose import compose  # noqa: E402
from sim.handlers import FAILURE_REPORT_S, SimSubtaskHandler  # noqa: E402
from sim.mjcf import build_xml  # noqa: E402
from sim.recorder import Recorder  # noqa: E402
from sim.robot import SimRobot  # noqa: E402
from sim.routing import VisGraphRouter  # noqa: E402
from sim.world import World  # noqa: E402
from tests._merger_helpers import build_commands, describe_plan, planned_travel  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCENARIOS = [os.path.join(HERE, "scenarios_visual.json"),
             os.path.join(ROOT, "scenarios", "merger.json")]


def load_scenario(name: str) -> dict:
    for path in SCENARIOS:
        if not os.path.exists(path):
            continue
        for sc in json.load(open(path)):
            if sc["name"] == name:
                return sc
    raise SystemExit(f"scenario '{name}' not found in {SCENARIOS}")


def planning_router(world: World) -> VisGraphRouter:
    """The router as the world looks at planning time (everyone at home).

    Its ``distance`` — collision-free path length — is what ``merge()`` ranks
    orderings by, so the planner's cost and the robot's driven distance are
    measured in the same metric.
    """
    return VisGraphRouter(world, [p.home for p in world.people])


def merger_distance(world: World, router: VisGraphRouter,
                    origin: Optional[Tuple[float, float]]):
    """``merge(distance=...)`` over location names: the router's path length
    between the resolved points, ``None`` meaning the robot origin."""
    def distance(a: Optional[str], b: str) -> Optional[float]:
        pa = origin if a is None else world.locator(a)
        pb = world.locator(b)
        if pa is None or pb is None:
            return None
        return router.distance(pa, pb)

    return distance


def planned_cost(actions: Sequence[Any], commands: Sequence[Any], distance,
                 origin: Optional[Tuple[float, float]], origin_name: Optional[str] = None) -> float:
    """Unweighted planned travel before return, using the merger's metric."""
    return planned_travel(actions, commands, distance, origin is not None, origin_name)


def start_name(world: World, origin: Optional[Tuple[float, float]]) -> Optional[str]:
    """The apartment location the robot starts on, handed to ``merge(origin_name=...)``."""
    if origin is None:
        return None
    for name, (x, y, _yaw) in world.locations.items():
        if math.hypot(x - origin[0], y - origin[1]) < 1e-6:
            return name
    return None


def _plan(world, commands, origin, args):
    distance = merger_distance(world, planning_router(world), origin)
    origin_name = start_name(world, origin)
    price_return = getattr(args, "price_return", False)
    if price_return and origin_name is None:
        raise SystemExit("--price-return requires the scenario origin to be a named location")
    plan = merge(
        commands, origin=origin, distance=distance, room_of=world.room_of,
        origin_name=origin_name, holding_penalty=getattr(args, "holding_penalty", 0.0),
        navigation_overhead=getattr(args, "navigation_overhead", 0.0),
        return_to=origin_name if price_return else None,
    )
    return plan, distance, origin_name


def sequential_plan(plan: InterleavedPlan) -> InterleavedPlan:
    """The per-command baseline, straight out of ``merge()``.

    ``plan.fallback`` is already the original sequence of each command, so the
    baseline needs no new scheduling code: same executor, same handlers, same
    world — only the ordering differs, which is exactly the claim under test.
    """
    actions = [pa for per_cmd in plan.fallback for pa in per_cmd]
    if actions and actions[0].source_idx == -1:
        actions = actions[1:]  # the robot already stands on the start
    return InterleavedPlan(actions=actions, fallback=[], origin_name=plan.origin_name)


def run_once(sc: dict, mode: str, out_dir: str, args) -> dict:
    world = World(args.apartment)
    origin = tuple(sc["origin"]) if sc.get("origin") is not None else None
    commands = build_commands(sc["commands"])

    plan, distance, origin_name = _plan(world, commands, origin, args)
    exec_plan = sequential_plan(plan) if mode == "sequential" else plan
    policy = getattr(args, "recovery_policy", None) or (
        RecoveryPolicy.SKIP_DEFER if mode == "merged" else RecoveryPolicy.SEQUENTIAL_FALLBACK
    )
    policy = RecoveryPolicy(policy)

    model = mujoco.MjModel.from_xml_string(build_xml(world, args.width, args.height))
    data = mujoco.MjData(model)
    robot = SimRobot(model, data, world)
    if origin is not None:
        robot.set_pose(origin[0], origin[1], math.pi / 2)
    robot.sync()
    home = (*robot.xy, robot.yaw)
    robot.t = sc.get("elapsed_before_execution_s", 0.0)

    rec = Recorder(model, data, world, robot, out_dir, sc["name"],
                   "MERGED  (interleaved)" if mode == "merged" else "SEQUENTIAL  (per command)",
                   [c.commands for c in commands], width=args.width, height=args.height,
                   fps=args.fps, speed=args.speed, camera=args.camera,
                   video=not args.no_video)
    robot.on_tick = rec.tick

    failures: Dict[str, List[Status]] = {}
    for spec in args.fail or []:
        key, _, n = spec.partition(":")
        failures.setdefault(key, []).extend([Status.EXECUTION_ERROR] * int(n or 1))

    handler = SimSubtaskHandler(world, robot, recorder=rec, failures=failures,
                                router=VisGraphRouter(world))
    return_start = {}

    def on_phase(phase):
        rec.branch = phase
        if phase == "return":
            rec.current = None
            rec.leaf_line = "Returning to start"
            robot.clear_annotations()
            return_start.update(t=robot.t, distance_m=robot.distance_m)

    def return_home(deadline):
        reached = handler.return_to_start(home, deadline)
        return_start["finished_t"] = robot.t
        return reached

    root = build_tree(
        exec_plan,
        subtask_handlers=[handler],
        on_action_complete=handler.on_action_complete,
        retry_count=args.retries,
        global_budget_s=args.budget,
        is_completed=lambda pa: False,  # resume using the tree's successful attempts
        on_action_start=handler.on_action_start,
        recovery_policy=policy,
        execution_hooks=ExecutionHooks(
            clock=lambda: robot.t, deadline=args.budget,
            set_action_deadline=handler.set_action_deadline,
            gripper_state=handler.gripper_state, on_phase=on_phase,
            room_of=world.room_of,
            estimate_deferred=lambda steps: handler.estimate_deferred(steps, home),
            return_to_start=return_home,
            report_failure=handler.report_failure, failure_report_s=FAILURE_REPORT_S,
        ),
    )
    if args.print_tree:
        print(render_tree_ascii(root))

    tree = py_trees.trees.BehaviourTree(root)
    try:
        tree.setup(timeout=15.0)
    except Exception as e:  # noqa: BLE001 — mirrors GPSRTM, setup is best-effort
        print(f"  tree setup raised: {e}")

    terminal = (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE)
    while root.status not in terminal:
        tree.tick()

    elapsed = robot.t
    robot.clear_annotations()
    if not args.no_video:
        robot.idle(1.2)  # playback only, outside the measured execution

    extra = {
        "tree_status": str(root.status),
        "planner_priced_return": mode == "merged" and getattr(args, "price_return", False),
        "planner_navigation_overhead_m": (
            getattr(args, "navigation_overhead", 0.0) if mode == "merged" else 0.0
        ),
        "recovery_policy": policy.value,
        "deadline_s": args.budget,
        "elapsed_before_execution_s": sc.get("elapsed_before_execution_s", 0.0),
        "elapsed_s": round(elapsed, 3),
        "returned": root.returned,
        "final_xy": [round(coord, 3) for coord in robot.xy],
        "final_held": robot.held.kind if robot.held else None,
        "objects_xyz": {obj.name: [round(coord, 3) for coord in robot.object_xyz(obj)]
                        for obj in world.objects},
        "return_distance_m": round(robot.distance_m - return_start["distance_m"], 3),
        "return_finished_s": round(return_start.get("finished_t", return_start["t"]), 3),
        "work_distance_m": round(return_start["distance_m"], 3),
        "failures": [
            {"cmd": f.action.source_cmd, "idx": f.action.source_idx,
             "kind": f.action.action.action, "status": f.status.name, "phase": f.phase,
             "deferred": f.deferred, "reported": f.reported,
             "recovery_status": None if f.recovery_status is None else f.recovery_status.name}
            for f in root.failures
        ],
        "skipped": [{"cmd": c, "idx": i, "reason": reason}
                    for (c, i), reason in sorted(root.skipped.items())],
        "planned_cost_m": round(planned_cost(exec_plan.actions, commands, distance,
                                             origin, origin_name), 3),
        "n_plan_actions": len(exec_plan.actions),
        "command_switches": sum(
            1 for a, b in zip(exec_plan.actions, exec_plan.actions[1:])
            if a.source_cmd != b.source_cmd),
    }
    extra["task_time_s"] = round(return_start["t"], 3)
    extra["cost_model_gap_m"] = round(
        return_start["distance_m"] - extra["planned_cost_m"], 3)
    summary = rec.summary(extra)
    rec.write(extra)
    video = rec.close()
    summary["video"] = video
    return summary


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="showcase_merge",
                    help="showcase_merge (demo) or showcase_all_leaves (13-leaf coverage)")
    ap.add_argument("--mode", choices=["merged", "sequential", "both"], default="both")
    ap.add_argument("--recovery-policy", choices=[policy.value for policy in RecoveryPolicy],
                    help="recovery policy (default: skip-defer for merged, sequential-fallback for sequential)")
    ap.add_argument("--out", default=os.path.join(ROOT, "out"))
    ap.add_argument("--apartment", default=os.path.join(HERE, "apartment.json"))
    ap.add_argument("--camera", default="topdown", choices=["topdown", "iso"])
    ap.add_argument("--width", type=int, default=832)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--speed", type=float, default=3.0,
                    help="sim-seconds per video second")
    ap.add_argument("--retries", type=int, default=2)
    ap.add_argument("--budget", type=float, default=GLOBAL_BUDGET_S)
    ap.add_argument("--fail", action="append",
                    help="inject failures, e.g. --fail 0.1:2 (cmd.idx:count) or --fail pick_object:1")
    ap.add_argument("--no-video", action="store_true", help="event log only, no rendering")
    ap.add_argument("--print-tree", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="print the merged order and exit")
    ap.add_argument("--holding-penalty", type=float, default=0.0,
                    help="extra cost per metre driven while holding (paper's lambda)")
    ap.add_argument("--price-return", action="store_true",
                    help="include the final drive to the named start in the planner objective")
    ap.add_argument("--navigation-overhead", type=float, default=0.0, metavar="METRES",
                    help="distance-equivalent cost per nonzero navigation hop")
    args = ap.parse_args()

    sc = load_scenario(args.scenario)
    os.makedirs(args.out, exist_ok=True)

    if args.dry_run:
        world = World(args.apartment)
        commands = build_commands(sc["commands"])
        origin = tuple(sc["origin"]) if sc.get("origin") is not None else None
        plan, distance, origin_name = _plan(world, commands, origin, args)
        seq = sequential_plan(plan)
        print(f"scenario: {sc['name']}  ({len(commands)} commands, origin={origin})\n")
        print("MERGED     :", describe_plan(plan))
        print(f"  planned travel before return {planned_cost(plan.actions, commands, distance, origin, origin_name):.2f} m, "
              f"{len(plan.actions)} actions")
        print("\nSEQUENTIAL :", describe_plan(seq))
        print(f"  planned travel before return {planned_cost(seq.actions, commands, distance, origin, origin_name):.2f} m, "
              f"{len(seq.actions)} actions")
        return 0

    results = {}
    for mode in (["merged", "sequential"] if args.mode == "both" else [args.mode]):
        print(f"\n=== {mode} ===")
        s = run_once(sc, mode, os.path.join(args.out, mode), args)
        results[mode] = s
        print(f"  actions {s['actions_succeeded']}/{s['actions_executed']} ok, "
              f"leaf kinds {s['n_leaf_kinds']}/13, tree={s['tree_status']}")
        print(f"  travelled {s['actual_distance_m']:.2f} m in {s['elapsed_s']:.1f} s "
              f"(including return, returned={s['returned']})")
        if s.get("video"):
            print(f"  video: {s['video']}  ({s['frames']} frames)")

    if args.mode == "both" and all(results[m].get("video") for m in results):
        out = compose(args.out, sc["name"], args.fps, args.speed)
        print(f"\nside-by-side: {out}")

    if args.mode == "both":
        md, sq = results["merged"], results["sequential"]
        saved = sq["actual_distance_m"] - md["actual_distance_m"]
        pct = 100.0 * saved / sq["actual_distance_m"] if sq["actual_distance_m"] else 0.0
        t_saved = sq["elapsed_s"] - md["elapsed_s"]
        t_pct = 100.0 * t_saved / sq["elapsed_s"] if sq["elapsed_s"] else 0.0
        print(f"\ntravel: merged {md['actual_distance_m']:.2f} m vs "
              f"sequential {sq['actual_distance_m']:.2f} m  "
              f"-> saved {saved:.2f} m ({pct:.1f}%)")
        print(f"time:   merged {md['elapsed_s']:.1f} s vs "
              f"sequential {sq['elapsed_s']:.1f} s  "
              f"-> saved {t_saved:.1f} s ({t_pct:.1f}%)")
        with open(os.path.join(args.out, "comparison.json"), "w") as f:
            json.dump({"scenario": sc["name"], "merged": md, "sequential": sq,
                       "saved_m": round(saved, 3), "saved_pct": round(pct, 2),
                       "time_saved_s": round(t_saved, 3),
                       "time_saved_pct": round(t_pct, 2)}, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
