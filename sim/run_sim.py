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

from gpsr_merger.bt_builder import build_tree, render_tree_ascii  # noqa: E402
from gpsr_merger.merger import InterleavedPlan, merge  # noqa: E402
from gpsr_merger.status import Status  # noqa: E402
from gpsr_merger.timeouts import GLOBAL_BUDGET_S  # noqa: E402
from sim.compose import compose  # noqa: E402
from sim.handlers import SimSubtaskHandler  # noqa: E402
from sim.mjcf import build_xml  # noqa: E402
from sim.recorder import Recorder  # noqa: E402
from sim.robot import SimRobot  # noqa: E402
from sim.world import World  # noqa: E402
from tests._merger_helpers import build_commands, describe_plan  # noqa: E402

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


def planned_cost(actions: Sequence[Any], world: World,
                 origin: Optional[Tuple[float, float]]) -> float:
    """The merger's own objective for this ordering.

    Sums Euclidean hops between consecutive resolvable ``go_to`` targets, which
    is precisely what ``merger._optimal_schedule`` minimises. Comparing it with
    the distance the robot actually drives is how the sim exposes displacement
    that the cost model does not account for.
    """
    last = origin
    total = 0.0
    for pa in actions:
        if getattr(pa.action, "action", "") != "go_to":
            continue
        xy = world.locator(getattr(pa.action, "location_to_go", "") or "")
        if xy is None:
            continue
        if last is not None:
            total += math.hypot(xy[0] - last[0], xy[1] - last[1])
        last = xy
    return total


def sequential_plan(plan: InterleavedPlan) -> InterleavedPlan:
    """The per-command baseline, straight out of ``merge()``.

    ``plan.fallback`` is already the original sequence of each command, so the
    baseline needs no new scheduling code: same executor, same handlers, same
    world — only the ordering differs, which is exactly the claim under test.
    """
    return InterleavedPlan(actions=[pa for per_cmd in plan.fallback for pa in per_cmd],
                           fallback=[])


def run_once(sc: dict, mode: str, out_dir: str, args) -> dict:
    world = World(args.apartment)
    origin = tuple(sc["origin"]) if sc.get("origin") is not None else None
    commands = build_commands(sc["commands"])

    plan = merge(commands, locator=world.locator, origin=origin)
    if mode == "sequential":
        exec_plan = sequential_plan(plan)
    else:
        exec_plan = InterleavedPlan(actions=plan.actions, fallback=plan.fallback)

    model = mujoco.MjModel.from_xml_string(build_xml(world, args.width, args.height))
    data = mujoco.MjData(model)
    robot = SimRobot(model, data, world)
    if origin is not None:
        robot.set_pose(origin[0], origin[1], math.pi / 2)
    robot.sync()

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

    handler = SimSubtaskHandler(world, robot, recorder=rec, failures=failures)
    completed = set()

    def on_complete(pa, status, result):
        if status == Status.EXECUTION_SUCCESS:
            completed.add((pa.source_cmd, pa.source_idx))
        handler.on_action_complete(pa, status, result)

    def on_fallback_entry():
        rec.branch = "sequential_fallback"
        rec.leaf_line = "SequentialFallbackLeaf"

    root = build_tree(
        exec_plan,
        subtask_handlers=[handler],
        on_action_complete=on_complete,
        retry_count=args.retries,
        global_budget_s=args.budget,
        on_fallback_entry=on_fallback_entry,
        is_completed=lambda pa: (pa.source_cmd, pa.source_idx) in completed,
        on_action_start=handler.on_action_start,
    )
    if args.print_tree:
        print(render_tree_ascii(root))

    tree = py_trees.trees.BehaviourTree(root)
    try:
        tree.setup(timeout=15.0)
    except Exception as e:  # noqa: BLE001 — mirrors GPSRTM, setup is best-effort
        print(f"  tree setup raised: {e}")

    terminal = (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE,
                py_trees.common.Status.INVALID)
    for _ in range(2000):
        tree.tick()
        if root.status in terminal:
            break

    robot.clear_annotations()
    robot.idle(1.2)  # hold the final frame so the video does not cut dead

    extra = {
        "tree_status": str(root.status),
        "planned_cost_m": round(planned_cost(exec_plan.actions, world, origin), 3),
        "n_plan_actions": len(exec_plan.actions),
        "command_switches": sum(
            1 for a, b in zip(exec_plan.actions, exec_plan.actions[1:])
            if a.source_cmd != b.source_cmd),
    }
    ends = [e for e in rec.events if e["phase"] == "end"]
    # When the last action finished — sim_time_s also includes the idle hold.
    extra["task_time_s"] = ends[-1]["t"] if ends else 0.0
    extra["cost_model_gap_m"] = round(
        rec.summary()["actual_distance_m"] - extra["planned_cost_m"], 3)
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
    args = ap.parse_args()

    sc = load_scenario(args.scenario)
    os.makedirs(args.out, exist_ok=True)

    if args.dry_run:
        world = World(args.apartment)
        commands = build_commands(sc["commands"])
        origin = tuple(sc["origin"]) if sc.get("origin") is not None else None
        plan = merge(commands, locator=world.locator, origin=origin)
        seq = sequential_plan(plan)
        print(f"scenario: {sc['name']}  ({len(commands)} commands, origin={origin})\n")
        print("MERGED     :", describe_plan(plan))
        print(f"  planned cost {planned_cost(plan.actions, world, origin):.2f} m, "
              f"{len(plan.actions)} actions")
        print("\nSEQUENTIAL :", describe_plan(seq))
        print(f"  planned cost {planned_cost(seq.actions, world, origin):.2f} m, "
              f"{len(seq.actions)} actions")
        return 0

    results = {}
    for mode in (["merged", "sequential"] if args.mode == "both" else [args.mode]):
        print(f"\n=== {mode} ===")
        s = run_once(sc, mode, os.path.join(args.out, mode), args)
        results[mode] = s
        print(f"  actions {s['actions_succeeded']}/{s['actions_executed']} ok, "
              f"leaf kinds {s['n_leaf_kinds']}/13, tree={s['tree_status']}")
        print(f"  travelled {s['actual_distance_m']:.2f} m in {s['sim_time_s']:.1f} s "
              f"(planned {s['planned_cost_m']:.2f} m, gap {s['cost_model_gap_m']:+.2f} m)")
        if s.get("video"):
            print(f"  video: {s['video']}  ({s['frames']} frames)")

    if args.mode == "both" and all(results[m].get("video") for m in results):
        out = compose(args.out, sc["name"], args.fps, args.speed)
        print(f"\nside-by-side: {out}")

    if args.mode == "both":
        md, sq = results["merged"], results["sequential"]
        saved = sq["actual_distance_m"] - md["actual_distance_m"]
        pct = 100.0 * saved / sq["actual_distance_m"] if sq["actual_distance_m"] else 0.0
        t_saved = sq["task_time_s"] - md["task_time_s"]
        t_pct = 100.0 * t_saved / sq["task_time_s"] if sq["task_time_s"] else 0.0
        print(f"\ntravel: merged {md['actual_distance_m']:.2f} m vs "
              f"sequential {sq['actual_distance_m']:.2f} m  "
              f"-> saved {saved:.2f} m ({pct:.1f}%)")
        print(f"time:   merged {md['task_time_s']:.1f} s vs "
              f"sequential {sq['task_time_s']:.1f} s  "
              f"-> saved {t_saved:.1f} s ({t_pct:.1f}%)")
        with open(os.path.join(args.out, "comparison.json"), "w") as f:
            json.dump({"scenario": sc["name"], "merged": md, "sequential": sq,
                       "saved_m": round(saved, 3), "saved_pct": round(pct, 2),
                       "time_saved_s": round(t_saved, 3),
                       "time_saved_pct": round(t_pct, 2)}, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
