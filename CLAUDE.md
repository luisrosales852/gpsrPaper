# CLAUDE.md

Research repo for a paper on GPSR multi-command **merging**. It is a standalone
extraction of `task_manager/task_manager/gpsr/` from `RoBorregos/home2` (branch
`qwenUpgrade`), plus a MuJoCo simulation that produces the paper's figures.

Read `README.md` too — it carries the narrative, the provenance/licensing status
and the "still to write" list. This file is the operational map.

## The one rule that shapes everything

`gpsr_merger/` is **the contribution**. Nothing outside it may modify, patch,
subclass or monkeypatch it. `sim/` and `tests/` are *consumers* — that is the
paper's claim, and breaking it invalidates the evidence. If a sim feature seems
to need a change in `gpsr_merger/`, that is a finding to report, not a patch to
apply.

Corollaries:

- `gpsr_merger/merger.py` has **zero third-party dependencies**. Keep it that way.
- `bt_builder.py` / `leaf_behaviours.py` may use **py_trees only**.
- Sim-only deps live in `sim/requirements-sim.txt`, deliberately *not* merged
  into the root `requirements.txt`.
- `reference/gpsr_task_manager.py` is the real ROS node, **unmodified**, with its
  original `task_manager.*` imports. It does not run here and must not be
  "fixed". It exists as the on-robot wiring reference — when you need to know how
  something is called for real, read `GPSRTM._execute_interleaved_batch` (line
  ~295).

## Commands

```bash
python3 run_scenarios.py          # merger smoke test — expect 29/29
python3 run_scenarios.py -v       # also print the merged ordering per scenario

# Simulation (needs the venv; Python 3.14)
python3.14 -m venv .venv
.venv/bin/pip install -r sim/requirements-sim.txt

.venv/bin/python -m sim.run_sim --mode merged --dry-run   # print both orderings, no render
.venv/bin/python -m sim.run_sim --mode both --no-video    # event log only, ~0.1 s
.venv/bin/python -m sim.run_sim --mode both               # -> out/side_by_side.{mp4,png}, ~1 min
.venv/bin/python -m sim.run_sim --scenario showcase_all_leaves   # 13-leaf coverage run
.venv/bin/pytest tests/test_sim_run.py -q                 # 12 tests, headless, <1 s
.venv/bin/python -m tests.test_sim_run                    # same checks without pytest
```

- `run_scenarios.py` needs no venv and no py_trees — that *is* the point of it.
- Video encoding shells out to the system `ffmpeg` (`brew install ffmpeg`); there
  is no Python video dep. `--no-video` skips it entirely, which is what CI and
  every iteration loop should use.
- Always run from the repo root (`-m sim.run_sim`, not `python sim/run_sim.py`);
  the entry scripts do `sys.path.insert` of the repo root themselves.
- Useful flags: `--fail pick_object:1` or `--fail 0.1:2` (`cmd.idx:count`) to
  inject failures and exercise Retry → Selector → `sequential_fallback`;
  `--print-tree`; `--scenario <name>`; `--camera iso`.
- Defaults are tuned for the 1080p race video: `--width 832 --height 720` matches
  the apartment's aspect (no grey side bars), `--speed 3.0`. Both modes must share
  one `--speed`/`--fps`, since `compose.py` lines them up on sim time.

## Architecture

```
gpsr_merger/           THE CONTRIBUTION — no third-party deps except py_trees in 2 files
  merger.py            interleaving planner; merge() -> InterleavedPlan
  bt_builder.py        build_tree() -> py_trees root Selector
  leaf_behaviours.py   ActionLeaf, SequentialFallbackLeaf, OneShotCallbackLeaf
  timeouts.py          per-action budgets + GLOBAL_BUDGET_S (300 s)
  status.py            vendored 11-line Status enum
scenarios/merger.json  29 declarative scheduler scenarios (commands + coords + expect)
tests/_merger_helpers.py  invariant checkers, BAML stand-in builder, expect-evaluator
tests/test_sim_run.py     12 tests on real headless sim runs (incl. collision checks)
sim/                   MuJoCo apartment; consumer of the contribution
reference/             the real ROS node, untouched
out/                   generated videos + run.json + comparison.json (gitignored)
```

### merger.py

`merge(commands, locator=None, origin=None, distance=None) -> InterleavedPlan`

1. `_decompose` each command into **go_to-led segments**.
2. Held-Karp DP (`_optimal_schedule`) over segments, minimising travel (2-D
   Euclidean by default, or the caller's `distance(from_name, to_name)`), with two-part
   eligibility: per-command prefix order, and a gripper-state check (only `neutral` segments may interleave while another
   command holds the gripper).
3. Flatten back to `PlanAction`s, then `_collapse_redundant_go_tos`.

Things that will bite you:

- **`_M_CAP = 16`.** Above 16 segments it silently falls back to
  `_sequential_schedule` (Held-Karp is O(2^M · M²)). Scenario
  `m_cap_fallback_to_sequential` pins this.
- **`LARGE_M = 1000.0`** is the sentinel cost for any edge touching an
  unresolved (`None`) coordinate. Unknown locations are avoided unless prefix
  order forces them.
- **`_TIEBREAK_EPSILON = 1e-6`** breaks ties toward per-command sequential order.
  Must stay far below any real distance difference.
- **`origin=None` is meaningful**, not a lazy default: it seeds the DP with no
  origin edge, avoiding a bias toward the SLAM map origin when the real pose is
  unknown. Scenarios cover both.
- Everything reaches command objects through `getattr`, so plain
  `SimpleNamespace` stand-ins work. That duck-typing is load-bearing.
- **`distance=` is the only metric hook.** `merge(..., distance=fn)` replaces
  `_euclidean`. It takes location **names**, `fn(from_name | None, to_name)`, where
  `None` is the robot origin (only asked when `origin` is not None) — the same
  signature as home2's `GPSRTM._path_cost`, which answers from a Nav2 path-length
  matrix. `_cost_matrix` calls it once per distinct location pair *before* the DP
  (never inside the O(2^M · M²) loop) and maps `None` or a non-finite
  (unreachable) cost to `LARGE_M`. The merger never learns about obstacles.
  Scenarios express it with `distance_overrides: [[name_a, name_b, cost|null]]`
  (see `tests/_merger_helpers.make_distance`).
- `plan.fallback` is the per-command original sequence — the sim reuses it
  verbatim as the sequential baseline, so no separate baseline scheduler exists.

### bt_builder.py / leaf_behaviours.py

```
Selector "gpsr_root" (memory)
  ├── Timeout(GLOBAL_BUDGET_S) ▸ Sequence "interleaved"
  │     └── Retry(2) ▸ Timeout(timeout_for(kind)) ▸ ActionLeaf   (per action)
  └── Sequence "sequential_fallback"
        └── SequentialFallbackLeaf  (per command; always returns SUCCESS)
```

- **Handler dispatch is by method name.** `_resolve_method(kind, handlers)` picks
  the first object in `subtask_handlers` exposing a callable named after the
  action kind, calls it with the bare action object, and expects
  `(Status, result)` (a bare `Status` is tolerated). This is the entire
  integration surface — no registration, no base class.
- Leaves are **synchronous and blocking**. py_trees `Timeout` decorators only
  check between ticks, so real timeout enforcement must live *inside* the
  handler. `sim/handlers.py` mirrors that with `robot.deadline` in sim-time —
  do not "fix" it with a wall-clock abort, it would misrepresent where
  enforcement lives on the robot.
- The fallback branch always returns SUCCESS: its job is to give every command a
  chance, not to gate on outcomes.
- `is_completed` enables *resume* rather than restart in the fallback: a
  completed non-`go_to` is skipped, and a `go_to` is skipped only when its whole
  following segment is already done.

### sim/

| file | role |
|---|---|
| `apartment.json` | open-plan floor plan: bounds, zones, furniture, 19 locations, 8 objects, 3 people — data only. Every location must be in free space (tested) |
| `routing/` | **all navigation lives here, never in `gpsr_merger/`** (Nav2 owns it on the robot). `base.Router` protocol (`set_people`, `free_point`, `plan_path`, `distance`); `obstacles.py` inflates furniture and people rects by `ROBOT_RADIUS` (0.364 m, from the mjcf footprint) + margin; `visgraph.VisGraphRouter` = visibility graph + Dijkstra, exact and deterministic. A grid/A* router can be added later behind the same protocol |
| `scenarios_visual.json` | `showcase_merge` (**default, the demo**): 4 commands that each zig-zag north↔south, so ordering matters — merged is ~26% faster / ~39% less driving. `showcase_all_leaves`: 4 commands whose union hits all 13 leaf kinds (coverage; the 7 original tests use it, and it saves only ~4–8% so it is a poor demo) |
| `world.py` | loads the apartment; `World.locator` is the merger-facing contract (mirrors `GPSRTM._resolve_xy`, returns `None` for unknown) |
| `mjcf.py` | apartment JSON → MuJoCo MJCF string |
| `robot.py` | kinematic base, gripper, pedestrians; raises `SimTimeout` past `deadline`. `drive_path` drives router waypoints; `follow_person`/`lead_person` move robot and person along one shared track |
| `handlers.py` | `SimSubtaskHandler` — the 13 leaf implementations; every movement goes through `self._path` (router, with the escorted person excluded) |
| `recorder.py` | frames → ffmpeg; writes the event log to `run.json` |
| `overlay.py` | PIL HUD, colour-by-source-command |
| `compose.py` | `--mode both` only: decodes both `run.mp4`s, crops their HUD panels and composes the 1920×1080 race video — per-pane clock/distance/current action, a Gantt timeline on a shared time axis (command colour; half shade = `go_to`), finish stamps, end card. Also writes `side_by_side.png` (clean final still) and `side_by_side_endcard.png` |
| `run_sim.py` | CLI: merged / sequential / both; writes `comparison.json` |

The robot never drives straight through things: handlers get every path from
`sim.routing`, and `run_sim` builds a *planning* router (people at home) whose
`distance` goes to `merge(distance=...)` (wrapped by name in `merger_distance`)
and to `planned_cost`, plus a separate
*execution* router on the handler (people where they currently stand). Planned and
driven distance are therefore the same metric. Adding interior walls is fine now
(add them as rects to the router's obstacles). Swapping the router for a
non-shortest-path planner would break the metric equivalence the paper leans on.

`out/{merged,sequential}/run.json` holds `summary`, `events`, `waypoints`,
`trail`, `people_trail` (every person's `[x, y]` at each trail sample, same
index), `labels` and `frames` (`[t, distance_m]` per captured video frame — how
`compose.py` syncs the panes). `summary.task_time_s` is the last action's end;
`sim_time_s` also includes the 1.2 s idle hold, so compare `task_time_s`.
`comparison.json` carries `saved_m`/`saved_pct` and `time_saved_s`/`time_saved_pct`.
Events come in `start`/`end` pairs with
`t, phase, kind, cmd, idx, status, result, x, y, held, distance_m, branch`.
Tests assert on this log, never on pixels.

## Tests and invariants

`tests/_merger_helpers.py` states the machine-checkable properties — cite these
by name in the paper:

- `non_goto_actions_preserved` — no command (or part of one) is dropped.
  `go_to` is exempt because collapse legitimately dedups navigation.
- `per_command_order_preserved` — intra-command sequencing never violated.
- `gripper_invariant_holds` / `gripper_walk_strict` — one item at a time, drop
  before pick; strict also requires the *same* command to release its own pick.
- `fallback_preserves_all` — each per-command fallback reproduces its command.

`tests/test_sim_run.py` re-witnesses these at execution time, plus
`test_merged_travels_less_than_sequential` and
`test_cost_model_gap_is_attributable_to_non_goto_displacement` (the planner only
models `go_to` hops, so the gap must come from follow/guide/pick stand-off).
Navigation evidence, all read from the event log:
`test_every_location_is_in_free_space`, `test_robot_never_overlaps_furniture_or_walls`,
`test_robot_keeps_clear_of_people` (these sweep `COLLISION_RUNS`: both showcases,
both orderings, and one run with injected failures) and
`test_planned_cost_is_the_driven_metric`.

**Adding a merger scenario:** append an object to `scenarios/merger.json` with
`commands`, optional `coords` / `origin`, and an `expect` block. Every key in
`expect` is optional; the supported checks are exactly the keys handled in
`tests/_merger_helpers.evaluate_expectations` (`plan_kinds`, `go_to_order`,
`location_visit_counts`, `precedence_per_cmd`, `gripper_walk_strict`, …). Add a
new check there rather than writing a bespoke test.

**Adding a leaf kind** means touching three places: `timeouts.ACTION_TIMEOUTS`,
a same-named method on `sim/handlers.SimSubtaskHandler`, and `ALL_KINDS` in
`tests/test_sim_run.py`.

## Scope limits worth remembering

- Only `showcase_merge` and `showcase_all_leaves` are fully provisioned in the apartment. The 27
  scenarios in `scenarios/merger.json` are unit tests *for the scheduler*; they
  reference locations like `A`/`loc7` and people like `Jane` that do not exist in
  the apartment, so their perception leaves correctly report `TARGET_NOT_FOUND`.
  That is expected, not a bug. `run_scenarios.py` is the check that matters for
  the merger.
- Scope claims to **the merger**. The "first part of the first command goes
  missing" symptom traced to upstream LLM decomposition, not to this code.
- **Licensing is unresolved** — home2 has no LICENSE file. Settle with RoBorregos
  before this repo goes public. Two authors touched the source folder (see
  README).

## Repo state

The simulation is committed (`935fdc9 doing simulation`). `out/`, `.venv/` and
`__pycache__/` are gitignored — regenerate videos with `--mode both`.
