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
python3 run_scenarios.py          # merger smoke test — expect 62/62
python3 run_scenarios.py -v       # also print the merged ordering per scenario

# Simulation (needs the venv; Python 3.14)
python3.14 -m venv .venv
.venv/bin/pip install -r sim/requirements-sim.txt

.venv/bin/python -m sim.run_sim --mode merged --dry-run   # print both orderings, no render
.venv/bin/python -m sim.run_sim --mode both --no-video    # event log only, ~0.1 s
.venv/bin/python -m sim.run_sim --mode both               # -> out/side_by_side.{mp4,png}, ~1 min
.venv/bin/python -m sim.run_sim --scenario showcase_all_leaves   # 13-leaf coverage run
.venv/bin/pytest tests/test_sim_run.py -q                 # 17 tests, headless, <1 s
python3 -m tests.test_progress_dp                         # DP vs Held-Karp + named-start brute force, no venv
python3 -m tests.test_dependencies                        # conflict order + oracle under deps, no venv
python3 bench_runtime.py [--latex]                         # paper's planner-runtime table, no venv
python3 bench_holding.py [--latex]                         # travel vs distance carried per lambda, no venv
.venv/bin/python -m tests.test_sim_run                    # same checks without pytest
```

- `run_scenarios.py` needs no venv and no py_trees — that *is* the point of it.
- Video encoding shells out to the system `ffmpeg` (`brew install ffmpeg`); there
  is no Python video dep. `--no-video` skips it entirely, which is what CI and
  every iteration loop should use.
- Always run from the repo root (`-m sim.run_sim`, not `python sim/run_sim.py`);
  the entry scripts do `sys.path.insert` of the repo root themselves.
- Useful flags: `--holding-penalty 1` (λ, default 0); `--fail pick_object:1` or `--fail 0.1:2` (`cmd.idx:count`) to
  inject failures and exercise one of the three recovery policies;
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
  timeouts.py          per-action budgets + GLOBAL_BUDGET_S (420 s)
  status.py            vendored 11-line Status enum
scenarios/merger.json  62 declarative scheduler scenarios (commands + coords + expect)
tests/_merger_helpers.py  invariant checkers, BAML stand-in builder, expect-evaluator
tests/test_sim_run.py     17 tests on real headless sim runs (incl. collision checks)
tests/test_progress_dp.py differential test: progress-vector DP vs tests/_heldkarp_oracle.py
tests/test_dependencies.py cross-command dependencies on random GPSR-shaped batches
sim/                   MuJoCo apartment; consumer of the contribution
reference/             the real ROS node, untouched
out/                   generated videos + run.json + comparison.json (gitignored)
```

### merger.py

`merge(commands, locator=None, origin=None, distance=None) -> InterleavedPlan`

1. `_decompose` each command into **go_to-led segments**.
2. Progress-vector DP (`_optimal_schedule`) over segments — state `(k, last_cmd)`,
   `k[c]` = segments of command c done — minimising travel (2-D
   Euclidean by default, or the caller's `distance(from_name, to_name)`), with two-part
   eligibility: per-command prefix order, and a gripper-state check (only `neutral` segments may interleave while another
   command holds the gripper).
3. Flatten back to `PlanAction`s, then `_collapse_redundant_go_tos`.

Things that will bite you:

- **`_STATE_CAP = 200_000`** on `n · ∏(L_c+1)` (`_state_count`). Above it the
  merger falls back to `_sequential_schedule`. Time is O(n² · ∏(L_c+1)), never
  worse than Held-Karp's O(2^M · M²). One-segment commands are the worst case
  (∏ = 2^M): `m_cap_fallback_to_sequential` (17 of them) pins the fallback,
  `beyond_old_m_cap_still_interleaves` (18 segments) pins that the old 16-segment
  cliff is gone. The old solver lives on as `tests/_heldkarp_oracle.py`.
  `improvements.md` tracks what still has to be ported to home2.
- **`LARGE_M = 1000.0`** is the sentinel cost for any edge touching an
  unresolved (`None`) coordinate. Unknown locations are avoided unless prefix
  order forces them.
- **`_TIEBREAK_EPSILON = 1e-6`** breaks ties toward per-command sequential order.
  Must stay far below any real distance difference.
- **`origin=None` is meaningful**, not a lazy default: it seeds the DP with no
  origin edge, avoiding a bias toward the SLAM map origin when the real pose is
  unknown. Scenarios cover both.
- **Entry vs exit.** A segment enters at its `go_to` target and exits there too,
  unless it contains a `guide_person_to` / `follow_person_until` with a
  destination: then `_Segment.exit` is that destination and the next hop is
  priced from it (`exit_*` scenarios). An escort also stops
  `_collapse_redundant_go_tos` from dropping the next `go_to`.
- **`at_origin` segments.** Actions before a command's first `go_to` run at the
  robot's start: free when first, priced with `distance(None, place)` otherwise
  (`LARGE_M` to return when `origin=None`). Without `origin_name` that is
  pricing only. With `merge(origin_name=...)` a `go_to(origin_name)` is inserted
  before such a segment (`PlanAction.source_idx == -1`, plain `SimpleNamespace`
  action) by `_flatten`, dropped by the collapse when the robot is already
  there; the DP prices it as driven (return `distance(place, origin_name)`,
  then from `origin_name`; from `None`, i.e. `seed`, while the robot has not
  moved), and it makes the origin known when `origin=None`.
  A name `distance` cannot reach is ignored (`_reachable`), so a typo plans
  like no name. `planned_travel(..., origin_name)` re-derives that drive;
  `test_named_start_is_priced_as_driven` checks it against brute force. Each headless command's fallback starts with it too, and
  those actions' dependency effects are keyed at `origin_name` instead of `None`
  (`_annotate_effects(start=)`; the checker takes the same `start`). Sim passes
  `run_sim.start_name` (`start_location`). `here_*` scenarios,
  `test_origin_name_only_adds_the_drive_back`, `improvements.md` §4–5.
- Everything reaches command objects through `getattr`, so plain
  `SimpleNamespace` stand-ins work. That duck-typing is load-bearing.
- **Cross-command dependencies (on by default).** Each action has read/write
  sets over `(objects|people, place)` (`_EFFECTS`, `_action_effects`); segments of
  different commands that conflict keep issue order (`_build_needs` → `needs[c][j]
  = ((d, m), ...)`, one `continue` in the DP). Keyed by **place, not name**, on
  purpose (parser naming is inconsistent). `room_of(name) -> room` (optional hook,
  like `distance=`) makes a room overlap the spots inside it; without it only equal
  names conflict. `dependencies=False` = travel-only, for ablations. Sim passes
  `World.room_of` (zones). `scenarios/merger.json` `dep_*` pin the cases.
- **`holding_penalty` (λ, default 0).** Every metre driven while some command
  holds an object costs λ extra: hops (`holding_penalty * row[u]` when the DP
  state has a holder) and escort legs inside a segment (the paper's ρ, distance
  part: `_carried_inside`, from `_Segment.escorts`, split by whether the
  segment starts holding; legs of a headless escort start at the pose while the
  robot has not moved). Still exact: both depend only on the state
  (`test_holding_penalty_is_exact`, brute force). `planned_travel(...,
  holding_penalty=)` re-derives it. `holding_penalty_*` scenarios pin the flips
  (`..._counts_escorts_while_holding` fails if only hops count);
  `bench_holding.py` gives the trade-off (λ=1: carried 6.49 m vs 8.04 at λ=0 and
  6.22 sequential, travel +2.9%). κ and τ are not modelled (`futureWork.md`).
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

### bt_builder.py / recovery.py

The memory sequence ticks the merged order, optional sequential fallback,
deferral, return, and failure reporting. `build_tree()` defaults to
`sequential-fallback`, while merged simulator runs default to `skip-defer`.
The paired trial runner chooses each recovery policy explicitly.

- The skill dispatch resolves methods by action name and accepts a bare
  `Status` or `(Status, result)`. Status callbacks and reports carry source
  command and action indices.
- `ExecutionHooks` supplies the clock, the absolute deadline, gripper
  verification, revisit estimates, and the return and reporting operations.
  The clock is simulated in `run_sim.py`. Skills enforce it inside their
  synchronous calls.
- Fallback gives the holder's command priority over competing picks. Both
  local policies keep independent work and shared navigation after a failure.
  Only independently recoverable information work can be deferred.
- `.venv/bin/python -m sim.recovery_trials` reproduces the paired paper
  table. `tests/test_recovery.py` checks the results and failure-specific
  execution traces. See `improvements.md` for the home2 port requirements.

### sim/

| file | role |
|---|---|
| `apartment.json` | open-plan floor plan: bounds, zones, furniture, 19 locations, 8 objects, 3 people — data only. Every location must be in free space (tested) |
| `routing/` | **all navigation lives here, never in `gpsr_merger/`** (Nav2 owns it on the robot). `base.Router` protocol (`set_people`, `free_point`, `plan_path`, `distance`); `obstacles.py` inflates furniture and people rects by `ROBOT_RADIUS` (0.364 m, from the mjcf footprint) + margin; `visgraph.VisGraphRouter` = visibility graph + Dijkstra, exact and deterministic. A grid/A* router can be added later behind the same protocol |
| `scenarios_visual.json` | `showcase_exits`: a guide and a follow that end far from where they start, plus a command with no `go_to` (pins exit pricing; start the guide from `lobby`, not `entrance` — a guide whose route passes the person's standing spot clips them). `showcase_merge` (**default, the demo**): 4 commands that each zig-zag north↔south, so ordering matters — merged is ~26% faster / ~39% less driving. `showcase_all_leaves`: 4 commands whose union hits all 13 leaf kinds (coverage; the 7 original tests use it, and it saves only ~7–14% so it is a poor demo) |
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
- `conflict_order_preserved` — every conflicting pair of actions of different
  commands runs in issue order, i.e. the plan is conflict-equivalent to sequential
  execution. Re-derived at action level, independent of the segment table.

`tests/test_sim_run.py` re-witnesses these at execution time, plus
`test_merged_travels_less_than_sequential` and
`test_cost_model_gap_is_attributable_to_non_goto_displacement` (the planner
models hops between segments, so the gap must come from follow/guide/pick
stand-off), `test_exit_locations_are_priced` (gap = in-segment displacement to
within `FOLLOW_GAP`) and `test_command_without_go_to_runs_at_the_origin`.
Navigation evidence, all read from the event log:
`test_every_location_is_in_free_space`, `test_robot_never_overlaps_furniture_or_walls`,
`test_robot_keeps_clear_of_people` (these sweep `COLLISION_RUNS`: the three showcases,
both orderings, and one run with injected failures) and
`test_planned_cost_is_the_driven_metric`.

**Adding a merger scenario:** append an object to `scenarios/merger.json` with
`commands`, optional `coords` / `origin` / `distance_overrides` / `rooms` (`{place: room}`
→ `room_of`) / `dependencies` (false = travel-only), and an `expect` block.
`tests/_merger_helpers.merge_kwargs` turns those into `merge()` arguments. Every key in
`expect` is optional; the supported checks are exactly the keys handled in
`tests/_merger_helpers.evaluate_expectations` (`plan_kinds`, `go_to_order`,
`location_visit_counts`, `precedence_per_cmd`, `gripper_walk_strict`,
`action_before`, `conflict_order_preserved`, …). Add a
new check there rather than writing a bespoke test.

**Adding a leaf kind** means touching four places: `timeouts.ACTION_TIMEOUTS`,
a same-named method on `sim/handlers.SimSubtaskHandler`, `ALL_KINDS` in
`tests/test_sim_run.py`, and `merger._EFFECTS` if it reads or changes objects or
people (a kind missing there is assumed to touch no shared state).

## Scope limits worth remembering

- Only `showcase_merge`, `showcase_all_leaves` and `showcase_exits` are fully provisioned in the apartment. The 62
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
