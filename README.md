# gpsrPaper

Standalone extraction of the GPSR multi-command **merger** and its behaviour-tree
execution layer, pulled out of `RoBorregos/home2` for research work.

## Layout

```
gpsr_merger/          the contribution
  merger.py           interleaving planner (Held-Karp DP over go_to-led segments)
  bt_builder.py       builds the py_trees tree from a merged plan
  leaf_behaviours.py  the leaf behaviours (ActionLeaf, SequentialFallbackLeaf, ...)
  timeouts.py         per-action + global time budgets
  status.py           vendored from task_manager/utils/status.py (11-line enum)
tests/
  _merger_helpers.py  invariant checkers + scenario harness
scenarios/
  merger.json         27 declarative scenarios (commands + coords + expect block)
sim/                  MuJoCo visual simulation (see below)
reference/
  gpsr_task_manager.py   the real ROS node, UNMODIFIED — on-robot wiring reference
run_scenarios.py      smoke test: runs all 27 scenarios
```

## Running

```bash
python3 run_scenarios.py       # 29/29 expected
python3 run_scenarios.py -v    # also print the merged ordering per scenario
```

`merger.py` has **no third-party dependencies at all**. Only `bt_builder.py` and
`leaf_behaviours.py` need py_trees:

```bash
pip install -r requirements.txt   # py_trees==2.2.3
```

## How this was extracted

Copied from `home2` (branch `qwenUpgrade`) and ported with exactly two import
rewrites — there was nothing else to change:

```
from task_manager.gpsr.*          ->  from gpsr_merger.*
from task_manager.utils.status    ->  from gpsr_merger.status
```

The folder had **zero ROS coupling** — no rclpy, no frida_* packages, no message
types. The merger reaches BAML command objects only through `getattr`, and
`leaf_behaviours._resolve_method` resolves subtask methods by name, so both are
duck-typed and run against plain stand-ins.

`reference/gpsr_task_manager.py` is deliberately left with its original
`task_manager.*` imports. It is not meant to run here.

### Deviation from upstream

One bug fix has since been made in `merger.py`, so it no longer matches `home2`
byte-for-byte. A segment's gripper state is now taken from its *last* gripper
action (`_Segment.ends_holding`), instead of assuming pick-before-place. Upstream,
a segment doing `place X, pick Y` with no `go_to` between was treated as ending
empty, letting another command's pick interleave and the robot hold two items.
Pinned by scenario `place_then_pick_in_one_segment_still_holds`. Not yet ported
back to `home2`.

## Simulation (`sim/`)

A MuJoCo apartment that runs a batch and records it, for the paper's figures.

```bash
python3.14 -m venv .venv
.venv/bin/pip install -r sim/requirements-sim.txt

.venv/bin/python -m sim.run_sim --mode merged --dry-run   # print both orderings
.venv/bin/python -m sim.run_sim --mode both               # -> out/side_by_side.{mp4,png}
.venv/bin/python -m sim.run_sim --mode both --no-video    # event log only (~0.1 s)
.venv/bin/pytest tests/test_sim_run.py -q                 # assert on the event log
```

The simulator is only a *consumer* of the contribution. `sim/run_sim.py` calls the
real `merge()` and the real `build_tree()`, exactly as
`reference/gpsr_task_manager.GPSRTM._execute_interleaved_batch` does, and swaps in
`sim.handlers.SimSubtaskHandler` for the robot's subtask managers. That works with no
adapter because `leaf_behaviours._resolve_method` dispatches an action kind to *any*
object exposing a method of that name — so a simulator is one class with 13 methods
returning `(Status, result)`. **Nothing under `gpsr_merger/` is modified or patched.**

The sequential baseline needs no new scheduling code either: `merge()` already
returns `plan.fallback`, so the baseline is the same executor, the same handlers and
the same world, with only the ordering changed.

| file | role |
|---|---|
| `apartment.json` | open-plan floor plan, locations, objects, people — data only |
| `routing/` | obstacle-aware navigation (the sim's stand-in for Nav2): visibility graph around furniture and people, behind a small `Router` protocol |
| `scenarios_visual.json` | `showcase_merge` (the demo, default) and `showcase_all_leaves` (all 13 leaf kinds) |
| `world.py` | loads the apartment; exposes the `locator` the merger needs |
| `mjcf.py` | apartment JSON → MuJoCo MJCF |
| `robot.py` | kinematic base, gripper, pedestrians; drives the waypoints `routing/` plans |
| `handlers.py` | `SimSubtaskHandler` — the 13 leaf behaviours |
| `recorder.py` | frames → ffmpeg, plus the event log written to `run.json` |
| `overlay.py` | PIL HUD; colour-by-source-command |
| `compose.py` | the side-by-side race video: shared clock, Gantt timeline, end card |
| `run_sim.py` | CLI: merged / sequential / side-by-side |

### The demo: `showcase_merge`

Four commands that each zig-zag between the north and south halves of the flat
(kitchen → garage, bedroom → bookshelf, entrance → bathroom, side table → office).
One after another they criss-cross the apartment; merged, the DP sweeps each region
once. Measured in the sim, with all 17 actions succeeding in both runs:

| | time | driven |
|---|---|---|
| merged | 119.1 s | 35.4 m |
| sequential | 161.6 s | 58.4 m |
| saved | **42.4 s (26%)** | **23.0 m (39%)** |

Time saved is smaller in relative terms than distance, because perception and
manipulation take the same time in both orderings. The merger only removes
driving. `showcase_all_leaves` shows why the scenario matters: its commands are
already local, so merging saves only ~4–8%. That run exists for leaf coverage, not
as a demo.

`out/side_by_side.mp4` shows both runs on one clock, with a Gantt timeline
coloured by command. The merged bar interleaves colours; the sequential bar is four
blocks with long driving stretches. `out/side_by_side.png` is the final still.

Two properties make a run evidence rather than decoration:

- **The gripper is enforced in the executor, not only in the tests.** `pick_object`
  hard-fails when something is already held, so a schedule violating the single-item
  invariant would visibly break the run — an execution-level witness for
  `_merger_helpers.gripper_invariant_holds`.
- **Timeouts are enforced inside the handler in sim-time**, against
  `timeouts.timeout_for`, mirroring the real subtask managers. py_trees' `Timeout`
  decorator only checks between ticks and the leaves block, so faking a wall-clock
  abort would misrepresent where enforcement actually lives.

**The robot does not drive through things.** `sim/routing/` plans every movement
on a visibility graph: furniture and standing people are grown by the robot's
footprint radius (0.364 m, plus a 5 cm margin), and Dijkstra over the obstacle
corners gives the exact shortest collision-free path. Follow and guide move robot
and person along one shared route. `tests/test_sim_run.py` checks this from the
event log: no trail sample puts the footprint inside furniture, a wall or a person
(both showcases, both orderings, and a run with injected failures). A straight-line
driver fails those tests.

**Planner and simulator use the same metric.** `merge()` takes an optional
`distance=` travel-cost function (default: Euclidean, which is what the robot uses).
`run_sim` passes the router's path length, so the DP ranks orderings by the distance
the robot will actually drive. In `showcase_merge` the planned cost is within 0.25 m
of the driven distance, and that residue is the pick stand-off. The merger never
sees obstacles, only a cost between two named locations. On the robot the same hook
is fed by Nav2: `GPSRTM._path_cost` answers from a path-length matrix computed in
one `/navigation/query_path_matrix` call.
`scenarios/merger.json` pins the hook: `custom_distance_wall_reorders` puts a wall
between two collinear locations and checks that the order changes.

### Scope of the simulation

Only `showcase_merge` and `showcase_all_leaves` are fully provisioned. The 29 scenarios in
`scenarios/merger.json` are unit tests *for the scheduler*: they reference locations
like `A`/`loc7` and people like `Jane` that the apartment does not contain, so their
perception leaves correctly report `TARGET_NOT_FOUND` and the tree drops to the
fallback branch. 13 of the 27 original ones have every `go_to` resolvable, but rendering
the rest meaningfully would mean populating the apartment to match each one.
`run_scenarios.py` remains the check that matters for the merger itself, and passes 29/29.

## Still to write

- **Port the two pure-Python tests.** In home2, `task_manager/scripts/test/test_hri_manager.py`
  lines 828-889 (`test_merger`) and 890-966 (`test_fallback_resume`) are ROS-free in
  substance — they are just methods on an rclpy Node using `self.get_logger()`.
  Swap the logger for asserts and they become pytest functions.
  `test_merged_plan_speech` (967-1026) does **not** port: it calls
  `GPSRTM._spoken_merged_plan` on the real node.

Note: home2's convention is to add tests to `test_hri_manager.py` rather than
standalone pytest files, but that is tied to its ROS launch harness. With no ROS
here, plain pytest is the better fit.

## Invariants worth citing

`tests/_merger_helpers.py` already states these as machine-checkable properties:

- `non_goto_actions_preserved` — every non-`go_to` action of every command survives
  the merge. This is the formal version of "the merger never drops a command."
- `per_command_order_preserved` — intra-command sequencing is never violated.
- `gripper_invariant_holds` / `gripper_walk_strict` — one item at a time, drop before
  pick, and a pick is released by the same command that acquired it.
- `fallback_preserves_all` — each per-command fallback reproduces its command in full.

Scope claims to *the merger*: the "first part of the first command goes missing"
symptom traced to upstream LLM decomposition, not to this code.

## Provenance / licensing — UNRESOLVED

- Source: `github.com/RoBorregos/home2`, `task_manager/task_manager/gpsr/`.
- **home2 has no LICENSE file.** Settle licensing with RoBorregos before this repo
  goes public.
- Two authors touched the folder:
  - `2965c2043` Added gpsr command merging (#937) — Luis Alvaro Rosales Salazar
  - `75c28b966` Gpsr/robocup (#1119) — Gilberto Malagamba Montejo
