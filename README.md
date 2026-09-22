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
  merger.json         26 declarative scenarios (commands + coords + expect block)
sim/                  MuJoCo visual simulation (see below)
reference/
  gpsr_task_manager.py   the real ROS node, UNMODIFIED — on-robot wiring reference
run_scenarios.py      smoke test: runs all 26 scenarios
```

## Running

```bash
python3 run_scenarios.py       # 26/26 expected
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

## Simulation (`sim/`)

A MuJoCo apartment that runs a batch and records it, for the paper's figures.

```bash
python3.14 -m venv .venv
.venv/bin/pip install -r sim/requirements-sim.txt

.venv/bin/python -m sim.run_sim --mode merged --dry-run   # print both orderings
.venv/bin/python -m sim.run_sim --mode both               # -> out/side_by_side.mp4
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
| `scenarios_visual.json` | the 4-command showcase batch (all 13 leaf kinds) |
| `world.py` | loads the apartment; exposes the `locator` the merger needs |
| `mjcf.py` | apartment JSON → MuJoCo MJCF |
| `robot.py` | kinematic base, gripper, pedestrians |
| `handlers.py` | `SimSubtaskHandler` — the 13 leaf behaviours |
| `recorder.py` | frames → ffmpeg, plus the event log written to `run.json` |
| `overlay.py` | PIL HUD; colour-by-source-command |
| `run_sim.py` | CLI: merged / sequential / side-by-side |

Two properties make a run evidence rather than decoration:

- **The gripper is enforced in the executor, not only in the tests.** `pick_object`
  hard-fails when something is already held, so a schedule violating the single-item
  invariant would visibly break the run — an execution-level witness for
  `_merger_helpers.gripper_invariant_holds`.
- **Timeouts are enforced inside the handler in sim-time**, against
  `timeouts.timeout_for`, mirroring the real subtask managers. py_trees' `Timeout`
  decorator only checks between ticks and the leaves block, so faking a wall-clock
  abort would misrepresent where enforcement actually lives.

Because the apartment is open-plan and the robot drives straight lines, distance
driven equals the Euclidean cost `merger._euclidean` optimises — verified: a drive to
`kitchen` covers 5.280152 m against a planned 5.280152 m. Planner and simulator are
measured in the same units, with no metric mismatch to argue about.

### Scope of the simulation

Only `showcase_all_leaves` is fully provisioned. The 26 scenarios in
`scenarios/merger.json` are unit tests *for the scheduler*: they reference locations
like `A`/`loc7` and people like `Jane` that the apartment does not contain, so their
perception leaves correctly report `TARGET_NOT_FOUND` and the tree drops to the
fallback branch. 13 of the 26 have every `go_to` resolvable, but rendering the rest
meaningfully would mean populating the apartment to match each one. `run_scenarios.py`
remains the check that matters for the merger itself, and still passes 26/26.

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
