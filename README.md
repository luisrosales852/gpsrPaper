# gpsrPaper

Standalone extraction of the GPSR multi-command **merger** and its behaviour-tree
execution layer, pulled out of `RoBorregos/home2` for research work.

## Layout

```
gpsr_merger/          the contribution
  merger.py           interleaving planner (progress-vector DP over go_to-led segments)
  bt_builder.py       builds the py_trees tree from a merged plan
  recovery.py        execution phases, recovery policies, and robot adapter hooks
  leaf_behaviours.py  handler dispatch and gripper-aware fallback ordering
  timeouts.py         per-action + global time budgets
  status.py           vendored from task_manager/utils/status.py (11-line enum)
tests/
  _merger_helpers.py  invariant checkers + scenario harness
scenarios/
  merger.json         64 declarative scenarios (commands + coords + expect block)
sim/                  MuJoCo visual simulation (see below)
reference/
  gpsr_task_manager.py   the real ROS node, UNMODIFIED — on-robot wiring reference
run_scenarios.py      smoke test: runs all 64 scenarios
```

## Running

```bash
python3 run_scenarios.py       # 64/64 expected
python3 run_scenarios.py -v    # also print the merged ordering per scenario
```

`merger.py` has no third-party dependencies. The executor uses py_trees.

```bash
pip install -r requirements.txt   # py_trees==2.2.3
```

### Testing against the original paper

Use `git show 8e77cd4:main.tex` to read the earliest paper in this repository.
The copy on `origin/main` at the time of the comparison has the same contents.
The Objective subsection and Proposition 1 give a testable claim for a
failure-free batch. The merger should return a feasible order with the lowest
modeled cost among all orders of its atomic segments.

```sh
.venv/bin/python -m pytest -q tests/test_paper_objective.py
.venv/bin/python run_scenarios.py
.venv/bin/python -m pytest -q
```

The first check enumerates all 720 orders of six segments from three commands.
It encodes the two issue-order conflicts explicitly, replays gripper ownership
and scores each feasible order in seconds. The test calls the public `merge()`
and checks its emitted drives and skills as well as its cost. Two layouts and
four settings exercise navigation overhead, carrying while driving or escorting,
and carrying during stationary skills. Any minimum-cost feasible order passes.
The API takes navigation overhead in metres, so the test passes `speed_m_s * κ`
for an overhead of κ seconds.

For an execution check with the simulation dependencies installed, run
`python -m sim.benchmark run scenarios/benchmark_2027.json --rates 0 --repeats 1 --out out/paper-nominal`.
Compare completed goals, elapsed time and distance for the same batch and layout
in `results.json`. This is a pilot with eight batches, two simulated layouts
and zero injected failures.

Person attention is shared across commands and locations. Either find skill
selects the person. Person-info, guide and follow consume that selection.
The merger keeps conflicting segments in issue order. In the two-person
simulation, it reads Angel's pose before searching for Tom and returns
“sitting”. With dependencies disabled, it finds Tom before reading the pose
and returns “standing”. Reports still draw on the issuing command's results.

```sh
.venv/bin/python -m pytest -q tests/test_paper_objective.py tests/test_paper_extensions.py -k person_attention
```

### Interleaving bonus and the 25-second endgame

Run both experiments from the repository root with the simulation dependencies installed.

```sh
.venv/bin/python -m sim.benchmark run scenarios/benchmark_2027.json --bonus-comparison --rates 0 0.2 --repeats 3 --out out/bonus
.venv/bin/python -m sim.endgame scenarios/endgame_25s.json --remaining 13 14 25 35 36 90 --bins 1 5 --out out/endgame.json
.venv/bin/python -m pytest -q tests/test_paper_objective.py tests/test_endgame.py
```

The primary method in the bonus comparison is `merger`, the ordinary
`merge()` call. It may interleave when that reduces the objective and can
choose a whole-command order, including the issued order. Back-to-back
execution provides the sequential reference. The two fixed-bit runs measure
the effect of restricting the available schedules.

`merge(interleaved=False)` finds the lowest-cost schedule of whole commands
in any legal order. `merge(interleaved=True)` finds the lowest-cost schedule
that starts work on another command while one is unfinished. Both use the
same gripper, attention and dependency constraints. An impossible class raises
`NoInterleavingSchedule`. The default call optimizes over both classes.

In the bonus benchmark, all three commands arrive together. Each pair runs on
the same layout with the same per-attempt random seed and recovery policy.
`bonus_comparisons` gives the additional planned navigation metres and nominal
travel seconds for interleaving, once per batch and layout. A negative value
means a saving over the best whole-command order. Both plans execute every
skill once in the nominal model, so their fixed skill times cancel. Elapsed
time and distance in the execution rows include the final return. The benchmark
compares observed successful work, completed goals and savings with the
issued-order baseline as simulation evidence. Official bonus credit requires
referee assessment. [GPSR rules, release 2026.2](https://github.com/RoboCupAtHome/RuleBook/blob/2026.2/tasks/GPSR.tex)

`plan_endgame` takes verified remaining actions, the current pose and holder,
nominal skill and travel times, the instruction point, and the remaining
budget. It maximizes the number of completed commands and breaks ties by
earliest return. It can stop and return with unfinished work. The default
one-second bins and experimental five-second bins round the available budget
down and each segment cost up. A 9-second report segment and a 5-second
return fit exactly in 14 seconds. Five-second bins charge 10 + 5 seconds,
so the same plan first becomes eligible at 15 seconds.
The fixture begins 395 seconds into the paper's 420-second trial. It assumes
a verified grasp and an observation already available to report. Its durations
are controlled inputs. Full-command credit and nominal durations define this
experiment; partial referee credit and uncertain durations require a scoring
and timing model.

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

`merger.py` no longer matches `home2` byte-for-byte; `improvements.md` lists every
change still to port. The solver is now a DP over per-command progress vectors
instead of subset Held-Karp (same optimum, no 16-segment cap). And one bug fix: a segment's gripper state is now taken from its *last* gripper
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

The simulator runs the repository's `merge()` and `build_tree()` with
`SimSubtaskHandler` as the skill adapter. The back-to-back demo uses the
per-command order in `plan.fallback`. Recovery and the initial ordering
are independent choices.

### Recovery trials

```bash
.venv/bin/python -m sim.run_sim --mode merged --no-video --recovery-policy skip-defer
.venv/bin/python -m sim.recovery_trials --out out/recovery
.venv/bin/pytest -q
```

Use `--navigation-overhead METRES` to add a fixed distance-equivalent cost
per positive-distance navigation leg in the command plan. The simulator
executes and times the return separately.

Merged simulation defaults to `skip-defer`. The `build_tree()` default is
`sequential-fallback`. Select `sequential-fallback`, `skip-only`, or
`skip-defer` with `--recovery-policy` to compare recovery on the same order.
Each trial records skill attempts, exhausted failures by source command,
actions skipped, spoken failure reports, and the return outcome.

The seven paired cases produce 21 policy runs and one nominal back-to-back
comparison. `out/recovery/results.json` records completed goals, partial
credit, elapsed time and distance including return, and goals recovered by
deferral. The fixture rubric awards one unit per correct report or delivery
and half a unit for an unfinished command with a verified pick or correct
intermediate observation. The [paper](main.tex) reports the comparison.
The simulator records evidence for the published bonus criteria in the
nominal case. Plan demonstration and official points require live evidence.
See the [home2 port notes](improvements.md#8-local-recovery-and-shared-deadline)
for the planned RoboCup 2027 integration.

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
| `run_sim.py` | CLI for merged and back-to-back runs with selectable recovery |
| `recovery_trials.py` | paired three-command paper trials and trace verification |

### The demo: `showcase_merge`

Four commands that each zig-zag between the north and south halves of the flat
(kitchen → garage, bedroom → bookshelf, entrance → bathroom, side table → office).
One after another they criss-cross the apartment; merged, the DP sweeps each region
once. The table measures time and distance before the final return. All 17 actions succeed in each run.

| | time | driven |
|---|---|---|
| merged | 119.1 s | 35.4 m |
| sequential | 161.6 s | 58.4 m |
| saved | **42.4 s (26%)** | **23.0 m (39%)** |

Time saved is smaller in relative terms than distance, because perception and
manipulation take the same time in both orderings. The merger only removes
driving. With shared attention enabled, `showcase_all_leaves` exercises all
13 skills and drives 56.8 m including return in both orderings.

`out/side_by_side.mp4` shows both runs on one clock, with a Gantt timeline
coloured by command. The merged bar interleaves colours; the sequential bar is four
blocks with long driving stretches. `out/side_by_side.png` is the final still.

The tree checks gripper state before every pick and verifies the matching
object before release. The adapter enforces per-skill deadlines inside
blocking calls using simulated time. Execution, recovery, and return share
420 seconds by default. In `run.json`, `elapsed_s` and `actual_distance_m`
include return. `task_time_s` and `work_distance_m` end before the return.
The optional video hold is outside the measurement.

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

Only `showcase_merge`, `showcase_all_leaves` and `showcase_exits` are fully provisioned. The 64 scenarios in
`scenarios/merger.json` are unit tests *for the scheduler*: they reference locations
like `A`/`loc7` and people like `Jane` that the apartment does not contain, so their
handlers can report `TARGET_NOT_FOUND`. The executor uses the navigation fallback
when a drive fails. Some fixtures resolve every `go_to`, but rendering
the rest meaningfully would mean populating the apartment to match each one.
`run_scenarios.py` remains the check that matters for the merger itself, and passes 64/64.

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
- `conflict_order_preserved` — conflicting actions of different commands (one writes
  object/person state at a place the other reads or writes) run in issue order, so
  the merged plan is conflict-equivalent to running the commands one by one.

Scope claims to *the merger*: the "first part of the first command goes missing"
symptom traced to upstream LLM decomposition, not to this code.

## Provenance / licensing — UNRESOLVED

- Source: `github.com/RoBorregos/home2`, `task_manager/task_manager/gpsr/`.
- **home2 has no LICENSE file.** Settle licensing with RoBorregos before this repo
  goes public.
- Two authors touched the folder:
  - `2965c2043` Added gpsr command merging (#937) — Luis Alvaro Rosales Salazar
  - `75c28b966` Gpsr/robocup (#1119) — Gilberto Malagamba Montejo

## Paper extensions and paired benchmark

The seven historical cases remain available through `python -m sim.recovery_trials`.
For a paired rerun with residual planning enabled, add `--replan --out out/recovery-replan`.
The flag also works in `sim.run_sim`. It extends the two local recovery policies.
After an exhausted failure, the executor removes unavailable work, restores the
entry drives for the surviving segments and solves again from the measured
pose and verified holder. The original action indices and the running deadline
survive each solve. An exhausted navigation failure excludes that destination
for the rest of the batch. The resulting order is recorded under `summary.replans`.
An uncertain gripper keeps the existing recovery guards in effect.

The planner accepts `skill_durations_s` and `speed_m_s`. It adds
`holding_penalty × speed_m_s × occupied_skill_seconds` to the distance objective.
The estimate treats grasp and release as changes at action completion.
Unoccupied constant skill durations cancel between complete schedules. The
simulator supplies nominal dwell estimates and logs measured `holding_time_s`
from the physical gripper state. The full numerical objective takes priority over
the historical command-order tie preference. Use `--holding-penalty 0.5` with
`sim.benchmark run` to enable the exposure cost. Add `--travel-exposure-only`
to isolate the effect of stationary dwell at the same weight.

Object counts and visual comparisons refer to a named fixture, or to fixtures
within the requested room. Objects have a semantic site as well as a physical
pose. Picking removes the object from its site and release assigns the new site.
Sensor range still limits visibility. The count handler also accepts people and
pose descriptions such as “standing people”.

### Reproduce the comparison

With the simulation dependencies installed, the checked-in manifest runs offline.

```sh
python -m sim.benchmark run scenarios/benchmark_2027.json --repeats 2 --out out/benchmark/study
python -m sim.stochastic scenarios/benchmark_2027.json --out out/benchmark/oracle-open.json
python -m sim.stochastic scenarios/benchmark_2027.json --apartment sim/apartment_partitioned.json --out out/benchmark/oracle-partitioned.json
```

The manifest has eight three-command batches sampled from four supported goal
families in the official grammar. It records the generator revision, seed,
template bindings, original text, spoken text, rephrasings, parsed actions and
independent expected goals. A small English grammar handles the default
utterances. To generate fresh batches from a checkout of the official generator:

```sh
python -m sim.benchmark generate --generator ../CommandGenerator --seed 2028 --batches 16 --output out/benchmark/new.json
```

An optional `--language-adapter PATH` runs an executable that reads a JSON
request on stdin and writes JSON on stdout. Operation `rephrase` receives
`text` and `seed` and returns up to three strings. Operation `parse` receives
only the selected text and returns the GPSR action list. Keep the model version
and prompt fixed in the adapter. Generated manifests preserve its file hash,
phrasing choices and outputs. The included run used the grammar with the
original utterances.

Each batch runs in an open apartment and a partitioned variant under one-by-one
(with operator returns), back-to-back, greedy, all three historical merged
policies, and skip-and-defer with residual planning. The first three orderings
share skip-only recovery. One-by-one models the inter-command drive, with zero
speech acquisition time. A keyed draw for each original command, action, skill
and attempt pairs faults across methods. The default sweep uses 0%, 10%, 20%
and 40% per-attempt failure probabilities. The simulator measures deliveries
and escorts in the final world state, checks named handovers and checks the
actual report at the operator. For a count, the scorer replays the object-site
changes earlier commands actually achieved in issue order. The manifest also
retains the nominal expected answer. Each solved goal earns one pilot unit and a
verified intermediate goal earns half a unit. The bonus-evidence metric also
requires a completed baseline and a time or distance saving. Official
competition points require a referee's score sheet.

Each run writes its event log. `results.json` contains paired rows and
mean/sample standard deviation by method and rate, including distance, time,
occupied time, goal units and observed gripper/context violations. To fit an
empirical model, provide a JSON array of individual skill attempts with
`skill`, boolean `success` and `duration_s`, including failures:

```sh
python -m sim.benchmark fit robot-attempts.json --output out/benchmark/skills.json
python -m sim.benchmark run scenarios/benchmark_2027.json --skills out/benchmark/skills.json --rates 0 --out out/benchmark/calibrated
```

The fitted model uses empirical per-skill failure frequencies and resamples
durations conditional on outcome. In MuJoCo, successful duration has a floor
set by the physical trajectory. The supplied rate applies to unobserved skills.

### Stochastic reference model

`sim.stochastic` compares exact backward induction to choosing a segment under
the assumption of future success and resolving after each outcome. Both
policies see the same dependencies, gripper state, one-second clock, routes,
skill durations and partial-goal rewards. The objective is expected elapsed
seconds minus `--seconds-per-goal` times pilot units. It includes the terminal
return or time up to the deadline. Skills within a segment execute in order,
information actions can retry once, and an exhausted failure drops the
remaining command. A failed release retains the holder. At the deadline an
unfinished segment earns no new credit. Navigation in this abstract model is
deterministic. The model treats success as satisfying the corresponding
abstract task. Its optimal value bounds policies in this model. Physical
execution, local deferral, semantic observation errors and competition points
are evaluated separately. Use `--skills` to supply fitted outcome and duration
distributions to this comparison.
