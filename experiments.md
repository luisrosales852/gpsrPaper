# Experiments for the GPSR paper

Use [main.tex](main.tex) as the submission manuscript. The central questions are whether the merger produces a correct interleaving, whether the robot saves time or motion while completing the same goals, and what it gains or loses by revisiting a failed information task. The first question can be tested on a parked Orin. The other two need independently observed physical outcomes.

| When | Study | Deliverable for the paper |
|---|---|---|
| Orin parked | Correctness, solver latency, spoken commands, scripted failures | Runtime table, error-by-stage table, regression traces |
| Workstation or parked Orin | Paired simulation across batches, layouts and faults | Recovery table, failure-rate figure, ablations |
| Robot moving | Skill calibration, paired ordering trials, paired recovery trials | Hardware results, trajectories and action timelines |
| After the system is frozen | Unseen spoken triples and any official runs | End-to-end outcomes and observed scores |
| Optional appendix | Deadline-aware endgame and stochastic policy comparison | Results for their respective timing and failure models |

The sample sizes below are proposed pilot targets. Count distinct batches, physical resets, speakers and repeated solver calls separately. A hundred timings of one input measure jitter on that input. They contribute one batch to an ordering comparison.

## 1. Parked Orin

Run the shell examples from the paper checkout with the matching `home2` checkout at `../home2`. Use the tested paper and robot revisions on the Orin and record each commit and any working-tree diff. Record the Orin model, power mode, JetPack and Python versions, and active workload. Check the imported file in a sourced ROS environment as well as the source checkout to catch a stale installed package.

### Planner correctness and latency

Run the following against the copied robot module. This process imports only the planner and test helpers. It sends no ROS goals. The pytest cases use an independent ordering oracle for travel, an exhaustive six-segment objective check, and dependency and gripper checks.

```sh
PYTHONPATH="../home2/task_manager${PYTHONPATH:+:$PYTHONPATH}" python3 - <<'PY'
import sys
import pytest
from task_manager.gpsr import merger

print("Testing", merger.__file__, flush=True)
sys.modules["gpsr_merger.merger"] = merger
import run_scenarios

if run_scenarios.main():
    raise SystemExit(1)
raise SystemExit(pytest.main(["-q", "tests/test_progress_dp.py",
                             "tests/test_dependencies.py", "tests/test_paper_objective.py"]))
PY
python3 bench_runtime.py
```

The current reference is 64 scenarios and 44 pytest cases. Include these targeted comparisons in the case log. For each, save both emitted plans with source command and action indices.

| Input | Evidence of correctness |
|---|---|
| Find Angel, read Angel's pose, and separately find Tom along the shortest route | Read Angel's pose before the search for Tom changes the attended person. |
| Visit the chairs, guide someone to the bathroom, then count at the chairs | The emitted actions include the return to the chairs before the count. |
| Pick a cup from a counter, then count cups in the containing room | The room and fixture resolve consistently. The count observes the issue-order effect of the pick. |
| A command starts with an instruction-point action that the gripper forces later | With a confirmed named start, the plan drives back before that action. |
| A release and another pick share a segment while a second command needs the gripper | The segment ends occupied. The second command waits until release. |

For the hardware latency table, save the historical module from `git -C ../home2 show 92800e8a2:task_manager/task_manager/gpsr/merger.py` in a temporary directory and import it in a separate benchmark process. Compare it with the port on identical parsed triples, coordinates and origin. Use the eight saved benchmark batches plus a three-command, four-segment-per-command stress case. Time a warmed public `merge()` with a monotonic clock for 100 calls per batch, first idle and then with the normal speech and perception services active. Repeat with the planner settings used on the robot. Save each sample in milliseconds. Report per-batch median and p95, segment count, and how often a fallback was selected. Measure live location resolution and path queries separately. The existing `bench_runtime.py` reports best-of-repeat solver time and a single public-call time. Its above-cap row times a direct solver invocation that the production entry point bypasses. [Runtime benchmark](bench_runtime.py)

### Spoken command to plan

Have two speakers each read the same ten held-out triples in counterbalanced order. Include similar person names, two commands that request the same type of fact, and a multi-clause instruction whose first clause can be lost. Freeze the ASR model, parser model, prompt, map vocabulary and independently labeled goals before recording. Use the microphone, speech recognizer and command interpreter, then call the public planner and stop after recording its output. The GPSR node's `test_mode` goes on to dispatch skills, so use the individual services or offline audio replay for this experiment.

For each utterance retain audio, reference text, transcript, parsed actions and arguments, merged order, and the timed plan announcement if enabled. Replay the reference transcript through the same parser and the reference actions through the same planner. Attribute the first semantic error to recognition, parsing, or merging. Report correct triples out of 20 speaker–batch trials, per-command counts, missing clauses, rephrasings, and median/p95 time from end of speech to plan availability. An observer can assess whether the spoken announcement matches the actual plan. The experiment supplies evidence for the speech, parsing and merging rows of the manuscript.

### Recovery traces without actuating the robot

Use simulated handlers or explicit test doubles for every action. Replay the same plan under sequential fallback, skip only, and skip-and-defer. Check the nominal case, one immediately recoverable failure, exhausted perception with a successful revisit, persistent perception failure, failed pick with shared navigation, failed release, and insufficient time for a revisit. Also test navigation failure and two similar reports with different known answers. Inspect which work ran or was skipped, the owner of the held object, answer provenance, return admission and elapsed simulated time. The seven paired fixtures have a runnable harness. [Paired cases](sim/recovery_trials.py) · [Recovery assertions](tests/test_recovery.py)

```sh
python3 -m pytest -q tests/test_recovery.py
python3 -m sim.recovery_trials --out out/experiments/recovery
```

The `home2/gpsrPaper` branch also contains the paper executor. The batch node defaults to `recovery_policy=skip-defer`. Set `skip-only` or `sequential-fallback` for paired comparisons and `replan_on_failure=true` for the residual-planning variant. It keeps successful facts by source command, reads fresh grasp feedback and checks the final TF pose against the instruction point. Missing or stale grasp readings prevent a new pick or release. Check the actual detector's polarity, update rate and placement on the parked robot.

Build and source matched versions of `frida_interfaces`, `frida_constants`, `nav_main` and `task_manager` in the robot's ROS environment. GPSR named drives use the additional `/navigation/go_to_map_area_timed` service. Its request carries an absolute ROS-clock deadline, so the navigation and task-manager processes need the same time reference. With the ROS environment and generated BAML types available, the following checks exercise the adapter and Nav2 cancellation with doubles, without constructing a robot node or dispatching real actions.

```sh
python3 -m pytest -q ../home2/task_manager/test/test_gpsr_batch_adapter.py ../home2/navigation/packages/nav_main/test/test_timed_navigation.py
```

To check the entire port against the paper suite, run the following from the paper checkout with its Python dependencies installed. It uses the modules from the robot checkout for planning, execution, recovery and the optional endgame.

```sh
PYTHONPATH="../home2/task_manager${PYTHONPATH:+:$PYTHONPATH}" python3 - <<'PY'
import importlib
import sys
import pytest

for module in ("status", "merger", "timeouts", "leaf_behaviours", "recovery", "bt_builder", "endgame"):
    port = importlib.import_module(f"task_manager.gpsr.{module}")
    print(port.__file__)
    sys.modules[f"gpsr_merger.{module}"] = port
raise SystemExit(pytest.main(["-q", "tests"]))
PY
```

For live deadline validation, record the actual stop time of each subsystem as well as the tree's timeout. Test an outstanding named drive, an arm goal, a slow perception request and operator speech. The executor shares a 420 s budget through return. Synchronous backends differ in their support for preemption. A late result or a cancellation request alone cannot establish when a physical operation stopped.

## 2. Paired simulation and ablations

Reproduce the seven-case recovery table before varying the experiment. Reference values for the nominal merged and issued-order runs are 102.0 s / 33.2 m and 122.8 s / 44.9 m through return. In the recoverable perception case the reference goal counts are 3 for fallback, 2 for skip, and 3 for defer. In the persistent case defer adds 32.2 s over skip for zero additional goals. Report all cases, including the incomplete return. Keep deterministic fixtures distinct from random trials.

Start the larger comparison with the checked-in eight triples and two layouts. Generate a fresh held-out manifest with the official generator and save its revision, seed, sampled commands, independent goals, and rejected or unsupported families. The checked-in manifest covers four families. If evaluating language, also freeze the rephrasing and parsing adapters and their versions. Split by base command batch before rephrasing so paraphrases of one batch stay together. Reserve independent calibration batches for selecting parameters.

```sh
python3 -m sim.benchmark run scenarios/benchmark_2027.json --rates 0 0.1 0.2 0.4 --repeats 5 --out out/experiments/rates
python3 -m sim.benchmark run scenarios/benchmark_2027.json --bonus-comparison --rates 0 0.2 --repeats 5 --out out/experiments/bonus
python3 bench_holding.py
```

Use identical batch, layout, initial state and keyed fault draws across policies. Compare one-by-one, all-at-once issued order, nearest eligible segment, and the paper merger. Hold recovery constant for ordering comparisons. Hold the merged plan constant for the three recovery policies. Evaluate residual replanning as a separately labeled variant. Plot solved goals and on-time returns against per-attempt failure rate. Report elapsed time including return, distance, actual time holding an object, violations, and partial goal units. Time the simulation separately if claiming simulator throughput. [Benchmark methods](sim/benchmark.py)

For the ablation table, change one factor per pair: cross-command dependencies, exit location of escorts, gripper capacity, map path versus Euclidean distance, navigation overhead κ, holding penalty λ, stationary holding time, and recovery versus restart. Include both classes of the interleaving bit alongside unrestricted planning and issued order. Implement missing ablations in the offline harness before including those rows. Run variants that relax gripper or location constraints only in simulation. Score the actual answer and physical goal independently of the planner's own cost. For the introductory history claim, replay the historical behavior on a labeled two-report batch and count wrong answers over a defined set of batches. If measuring history leakage, compare shared versus command-scoped history with the plan held fixed.

## 3. Moving robot

### Calibrate on separate runs

On a fixed map, measure repeated short and long navigation legs, a doorway or wall detour, and repeated visits to the same site. Include empty and occupied gripper runs. Use independent final poses, actual base trajectories and timestamps. Estimate travel speed from moving time and fixed navigation overhead from the remaining dispatch-to-arrival interval. Time stationary work separately from travel and grasp-to-release exposure. Log every skill attempt, including failures, retries and timeouts. Estimate per-skill success probability and duration conditional on outcome, and report counts. Select λ and any revisit margin on calibration batches and freeze them before evaluation. In the merger API, pass κ in distance-equivalent units, `speed_m_s * overhead_seconds`.

The robot planner requests Nav2 path lengths when `use_nav_paths=true` and the query service is ready. Named legs are cached within a batch. Current-pose legs are refreshed after motion. The planner falls back to Euclidean coordinates when the service is unavailable at planning time. Set `use_nav_paths=false` for the Euclidean ablation. Record service availability, unresolved legs and lookup overhead. Record `holding_penalty`, `navigation_overhead_m`, `speed_m_s`, and `skill_duration_s.<skill>` with each run. Use the fitted skill model for a held-out simulation rerun. [Fitting input schema and commands](README.md#reproduce-the-comparison)

### Does merging save motion while preserving goals?

Start with six distinct triples, two policies and two resets per policy (24 runs). Include a shared stop, a wall detour, an escort ending in a different room, an object move followed by a count, two reports with different ground truth, and a low-overlap batch with little opportunity to save motion. Use identical **parsed** triples, start poses, object placements and person scripts in each pair. Counterbalance which policy runs first. Both methods receive the whole batch and use the same skill handlers, recovery policy and final return. Build the back-to-back baseline by flattening the original commands in issue order with equivalent redundant-drive handling. In the robot node, `interleave_enabled=false` changes command intake to one-by-one. In the paper merger, `interleaved=False` can reorder whole commands. Neither switch implements this paired baseline.

For every run, score correct goals out of three and arrival at the instruction point within the common budget. Also report elapsed seconds, meters from the measured trajectory, occupied-gripper seconds, report correctness, unauthorized gripper transitions and wrong-location actions. Compare travel/time savings among equal-outcome pairs and show the outcomes of **all** pairs, including failures. Plot both trajectories and a command-colored action timeline for one prespecified batch. Mark pick, release, report and return.

### Does local recovery pay for the revisit?

After the robot recovery hooks pass parked replay, compare fallback, skip only, and skip-and-defer on the same initial merged plan. Start with five conditions: nominal, recoverable occlusion, persistent occlusion, failed pick with an independently needed drive, and insufficient budget for a revisit. Use two resets per policy per condition (30 runs) as a pilot. Add a failed-release case as soon as the gripper's occupied/uncertain signal can be observed independently. Trigger scripted faults by original command, action and attempt. For a recoverable occlusion, reveal the target after the immediate attempts, in the same way for each policy. For a deadline trial, record the budget already consumed at batch start.

Measure goals recovered, wrong or duplicate reports, extra distance/time, surviving independent work, gripper ownership, and return by deadline. Verify releases, delivery sites and reported answers with observation or video. A handler success code alone is insufficient. Keep natural failures in the logs and distinguish them from deliberate injections. Attribute every failure to the earliest observed cause, including execution and return.

### End-to-end spoken trials and scoring

Once parameters are frozen, draw fresh held-out triples and execute with live speech, plan demonstration, skills and return. Start with six triples and counterbalance across speakers and orderings. Record rephrasing, human assistance, what the operator actually heard, independent task outcomes, and measured time/distance. Start the clock at the agreed trial boundary and log pauses and time spent before planning. Apply the stated scoring edition consistently. Report lab-assessed bonus eligibility and any awarded competition points in separate fields. Official competition results belong in the manuscript when an actual score sheet exists. [GPSR procedure](https://github.com/RoboCupAtHome/RuleBook/blob/2026.2/tasks/GPSR.tex) · [Seven-minute score sheet](https://github.com/RoboCupAtHome/RuleBook/blob/2026.2/scoresheets/GPSR.tex)

## 4. Optional comparisons already named in the drafts

| Question | Experiment and evidence needed to keep the claim |
|---|---|
| What should the robot do near the deadline? | Replay the two-job fixture from [main-endgame.tex](main-endgame.tex). Sweep 0–45 s in 1 s and 5 s bins against exhaustive enumeration. Then replace fixture times with held-out measured durations, vary the start pose and remaining budget, and compare ordinary ordering with deadline-aware choice on actual goals and on-time return. |
| What is the gap to an optimal stochastic policy? | Run `python3 -m sim.stochastic scenarios/benchmark_2027.json --out out/experiments/oracle.json` for each layout (supply `--apartment sim/apartment_partitioned.json` and a different output on the second run). Compare exact backward induction and receding-horizon decisions under identical duration, outcome, deadline and reward models. Report expected value differences by failure rate and an example of a changed decision. The bound applies to the specified abstract model. |
| Are PDDL or LLM merging competitive? | If retaining these baseline rows, implement the PDDL domain and pin the LLM version, prompt, seed policy, timeout and invalid-plan handling. Give them identical parsed goals, a common plan validator, action costs, and the same execution policy. Report planning time and invalid plans as well as execution outcomes. |
| Do simulated conclusions transfer across environments? | Add a distinct arena and, if retaining the high-fidelity simulation claim, replay a prespecified subset using the full ROS navigation stack. Compare measured paths, action times and ordering of policy outcomes against the paired pilot. |

```sh
python3 -m pytest -q tests/test_endgame.py
python3 -m sim.endgame scenarios/endgame_25s.json --remaining 13 14 25 35 36 40 --bins 1 5 --out out/experiments/endgame.json
```

At 25 s the fixture predicts report and return in 14 modeled seconds. Delivery and return take 27 s. The minimum nominal time to do both and return is 36 s. Keep these as fixture results until physical durations and return outcomes are available.

## 5. Analysis and submission

Keep one row per run with batch ID, policy and settings, layout, seed/reset, clock boundary, goal outcomes and return status. Retain one event per attempt with original command/action IDs, target, timestamps, result, measured pose, gripper observation and recovery branch. Connect each run to its input manifest, raw trace, independent labels and software revisions. Report missing telemetry, interventions and deadline-censored runs explicitly.

Use the batch as the unit for paired inference. Publish paired differences and denominators, plus descriptive medians/ranges for small physical pilots. With a larger held-out collection, bootstrap **batches** for confidence intervals, preserving all repeats and policies within a sampled batch. Report solved goals, partial pilot units and referee points in their own units. Size any follow-up from the variability of paired differences observed in the pilot.

Populate the runtime and failure-stage tables from parked measurements, the ordering and robustness figures from paired comparisons, and the hardware results from observed goals and returns. For each planned table row in [main.tex](main.tex), either attach completed evidence or move the comparison to future work. Use the measured cost of unsuccessful revisits as well as successful recoveries in the conclusion.
