# Future work — what `main.tex` describes that the code doesn't do yet

Remaining work after the local recovery study. Completed changes and robot
integration requirements are tracked in `improvements.md`.

Size: S = a day or less · M = a few days · L = a project of its own.

## §II-D Objective — the step cost g(x, h, s)

| term | paper | code now | needed | size |
|---|---|---|---|---|
| d / v | travel time | metres (v = 1) | one sentence in the paper: with v constant the argmin is the same | — |
| λ on hops | `(1 + λ·[h ≠ ∅])·d` | ✅ `holding_penalty` | pick λ: from the score sheet or a held-out set of batches (paper todo) | S |
| ρ(s, h) | holding time *inside* a segment | ✅ distance part (escort legs while holding) | stationary dwell is implemented from per-skill estimates. Calibrate on robot logs | S + robot logs |
| κ | fixed overhead per real move | opt-in `navigation_overhead` on positive-distance navigation in the command plan | measure the robot's overhead and convert seconds to distance-equivalent cost | robot logs |
| τ(s) | action durations | configurable stationary durations and a stochastic duration model | changes no order by itself (every schedule runs every segment); needed for ρ's time part, the deadline and the MDP | S + robot logs |

## §III-A Algorithm 1

- **g₀ (initial gripper state).** Residual solves accept the verified holder. Validate this on the robot.
- The proof sketch describes a backward pass; the code runs the equivalent
  forward pass in layers of Σk. Both are exact — optional sentence.

## §III-B Objectives that are not additive

- **Interleaving bonus.** The paired benchmark now measures both values of the
  terminal bit. Calibrate a reward and compare its predictions with referee
  decisions about meaningful savings.
- **Time limit.** The deterministic endgame experiment includes return time
  and tests one- and five-second bins. Validate travel and skill times on the
  robot, then evaluate partial credit and uncertain durations on held-out batches.

## Replanning and stochastic recovery

- Validate the opt-in residual solve on physical runs. The simulator already re-solves from the measured pose and verified holder.
- Fit the stochastic reference model to robot logs. The abstract solver already compares optimal and certainty-equivalent decisions on small batches. Extend the model to cover deferral and navigation failures.
- Integrate cancellation in the live subtask managers so the shared clock
  caps each blocking skill.

## §II-A / Table II — modelling differences

- **Person attention on the robot.** The local merger orders shared attention
  reads and writes across places. Port the rule to `home2` and repeat the
  two-person pose test with the robot camera.
- Table II verify-todo: `give_object` fixed 2026-11-21 (writes objects).
  The rest matches `_EFFECTS`; reads are a superset (conservative).

## §IV GPSR-Sim benchmark

- Broaden the four supported official-generator goal families. The current benchmark has eight saved batches with independent goals and a language-adapter interface. Run it with the competition parser and a pinned rephrasing model.
- Evaluate independent arenas beyond the open loft and its partitioned variant.
- Collect robot skill-attempt logs for the empirical model and measure target absence rates.
- **GPSR score sheet** with partial credit and the interleaving bonus. M
- **Fidelity tiers**: an abstract sim running thousands of trials, plus a
  subset in Gazebo / Isaac Sim with the full ROS 2 stack. L

## Broader experiments

- Extend the paired trials beyond scripted faults to a held-out set of
  official generator batches, varied layouts, and fitted skill durations.
- Extend the paired greedy and one-by-one comparisons to PDDL and language-model merging. Evaluate the effect of the dependency analysis,
  exit locations, holding penalty, and planner objective extensions.
- Compare the fixture rubric with observed referee scoring on the robot.

## Robot (the Orin)

- **Planner runtime** on the robot's Orin: `python3 bench_runtime.py --latex`.
- **Logs from real batches** to fit κ, τ and p_k (for the travel-time objective and stochastic evaluation).
- **RoboCup 2027 results** after live execution and referee scoring.
- **Shared command history in home2**: `GPSRTM._on_action_complete` stores
  results without `source_cmd`, so `say_with_context` can read another
  command's result. Fix in home2 (see `possibleImprovements.md` #13).

## Introduction and figures

- Measure the earlier competition system's wrong answers: "N incorrect
  answers in M simulated batches" (`dependencies=False` plus the shared
  history).
- Teaser figure: merged vs sequential trajectories with a Gantt bar per
  command (`sim/compose.py` already renders this for the video).
