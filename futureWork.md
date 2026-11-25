# Future work — what `main.tex` describes that the code doesn't do yet

Remaining work after the local recovery study. Completed changes and robot
integration requirements are tracked in `improvements.md`.

Size: S = a day or less · M = a few days · L = a project of its own.

## §II-D Objective — the step cost g(x, h, s)

| term | paper | code now | needed | size |
|---|---|---|---|---|
| d / v | travel time | metres (v = 1) | one sentence in the paper: with v constant the argmin is the same | — |
| λ on hops | `(1 + λ·[h ≠ ∅])·d` | ✅ `holding_penalty` | pick λ: from the score sheet or a held-out set of batches (paper todo) | S |
| ρ(s, h) | holding time *inside* a segment | ✅ distance part (escort legs while holding) | time part (dwelling while holding) needs τ | S + robot logs |
| κ | fixed overhead per real move | opt-in `navigation_overhead` on positive-distance navigation in the command plan | measure the robot's overhead and convert seconds to distance-equivalent cost | robot logs |
| τ(s) | action durations | not modelled | changes no order by itself (every schedule runs every segment); needed for ρ's time part, the deadline and the MDP | S + robot logs |

## §III-A Algorithm 1

- **g₀ (initial gripper state).** `merge()` assumes an empty gripper. Only
  matters when planning mid-batch, i.e. for replanning.
- The proof sketch describes a backward pass; the code runs the equivalent
  forward pass in layers of Σk. Both are exact — optional sentence.

## §III-B Objectives that are not additive

- **Interleaving-bonus bit** b in the state (×2 states). Needs the technical
  committee's definition of "meaningful" interleaving. M
- **Time limit**: elapsed time t in the state, Δ = 1 s, T_max = 420 s
  (315,000 states with the bonus bit). Needs τ and κ in seconds. Note
  `_STATE_CAP` is 2×10⁵, below that count. M

## Replanning and stochastic recovery

- Re-solve from the live pose and verified gripper state when a recovery
  changes the remaining work.
- Compare replanning and local recovery with an exact stochastic policy
  on small batches. Model elapsed time, skill outcomes, pose, gripper state
  and goal progress. Fit the outcomes and durations to robot logs.
- Integrate cancellation in the live subtask managers so the shared clock
  caps each blocking skill.

## §II-A / Table II — modelling differences

- **"Attention"** (the person attended to) is a state variable in the paper;
  the code keys people by place instead. Equivalent while find/use stay in one
  segment, which is how the parser emits them.
- Table II verify-todo: `give_object` fixed 2026-11-21 (writes objects).
  The rest matches `_EFFECTS`; reads are a superset (conservative).

## §IV GPSR-Sim benchmark

- Batches from the **official command generator**, each rephrased up to 3× by
  a language model (model + prompt to fix); ground-truth goals from the
  generator. L
- **Several layouts with interior walls** (hand-built and/or HSSD). The sim has
  one apartment, no interior walls. M–L
- **Stochastic skills**: success p_k and duration distributions fitted from
  robot logs; people/objects absent with probability X. M + robot logs
- **Final-state scoring beyond the paper fixtures**, including escort and
  handover goals. M
- **GPSR score sheet** with partial credit and the interleaving bonus. M
- **Fidelity tiers**: an abstract sim running thousands of trials, plus a
  subset in Gazebo / Isaac Sim with the full ROS 2 stack. L

## Broader experiments

- Extend the paired trials beyond scripted faults to a held-out set of
  official generator batches, varied layouts, and fitted skill durations.
- Compare greedy ordering, one-by-one execution with returns, PDDL, and
  language-model merging. Evaluate the effect of the dependency analysis,
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
