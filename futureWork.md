# Future work — what `main.tex` describes that the code doesn't do yet

Audit of 2026-11-21, by paper section. Nothing is cut from the paper; each item
is either implemented later or reworded once results are in. Done items move to
`improvements.md` (the home2 port notes).

Size: S = a day or less · M = a few days · L = a project of its own.

## §II-D Objective — the step cost g(x, h, s)

| term | paper | code now | needed | size |
|---|---|---|---|---|
| d / v | travel time | metres (v = 1) | one sentence in the paper: with v constant the argmin is the same | — |
| λ on hops | `(1 + λ·[h ≠ ∅])·d` | ✅ `holding_penalty` | pick λ: from the score sheet or a held-out set of batches (paper todo) | S |
| ρ(s, h) | holding time *inside* a segment | ✅ distance part (escort legs while holding) | time part (dwelling while holding) needs τ | S + robot logs |
| κ | fixed overhead per real move | not modelled | `step += κ` when the hop length > 0; value measured on the robot | S + robot logs |
| τ(s) | action durations | not modelled | changes no order by itself (every schedule runs every segment); needed for ρ's time part, the deadline and the MDP | S + robot logs |

## §III-A Algorithm 1

- **g₀ (initial gripper state).** `merge()` assumes an empty gripper. Only
  matters when planning mid-batch, i.e. for replanning.
- The proof sketch describes a backward pass; the code runs the equivalent
  forward pass in layers of Σk. Both are exact — optional sentence.

## §III-B Objectives that are not additive

- **Interleaving-bonus bit** b in the state (×2 states). Needs the technical
  committee's definition of "meaningful" interleaving. M
- **Time limit**: elapsed time t in the state, Δ = 1 s, T_max = 300 s
  (≈2.3×10⁵ states with the bonus bit). Needs τ and κ in seconds. Note
  `_STATE_CAP` is 2×10⁵, below that count. M

## §III-C Execution and replanning

Today: Retry(2) per action; any action that still fails abandons the
interleaved branch and runs every command sequentially (with resume).

- **Information failures continue the plan** (count, get_visual_info,
  get_person_info, find_person): retry once, record the failure in the
  command's register, report it when the answer is due. No replanning needed:
  the remaining segments don't change. S–M
- **Manipulation failures drop the command's dependent segments** (up to its
  release). Without replanning, skipping them in place keeps the plan feasible
  (a subsequence of a feasible schedule respects ≺ and the gripper) but maybe
  not travel-optimal. S–M
- **Navigation failures mark the location unreachable** for the rest of the
  batch. M
- **Replanning**: re-solve the DP from the current pose and gripper state
  after any change (needs g₀, and the tree rebuilt mid-run). Paper: "a solve
  takes X ms". M
- **Non-blocking leaves** reporting RUNNING. Robot side (home2 subtask
  managers); timeouts are enforced inside the skills today. M

## §III-D Optimality under failure

- **Exact MDP** over (k, ℓ, f, b): success / info failure / manipulation
  failure outcomes, terminal −β·Score. ≈2×10⁴ states. Needs p_k and τ from
  robot logs and the score sheet. Gives Table `tab:gap`. L
- Decide whether the exact policy replaces replanning on the robot.

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
- Paper says distances are **occupancy-grid path lengths**; the sim uses an
  exact visibility graph. Reword or add a grid router behind `routing.base`. S
- **Stochastic skills**: success p_k and duration distributions fitted from
  robot logs; people/objects absent with probability X. M + robot logs
- **Judging on the final world state** (object placed, person guided, answer
  matches ground truth), not on skill status codes. M
- **GPSR score sheet** with partial credit and the interleaving bonus. M
- **Fidelity tiers**: an abstract sim running thousands of trials, plus a
  subset in Gazebo / Isaac Sim with the full ROS 2 stack. L

## §V Experiments

- **Batch runner** producing Table `tab:main`, Fig. `fig:failure`,
  `tab:gap` and `tab:ablation` from many sim batches. M
- **Baselines**:
  - one-by-one (return to the operator between commands): S;
  - back-to-back: exists (`plan.fallback`);
  - greedy (nearest eligible segment; also a better fallback above
    `_STATE_CAP`): S;
  - PDDL with Fast Downward (domain file): M;
  - LLM merge (model + prompt): M;
  - Ours (exact MDP): needs §III-D.
- **Ablations**: −dependencies, −gripper, −exits, Euclidean exist as switches.
  −repair, κ = 0 and −bonus bit need their features.
- **Metrics**: score, % solved, % bonus, time, distance, violations.
  Violations are checked by the sim tests; score and bonus need the score
  sheet.

## Robot (the Orin)

- **Planner runtime** on the robot's Orin: `python3 bench_runtime.py --latex`.
- **Logs from real batches** to fit κ, τ and p_k (feeds §II-D, §III-D, §IV).
- **RoboCup 2026 result** sentence (abstract, §V).
- **Shared command history in home2**: `GPSRTM._on_action_complete` stores
  results without `source_cmd`, so `say_with_context` can read another
  command's result. Fix in home2 (see `possibleImprovements.md` #13).

## Introduction and figures

- Measure the earlier competition system's wrong answers: "N incorrect
  answers in M simulated batches" (`dependencies=False` plus the shared
  history).
- Teaser figure: merged vs sequential trajectories with a Gantt bar per
  command (`sim/compose.py` already renders this for the video).
