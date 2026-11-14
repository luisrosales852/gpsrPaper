# Possible improvements — what `main.tex` claims that the code doesn't do yet

Audit of the paper draft against the code, 2026-11-11. Done items move to
`improvements.md`, which is the list of changes to port to home2.

Status: ✅ done · 🚧 in progress · ⬜ not started

| # | item | paper section | where the work is | size |
|---|---|---|---|---|
| 0 | ✅ Progress-vector DP | §Method, Alg. 1 | `merger.py` | done |
| 1 | ✅ Cross-command dependencies (read/write sets) | §Interference, Table I, abstract | `merger.py` | small |
| 2 | ⬜ Exit locations for guide/follow | §Commands and segments | `merger.py` | small |
| 3 | ⬜ Commands with no leading `go_to` | §Commands and segments | `merger.py` | small |
| 4 | ⬜ Full step cost g(x, h, s): κ, λ, time | §Objective | `merger.py` | small |
| 5 | ⬜ Greedy baseline (and better fallback above the cap) | §Baselines | `merger.py` | small |
| 6 | ⬜ Replanning after a failure | §Execution and replanning, title | `merger.py` + `bt_builder.py` + `leaf_behaviours.py` | medium |
| 7 | ⬜ Interleaving-bonus bit, 420 s time limit | §Objectives that are not additive | `merger.py` | medium |
| 8 | ⬜ Exact failure MDP + replanning-vs-optimal gap | §Optimality under failure | new module | large |
| 9 | ⬜ GPSR-Sim benchmark | §GPSR-Sim | `sim/` + generator | large |
| 10 | ⬜ PDDL and LLM-merge baselines | §Baselines | new | medium–large |
| 11 | ⬜ Non-blocking leaves (`RUNNING`) | §Execution | robot only | medium |
| 12 | ⬜ Planner runtime table from a committed script | §Planner runtime | `tests/` or `bench/` | tiny |
| 13 | ⬜ Per-command result registers in the sim | §Interference (register_c) | `sim/handlers.py` | tiny |

---

## 1. Cross-command dependencies ✅ (see improvements.md §2)

The abstract claims "answer errors caused by naive interleaving are
eliminated". Without this, the merger reorders segments of different commands
freely. For example, if A picks the cup from the table and B counts the cups on
the table, B can end up counting after A's pick and report the wrong number.

Each action reads and writes state variables (object and person state at a
location). Two segments of different commands *interfere* if one writes what
the other reads or writes. When they do, the command that was given first goes
first. With progress vectors this is one more eligibility check in the
transition: B may advance to segment j only if `k[A] ≥ i`.

Consequence worth stating in the paper: every conflicting pair is ordered as in
sequential execution, so the merged plan is **conflict-equivalent to running
the commands one by one** (conflict serializability, as in databases). Every
answer and every world effect is the same as in the sequential baseline; only
travel changes.

## 2. Exit locations

The paper says a segment containing `guide_person_to` (`destination_room`) or
`follow_person_until` (`destination`) *ends* at that destination. The merger
assumes the robot ends at the segment's `go_to` target. Fix: give `_Segment`
an `exit`, and use `edge[exit(here)][entry(u)]`. This closes part of the
`cost_model_gap_m` the sim measures.

## 3. Commands with no leading `go_to`

The paper says leading actions attach to the robot's current position. The
code makes them a segment with `location=None`, which costs `LARGE_M`, so
"find a person here, then…" is treated as an unknown place.

## 4. Full step cost g(x, h, s)

The paper's cost has three terms the code lacks: κ, a fixed overhead per
actual move (rewards grouping at the same location); λ, a penalty for driving
while holding something; and conversion of distance to time. The progress
state already gives the holder `h`, so both fit in the transition.
**Paper fix:** τ(s), the duration of non-navigation actions, does *not* change
the best order, because every schedule runs every segment once. It only
matters with a deadline (#7).

## 5. Greedy baseline

At each step, take the nearest segment that's allowed to run next. It's in the
paper's baseline list, it's O(M·n), and it's a better fallback above
`_STATE_CAP` than plain sequential.

## 6. Replanning after a failure

The title says "under Execution Failure". Today a failure falls back to running
each command again with skip-if-done. The paper describes instead:
- an info action that fails is retried once, then the plan continues;
- a failed manipulation drops the command's segments that depend on the object;
- a navigation failure marks the location unreachable;
- then the DP is re-run from the current pose and gripper state.

Merger side: `merge()` needs an initial-holding input (g₀ in Alg. 1); the
post-failure state is a progress vector plus a pose. Executor side:
`bt_builder` / `leaf_behaviours` must rebuild the tree after a failure.

## 7. Interleaving-bonus bit and the time limit

Bonus: one extra bit in the state (×2 states); needs the technical committee's
definition of "meaningful" interleaving. Deadline: elapsed time in the state
(×420/Δ). Both need #4's time model.

## 8. Exact failure MDP

Needs #6 and per-skill success probabilities / durations fitted from robot
logs. Gives the upper bound for Table "gap to optimal".

## 9. GPSR-Sim

Batches from the official command generator, several layouts, stochastic
skills, success judged on the final world state. A project of its own.

## 10. PDDL and LLM-merge baselines

PDDL needs a domain file; LLM merge needs a fixed model and prompt.

## 11. Non-blocking leaves

Robot side only; the sim doesn't need it.

## 12. Runtime table

The benchmark used for `improvements.md` already produces the numbers; it
should become a committed script so the table can be regenerated.

## 13. Per-command result registers in the sim

`sim/handlers.py:379`: when `previous_command_info` is empty,
`say_with_context` falls back to `self.last_results[-2:]`, a buffer shared by
**all** commands. In an interleaved plan those can be another command's
results, which is the "answer drawn from another command" error from the
paper's introduction. Give each command its own results list. **Check whether
home2's `say_with_context` has the same fallback.** If it does, it's a real bug
on the robot, and a measured motivating example for the paper.
