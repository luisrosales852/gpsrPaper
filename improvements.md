# Improvements to port to the robot (home2)

Changes made in `gpsrPaper/gpsr_merger/merger.py` that are **not yet in
`RoBorregos/home2`** (`task_manager/task_manager/gpsr/merger.py`, branch
`qwenUpgrade`). Newest first.

---

## 1. Progress-vector DP replaces the subset (Held-Karp) DP — 2026-11-11

### What changed and why

The merger splits each command into go_to-led **segments** and searches for the
cheapest interleaving. The old solver was Held-Karp: state = `(S, last)`, where
`S` is a bitmask of which segments are done, so the table has 2^M entries
(M = total segments). Segments of one command must run in order, so the only
reachable `S` are "the first k_c segments of each command". That means the
state is really just

```
k = (k_0, ..., k_{n-1})     k_c = how many segments of command c are done
```

plus **which command ran last** (that fixes where the robot is standing). The
old code already computed k_c internally (`(S & cmd_seg_mask[c]).bit_count()`
in `_gripper_holder` and in the prefix check). It just stored the result in a
much larger table.

The new solver runs the DP directly on `(k, last_cmd)`:

- **Same optimal plan.** Same feasible schedules, same costs, same epsilon
  tie-break. Verified against the old solver on all scenarios and on 400 random
  batches: equal cost every time, identical schedule in 307/400. The other 93
  are exact ties, equal to within 2e-13 even with the tie-break term counted.
- **No more silent cliff at 16 segments.** The old `_M_CAP = 16` made any
  batch with more than 16 segments fall back to sequential order, with zero
  savings. The new cap is on the state count `n · ∏(L_c+1)` (`_STATE_CAP =
  200_000`). That bound is never larger than 2^M, and it only triggers for
  batches of many one- or two-segment commands.
- **The paper's claim is now true.** `main.tex` (contributions, §Method,
  Alg. 1, runtime table) describes this DP.
- **It's the right state for the next features.** Replanning after a failure,
  the interleaving-bonus bit, the time limit and the failure MDP all add fields
  to this small state. Adding them to a 2^M state was not practical.

### Measured (MacBook, CPython 3, Euclidean distances, random pick/place)

| n cmds | segs/cmd | M | progress states | subset states 2^M·M | progress DP | old Held-Karp |
|---|---|---|---|---|---|---|
| 3 | 2 | 6  | 81     | 384         | 0.05 ms | <1 ms |
| 3 | 3 | 9  | 192    | 4 608       | 0.09 ms | <1 ms |
| 3 | 4 | 12 | 375    | 49 152      | 0.26 ms | 4 ms |
| 4 | 4 | 16 | 2 500  | 1 048 576   | 1.7 ms  | 56 ms |
| 3 | 6 | 18 | 1 029  | 4.7 M       | 0.7 ms  | fell back to sequential |
| 4 | 5 | 20 | 5 184  | 21 M        | 2.7 ms  | fell back to sequential |
| 6 | 4 | 24 | 93 750 | 403 M       | 42 ms   | fell back to sequential |

For a real GPSR batch (3 commands) both solvers take milliseconds, so the robot
won't notice a speed difference. What it gains is no cap and a state the
planned features can build on.

Simulation `showcase_merge` is unchanged: merged 35.38 m / 119.1 s vs sequential
58.37 m / 161.6 s (39.4% less driving, 26.3% less time).

### How to port it to home2

`gpsr_merger/merger.py` imports only `dataclasses` and `typing`, so you can
**copy the whole file over** home2's:

```bash
cp gpsrPaper/gpsr_merger/merger.py \
   home2/task_manager/task_manager/gpsr/merger.py
```

No import rewrites are needed, and `merge()`'s signature is backward compatible.
The robot's call in `GPSRTM._execute_interleaved_batch`
(`merge(self.batched_commands, locator=..., origin=...)`) works unchanged.

Copying the file also brings in two earlier changes that home2 does not have
yet (see "Also pending" below). If you want *only* the DP change, the edits
are:

1. Replace `_M_CAP = 16` with `_STATE_CAP = 200_000`.
2. Replace `_gripper_holder(S, n_cmds, cmd_seg_mask, holding_after)` with
   `_gripper_holder(k, holding_after)`: first `c` with `holding_after[c][k[c]]`.
3. Add `_state_count(seg_lens) = n · ∏(L_c+1)`.
4. Replace `_optimal_schedule` with the layered DP over `(k, last_cmd)`.
   - start state is `((0,…,0), -1)`;
   - from `(k, last)` try each command `c` with `k[c] < L[c]` in ascending order;
   - next segment `u = seg_ids[c][k[c]]`, skip if another command holds the
     gripper and `u` is not gripper-neutral;
   - step cost `seed[u] + ε·c` from the start state, otherwise
     `edge[here][u] + ε·(c ≠ last)` with `here = seg_ids[last][k[last]-1]`;
   - keep strict `<` so ties resolve the same way every run;
   - process states in layers of equal `sum(k)`, then walk the parent pointers
     back from the best `(L, c)`.
5. `_sequential_schedule(segments)`: stable sort of flat indices by `segments[u].cmd`.
6. In `merge()`: drop `seg_cmd`/`seg_pos`/`cmd_seg_mask`, build
   `seg_ids[c] = list of flat indices`, and gate on
   `_state_count(...) > _STATE_CAP` instead of `M > _M_CAP`.

### Verifying on the robot

1. **Before deploying**, in gpsrPaper:
   ```bash
   python3 run_scenarios.py               # 30/30
   python3 -m tests.test_progress_dp      # progress DP == Held-Karp oracle
   ```
2. **Same plan as before.** Use `TEST MODE` with hardcoded commands in
   `gpsr_task_manager.py` and run the same batch with the old and new merger.
   The line `Merged N commands → K interleaved actions` and the spoken merged
   plan should match. They can only differ when two orderings have exactly
   equal cost.
3. **Deterministic.** Run the same batch several times with the robot at the
   same start pose. The plan must be identical every time. The DP has no
   randomness: it uses ordered dicts, tries commands in ascending order and
   keeps strict `<`. Distances come from map coordinates, so the only thing
   that can change the plan is `origin`, the robot's current pose.
4. *(Optional)* Time the call. Wrap `merge(...)` in
   `time.perf_counter()` and log the milliseconds. Expect well under 10 ms for a
   3-command batch on the robot PC.

---

## Also pending (made earlier, also not in home2)

- **`_Segment.ends_holding` gripper fix.** A segment's gripper state now comes
  from its *last* gripper action. Upstream, a segment doing `place X, pick Y`
  was treated as ending empty, which let another command's pick interleave so
  the robot would hold two items. Pinned by scenario
  `place_then_pick_in_one_segment_still_holds`.
- **`distance=` hook on `merge()`.** The planner can rank orderings by real
  path length instead of straight-line distance. The robot code in
  `reference/gpsr_task_manager.py` does **not** pass it yet: there is no
  `_path_cost` there, and it calls `merge(..., locator, origin)` only. Wiring it
  needs a `distance(from_name | None, to_name) -> metres | None` built from a
  Nav2 path-length query, computed once per batch before calling `merge()`.
  Until then the robot plans with Euclidean distance, which can pick a worse
  order when a wall separates two nearby locations (scenario
  `custom_distance_wall_reorders`).
