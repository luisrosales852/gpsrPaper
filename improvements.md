# Improvements to port to the robot (home2)

Changes made in `gpsrPaper/gpsr_merger/merger.py` that are **not yet in
`RoBorregos/home2`** (`task_manager/task_manager/gpsr/merger.py`, branch
`qwenUpgrade`). Newest first.

---

## 2. Cross-command dependencies — 2026-11-12

### What changed and why

Before this change the merger reordered segments of different commands freely.
Example: A is issued first, "find Jane in the living room, take a cup from the
kitchen table and give it to her", and B is "count the cups on the kitchen
table and tell me". Travel-only planning lets B count while A is still on its
way to the table, so B reports one cup more than if the commands had run one
by one. (Scenario `dep_count_waits_without_dependencies_is_violated`.)

Now every action has a **read set and a write set** over shared state. Two
segments of different commands **conflict** if one writes something the other
reads or writes. When they do, the **command that was issued first goes first**.
Every other pair can still be reordered freely for travel.

Because every conflict keeps the sequential order, the merged plan is
**conflict-equivalent to running the commands one by one** (conflict
serializability, as in databases). Every answer and every world effect is the
same as in the sequential baseline; only the travel changes.

**State variables** are `(domain, place)`, with domain `objects` or `people`.
They are keyed by **place, not object/person name**, because the parser doesn't
name things consistently across commands ("cup" / "cups" / "drinks", "Jane" /
"the person in red"), while every action has a location. That can order two
actions that didn't strictly need it (only costs travel), but it never misses a
real conflict (which would cost a wrong answer).

| action | reads (at current place) | writes |
|---|---|---|
| `pick_object` | objects | objects |
| `place_object` | — | objects |
| `give_object` | people | — |
| `count` | objects, people | — |
| `get_visual_info` | objects | — |
| `find_person`, `find_person_by_name`, `get_person_info` | people | — |
| `guide_person_to`, `follow_person_until` | people | people here **and** at the destination |
| `go_to`, `answer_question`, `say_with_context` | — | — |

**Rooms.** The official GPSR generator mixes room-level references ("count the
people *in the living room*", "find a cup *in the kitchen*") with placement and
beacon references ("guide Jane *from the couch*", "count the drinks *on the
kitchen table*"). Two places overlap if they are the same, or if one contains
the other. Two different spots in the same room (couch vs. armchair) do not
overlap. `merge()` learns containment through a new optional hook,
`room_of(name) -> room`. Without it, only identical names conflict, and
"guide from the couch" vs. "count people in the living room" is **missed**
(pinned by `dep_guide_vs_count_without_rooms_is_missed`). Other rules: names
are normalised (case, `_` → space), and a follow with no destination writes
"people anywhere", which conflicts with every people read.

**In the progress-vector DP** this is one extra eligibility check. `needs[c][j]`
lists `(d, m)`: segment j of command c may start only once an earlier command d
has done m segments. Like the gripper rule, it depends only on `k`, so the DP
stays exact. Constraints only point at earlier-issued commands, so sequential
order always satisfies them, and the DP can never get stuck because of them.

**Measured** on 300 random GPSR-shaped batches (`tests/test_dependencies.py`):
185 contain a cross-command conflict, and **travel-only planning breaks conflict
order in 111 of them**. Enforcing dependencies costs **1.24 m** extra travel on
average. Both sim showcases have no shared places, so their plans are unchanged
(35.38 m vs 58.37 m).

### API

```python
merge(commands, locator=None, origin=None, distance=None,
      dependencies=True,   # False = travel-only (ablation / old behaviour)
      room_of=None)        # name -> containing room, or None
```

Backward compatible. **Note: `dependencies=True` is the default, so the robot's
plans change once this is ported**, but only for batches with a real conflict.

### How to port it to home2

1. Copy `gpsr_merger/merger.py` over `task_manager/task_manager/gpsr/merger.py`.
   It is still stdlib-only.
2. **Pass `room_of`** in `GPSRTM._execute_interleaved_batch`. The robot already
   has the room: `query_location` returns `hit.area` (the room) and
   `hit.subarea` (the spot), which is what `_resolve_xy` uses. Add a memoized
   helper next to `_resolve_xy`:

   ```python
   def _resolve_room(self, location_name: str):
       """Room (areas.json top-level key) containing a location, or None."""
       if not location_name:
           return None
       try:
           hits = self.subtask_manager.hri.query_location(location_name, top_k=1)
           return hits[0].area if hits else None
       except Exception as e:  # noqa: BLE001
           self.get_logger().warning(f"Room resolve failed for '{location_name}': {e}")
           return None
   ```

   and call `merge(..., room_of=self._resolve_room)`. It is asked once per
   distinct place in the batch, before the DP. If `query_location` already sits
   behind `_location_cache`, cache the area there too, to avoid a second query.
3. Nothing else changes. `build_tree`, the leaves and the fallback are untouched.

### Verifying on the robot

1. Before deploying, in gpsrPaper:
   ```bash
   python3 run_scenarios.py               # 40/40, includes the 10 dep_* scenarios
   python3 -m tests.test_dependencies     # invariants + oracle under dependencies
   python3 -m tests.test_progress_dp
   ```
2. **Conflict case on the robot.** In TEST MODE, hardcode the batch from
   `dep_count_waits_for_earlier_pick_same_placement`, using real arena
   locations: an earlier command takes an object from a table, and a later one
   counts objects on that same table. Check that the logged merged plan has the
   count *after* the pick, and that the number the robot says matches what is
   on the table after the pick.
3. **Room case.** Hardcode "guide <name> from <beacon in room R> to <other
   room>" followed by "count the people in R". The count must come after the
   guide. If it doesn't, `room_of` isn't wired, or `query_location` doesn't
   return the beacon's room as `area`.
4. **No-conflict batches unchanged.** Any batch with no shared places must give
   the same plan as before the port (compare against `dependencies=False`).

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
