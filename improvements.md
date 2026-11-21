# Improvements to port to the robot (home2)

Changes made in `gpsrPaper/gpsr_merger/merger.py` that are **not yet in
`RoBorregos/home2`** (`task_manager/task_manager/gpsr/merger.py`, branch
`qwenUpgrade`). Newest first.

---

## 5. Named start: drive back before headless actions — 2026-11-20

### What changed and why

Closes the limit in §4. `merge(..., origin_name="start_location")` names the
robot's start. When a segment of actions before a command's first `go_to` is
not scheduled first, the plan now contains a `go_to(origin_name)` right before
it, so `pick cup → go_to A → place → pick book` becomes
`… → place → go_to start → pick book` (`here_named_start_drives_back_when_forced_late`).

- **Insertion follows the schedule.** `_flatten` adds the `go_to` after the DP,
  and `_collapse_redundant_go_tos(start=origin_name)` drops it when the robot is
  already there (first segment, or right after a command's own `go_to(start)`).
  Travel-only, when the robot's pose is on the named start, the work order and
  the travel equal those without a name; every inserted `go_to` lands where the
  robot is not (`test_origin_name_only_adds_the_drive_back`, 300 batches).
- **Priced as driven.** The DP prices what the plan drives: returning is
  `distance(place, origin_name)` (the drive's own direction), leaving after a
  return is `distance(origin_name, ·)`, and leaving before the robot has moved
  is `distance(None, ·)`, from its own pose, since the inserted `go_to` is
  dropped then. "Not moved yet" = every finished segment is an at-start one
  without an escort, a function of the progress vector, so the DP stays exact
  (it then prices the next step with `seed`, like the first one). Checked against brute force over every feasible order,
  each scored by the travel its flattened plan drives, with the pose up to
  1.5 m off the named start and a 15% asymmetric metric
  (`test_named_start_is_priced_as_driven`): the old pricing (always
  `distance(None, ·)`) was suboptimal in 28/300 batches (0.77 m mean, 3.25 m
  max), the new one in 0/300. Nothing changes in the sim, where the pose is on
  `start_location`; on the robot the TF pose never is, and Nav2 path lengths
  are not symmetric.
- **`origin=None` + a name** makes the origin known: `distance(origin_name, ·)`
  replaces `distance(None, ·)`, so the plan equals the known-origin plan instead
  of pushing a return to `LARGE_M` (`here_named_start_prices_unknown_origin`).
- **Unreachable name = no name.** If `distance` cannot price a drive to
  `origin_name` (`distance(None, origin_name)` with a known pose, or from it to
  any place in the batch without one), `merge()` ignores it (`_reachable`).
  Otherwise one typo or a blocked start would put a failing `go_to` in the plan,
  fail the interleaved branch after its retries, drop the batch to the
  sequential fallback (which repeats the failing `go_to`), price every trip to
  the start at `LARGE_M`, and key forced-late headless actions at a place the
  robot never reaches. Pinned by `here_named_start_unknown_name_falls_back`,
  `here_named_start_unreachable_falls_back`,
  `here_named_start_unknown_name_origin_none` and
  `test_unreachable_origin_name_is_ignored` (300 batches: plan and fallback
  identical to no name). Silent on purpose: home2 can log by checking the name
  with `_resolve_xy` before calling `merge()`.
- **Fallback.** Each headless command's fallback starts with the inserted
  `go_to`, since the fallback runs after a failure, away from the start.
  `sim.run_sim.sequential_plan` drops it for the very first command only.
- **Marker.** Inserted actions have `source_idx == -1` and a
  `SimpleNamespace(action="go_to", location_to_go=origin_name)` action. At most
  one per command, so `(source_cmd, source_idx)` stays unique.
- **Dependencies act at the start.** With `origin_name`, actions before a
  command's first `go_to` read/write state at that name instead of at the unknown
  place `None`, which overlaps every place and made them conflict with almost
  everything. This only removes constraints, so travel never grows. On 300
  random batches it shortens 34 plans by 7.88 m on average, and the inserted
  drive-backs drop from 39 to 5: most "forced late" headless segments were
  spurious conflicts (`dep_here_unknown_place_conflicts_with_everything` vs
  `dep_here_named_start_drops_spurious_conflict`). A real conflict at the start
  still holds (`here_named_start_no_double_go_to`). Without `origin_name`
  nothing changes.

Sim showcases are unchanged (their headless segment already runs first).

### How to port it to home2

File copy, then pass `origin_name=<instruction-point name>` from
`GPSRTM._execute_interleaved_batch`. Check: (1) `_path_cost` / `_resolve_xy`
resolve that name; (2) the `go_to` handler accepts a `SimpleNamespace` with
`action`/`location_to_go` (it reads them by attribute); (3) `_completed` keys
`(cmd, -1)` are harmless.

### Verifying on the robot

Hardcode `here_named_start_drives_back_when_forced_late` with real places: the
robot must drive back to the instruction point before the second pick.

---

## 4. Commands with no leading `go_to` run at the origin — 2026-11-16

### What changed and why

"Tell me a joke, then go to the kitchen and count the cups" parses to a command
whose first action is not a `go_to`. That action happens where the robot stands
when the batch is given. The merger used to give that segment `location=None`,
i.e. an unknown place costing `LARGE_M` (1000 m) in and out, so its position in
the plan was arbitrary.

Now `_decompose` marks it `at_origin`, and `_cost_matrix` prices it:

| hop | origin known | `origin=None` |
|---|---|---|
| start → it | 0 | 0 |
| it → next place | `distance(None, place)` | 0 (same as the no-origin seed) |
| a place → it | `distance(None, place)` | `LARGE_M` |

A `go_to` whose name does not resolve is still unknown (`LARGE_M`).

**Limit (fixed in §5 with `origin_name`).** This is pricing only. The plan has no action
that drives the robot back to the origin, because the parser gave none and the
origin has no name. The DP runs these segments first when it can. It cannot when
the gripper or a dependency forces them later
(`here_forced_late_by_gripper`), and it may choose not to when the segment ends
with an escort (`here_escort_can_run_late_without_drive_back`). In both cases
the action runs wherever the robot is, as it already does in sequential order.
Fixing it needs a named start location the merger could insert a `go_to` for.

### How to port it to home2

File copy; `merge()`'s signature is unchanged. Nothing to wire.

### Verifying on the robot

Hardcode a batch whose second command starts with `answer_question` and whose
first command starts with a far `go_to`. The answer must be given before the
robot drives anywhere.

---

## 3. Exit locations for guide/follow — 2026-11-16

### What changed and why

"Go to the couch, guide Jane to the kitchen" leaves the robot in the kitchen.
The merger priced the next hop from the couch. Now a segment containing
`guide_person_to` (`destination_room`) or `follow_person_until` (`destination`)
has an `exit`, and `_cost_matrix` uses `edge = distance(exit(previous),
entry(next))`. The DP is untouched: the robot's position is still a function of
the last segment, so it stays exact (checked against the Held-Karp oracle on
400 random batches that now include escorts and headless commands).

- A follow with **no destination** keeps `exit = entry`. Its end point is
  unknown, and pricing it as `LARGE_M` would push every such follow to the end.
- **Collapse bug fixed.** `_collapse_redundant_go_tos` only tracked `go_to`
  targets, so `go_to A, guide to B, go_to A` dropped the second `go_to A` and
  the robot did A's work in B. An escort now resets the tracked location, so the
  next `go_to` is always kept (also right after an escort to that same place:
  a follow stops behind the person, not on the spot). Pinned by
  `exit_keeps_go_to_after_escort`; `shared_start_first_command_goto_protected`
  had pinned the bug (count at `chairs` executed in the bathroom) and now
  expects two visits.

### Measured

300 random GPSR-shaped batches (`tests/test_dependencies.py` generator), same
gripper and dependency rules, travel measured with exits:

| | old pricing | exits + origin |
|---|---|---|
| mean travel saved vs sequential | 7.44 m | 8.77 m |
| batches where the merged plan is *worse* than sequential | 19 | 0 |

The plan changes in 107 of the 223 batches that contain an escort or a
headless command, saving 3.73 m on average (max 16.3 m) in those.

Simulation (collision-free path length, `--no-video`):

| scenario | merged before | merged now | sequential |
|---|---|---|---|
| `showcase_all_leaves` | 38.24 m / 136.3 s | 35.49 m / 132.6 s | 41.38 m / 142.7 s |
| `showcase_merge` (no escort) | 35.38 m / 119.1 s | unchanged | 58.37 m / 161.6 s |
| `showcase_exits` (new) | order priced the old way: 37.3 m planned | 37.52 m / 113.8 s (19.9 m planned) | 39.85 m / 117.1 s (21.6 m planned) |

`cost_model_gap_m` is now the in-segment displacement (the escort itself and
the pick stand-off) to within one `FOLLOW_GAP`
(`test_exit_locations_are_priced`).

### How to port it to home2

File copy; `merge()`'s signature is unchanged. The destination names go through
the same `locator` / `distance` as `go_to` targets, so `_resolve_xy` must
resolve room names such as "kitchen" (it already does for `go_to`).

### Verifying on the robot

Hardcode `exit_guide_prices_next_hop_from_destination` with real locations: a
guide from A to a far room B, one command near B and one near A. The plan must
do the command near A first, then the guide, then the command near B.

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
174 contain a cross-command conflict, and **travel-only planning breaks conflict
order in 112 of them**. Enforcing dependencies costs **2.25 m** extra travel on
average (numbers re-measured 2026-11-16 with exit locations and headless
commands in the generator; before that: 185 / 111 / 1.24 m). Both sim showcases have no shared places, so their plans are unchanged
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
   python3 run_scenarios.py               # 57/57, includes the 10 dep_* scenarios
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
