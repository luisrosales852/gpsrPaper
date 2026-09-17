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
sim/                  (empty) simulation handlers go here
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

## Still to write

- **Port the two pure-Python tests.** In home2, `task_manager/scripts/test/test_hri_manager.py`
  lines 828-889 (`test_merger`) and 890-966 (`test_fallback_resume`) are ROS-free in
  substance — they are just methods on an rclpy Node using `self.get_logger()`.
  Swap the logger for asserts and they become pytest functions.
  `test_merged_plan_speech` (967-1026) does **not** port: it calls
  `GPSRTM._spoken_merged_plan` on the real node.
- **Mock handlers in `sim/`.** `test_fallback_resume` contains a `RecordingHandler`
  stub that is the natural starting point. A sim handler is any object with methods
  named after the action kinds (`go_to`, `pick_object`, `place_object`, ...) returning
  `(Status.EXECUTION_SUCCESS, result)`. The `locator` is just `lambda name: (x, y)`,
  so a simulated apartment is a dict of coordinates.

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
