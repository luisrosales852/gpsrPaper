"""Failure-specific execution regressions and the paired paper trials."""

import json

from gpsr_merger.bt_builder import build_tree
from gpsr_merger.merger import merge
from gpsr_merger.recovery import ExecutionHooks, GripperState, RecoveryPolicy
from gpsr_merger.status import Status
from py_trees.common import Status as TreeStatus
from sim.recovery_trials import CUPS, INFORMATION, TOM, Trial, _run, fetch, go, run_trials
from tests._merger_helpers import build_commands


def _read_log(out, trial, policy):
    return json.loads((out / trial / policy.value / "run.json").read_text())


def _attempts(log, cmd, idx):
    return [e for e in log["events"]
            if (e["phase"], e["cmd"], e["idx"]) == ("end", cmd, idx)]


def test_paired_trials(tmp_path):
    rows = run_trials(tmp_path)
    scores = {(r["trial"], r["policy"]): r for r in rows}
    fallback, skip, defer = RecoveryPolicy
    for policy in RecoveryPolicy:
        assert scores["nominal", policy.value]["completed_goals"] == 3
        transient = _read_log(tmp_path, "transient-information", policy)
        assert [e["status"] for e in _attempts(transient, 0, 1)] == [
            "EXECUTION_ERROR", "EXECUTION_SUCCESS",
        ]
        assert not transient["summary"]["failures"]
        assert not transient["failure_reports"]
        assert scores["transient-information", policy.value]["completed_goals"] == 3
        assert scores["failed-release", policy.value]["partial_credit"] == 0.5
        release = _read_log(tmp_path, "failed-release", policy)
        assert release["summary"]["final_held"] == "book"
        assert not _attempts(release, 1, 1)

    for policy in (skip, defer):
        shared = _read_log(tmp_path, "failed-pick-shared-navigation", policy)
        assert len(_attempts(shared, 0, 1)) == 1
        assert not _attempts(shared, 0, 3)
        assert _attempts(shared, 0, 2)[0]["status"] == "EXECUTION_SUCCESS"
        assert _attempts(shared, 2, 1)[0]["status"] == "EXECUTION_SUCCESS"
        assert _attempts(shared, 1, 3)[0]["status"] == "EXECUTION_SUCCESS"

    recovered = _read_log(tmp_path, "deferred-target", defer)
    assert [e["purpose"] for e in _attempts(recovered, 1, 1)] == ["plan", "reacquire"]
    assert [e["branch"] for e in _attempts(recovered, 1, 2)] == [
        "interleaved", "interleaved", "deferred",
    ]
    assert len(_attempts(recovered, 1, 4)) == 1
    assert not recovered["failure_reports"]
    assert scores["deferred-target", defer.value]["goals_recovered_by_defer"] == 1
    assert scores["deferred-target", skip.value]["completed_goals"] == 2

    persistent = _read_log(tmp_path, "persistent-information", defer)
    assert len(_attempts(persistent, 1, 2)) == 3
    assert not _attempts(persistent, 1, 4)
    assert [(e["cmd"], e["idx"]) for e in persistent["failure_reports"]] == [(1, 2)]

    tight = _read_log(tmp_path, "insufficient-time", defer)
    assert tight["summary"]["failures"][0]["deferred"] == "insufficient_time"
    assert tight["summary"]["returned"]
    assert all(e["branch"] != "deferred" for e in tight["events"])
    timed_out = _read_log(tmp_path, "insufficient-time", fallback)
    assert not timed_out["summary"]["returned"]
    assert timed_out["summary"]["elapsed_s"] <= 420.0


def test_navigation_failure_uses_fallback_and_blocks_wrong_location(tmp_path):
    trial = Trial("navigation", INFORMATION, ("0.0:3",))
    log, score = _run(trial, RecoveryPolicy.SKIP_DEFER, tmp_path)
    assert [e["branch"] for e in _attempts(log, 0, 0)] == [
        "interleaved", "interleaved", "sequential_fallback",
    ]
    assert not _attempts(log, 0, 1)
    assert not _attempts(log, 0, 3)
    assert score["completed_goals"] == 2


def test_deferred_work_runs_in_failure_order(tmp_path):
    trial = Trial("two-failures", INFORMATION, ("0.1:2", "1.2:2"))
    log, score = _run(trial, RecoveryPolicy.SKIP_DEFER, tmp_path)
    failed = [(f["cmd"], f["idx"]) for f in log["summary"]["failures"]]
    recovered = [(e["cmd"], e["idx"]) for e in log["events"]
                 if e["phase"] == "end" and e["purpose"] == "deferred"]
    assert len(failed) == 2 and failed == recovered
    assert score["goals_recovered_by_defer"] == 2


def test_deferred_find_restores_dependent_observation(tmp_path):
    trial = Trial("failed-find", INFORMATION, ("1.1:2",))
    log, score = _run(trial, RecoveryPolicy.SKIP_DEFER, tmp_path)
    assert [e["branch"] for e in _attempts(log, 1, 2)] == ["deferred"]
    assert [e["branch"] for e in _attempts(log, 1, 4)] == ["deferred"]
    assert score["goals_recovered_by_defer"] == 1


def test_intervening_world_change_prevents_stale_revisit(tmp_path):
    commands = [CUPS, TOM, fetch("cup", "kitchen", "garage")]
    trial = Trial("changed-observation", commands, ("0.1:2",))
    log, score = _run(trial, RecoveryPolicy.SKIP_DEFER, tmp_path)
    assert log["summary"]["failures"][0]["deferred"] == "ineligible"
    assert len(_attempts(log, 0, 1)) == 2
    assert [(e["cmd"], e["idx"]) for e in log["failure_reports"]] == [(0, 1)]
    assert score["completed_goals"] == 2


def test_uncertain_pick_blocks_manipulation_but_keeps_shared_navigation_goal():
    class Handler:
        state = GripperState.EMPTY

        def go_to(self, action):
            calls.append(action.location_to_go)
            return Status.EXECUTION_SUCCESS

        def pick_object(self, action):
            calls.append(action.object_to_pick)
            self.state = GripperState.UNCERTAIN
            return Status.EXECUTION_ERROR

        def place_object(self, action):
            raise AssertionError("released an unverified object")

    calls = []
    handler = Handler()
    commands = [fetch("cup", "kitchen", "bookshelf"), [go("bookshelf")],
                fetch("book", "bedroom", "garage")]
    plan = merge(build_commands(commands))
    tree = build_tree(
        plan, [handler], recovery_policy=RecoveryPolicy.SKIP_ONLY,
        execution_hooks=ExecutionHooks(gripper_state=lambda: handler.state),
    )
    while tree.status not in (TreeStatus.SUCCESS, TreeStatus.FAILURE):
        tree.tick_once()
    assert calls == ["kitchen", "cup", "bookshelf"]
    assert (0, 3) in tree.skipped and (2, 1) in tree.skipped
