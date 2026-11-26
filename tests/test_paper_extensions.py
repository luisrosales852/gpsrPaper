import json
from functools import partial
from pathlib import Path
from types import SimpleNamespace

import pytest

from gpsr_merger.merger import merge
from gpsr_merger.recovery import RecoveryPolicy
from sim.benchmark import parse_utterance, score
from sim.recovery_trials import INFORMATION, Trial, _run, fetch, go, say
from sim.run_sim import HERE, run_once
from sim.skill_model import SkillModel, fit_skill_model
from sim.stochastic import compare
from sim.world import World
from tests._merger_helpers import build_commands, make_locator


@pytest.mark.parametrize("dependencies,pose", [(True, "sitting"), (False, "standing")])
def test_person_attention_in_execution(tmp_path, monkeypatch, dependencies, pose):
    scenario = {
        "name": "two-person-attention", "origin": World().locator("couch"),
        "commands": [
            [go("couch"), {"action": "find_person_by_name", "name": "Angel"},
             go("side_table"), {"action": "get_person_info", "info_type": "pose"}],
            [go("living_room"), {"action": "find_person_by_name", "name": "Tom"}],
        ],
    }
    monkeypatch.setattr("sim.run_sim.merge", partial(merge, dependencies=dependencies))
    args = SimpleNamespace(apartment=Path(HERE)/"apartment.json", width=832, height=720,
                           fps=30, speed=3, camera="topdown", retries=2, budget=420,
                           no_video=True, print_tree=False, fail=[])
    run_once(scenario, "merged", str(tmp_path), args)
    log = json.loads((tmp_path / "run.json").read_text())
    skills = [e for e in log["events"] if e["phase"] == "end" and e["kind"] != "go_to"]
    assert all(e["status"] == "EXECUTION_SUCCESS" for e in skills)
    expected = ["Angel", pose, "Tom"] if dependencies else ["Angel", "Tom", pose]
    assert [e["result"] for e in skills] == expected


def test_exact_cost_precedes_tie_preference():
    commands = build_commands([[go("A")], [go("B")]])
    locator = make_locator({"A": (1.0000005, 0), "B": (0, 1)})
    assert [pa.location for pa in merge(commands, locator=locator, origin=(0, 0)).actions] == ["B", "A"]


def test_stationary_exposure_selects_schedule():
    commands = build_commands([fetch("book", "A", "B"), [go("C"), {"action": "answer_question"}]])
    kw = dict(locator=make_locator({"A": (0, 0), "B": (10, 0), "C": (1, 0)}), origin=(0, 0),
              speed_m_s=0.5, skill_durations_s={"answer_question": 40.0})
    # A two-metre detour before picking avoids 40 seconds of occupied dwell.
    def work(weight):
        return [pa.action.action for pa in merge(commands, holding_penalty=weight, **kw).actions
                if pa.action.action != "go_to"]
    assert work(0) == ["pick_object", "answer_question", "place_object"]
    assert work(1) == ["answer_question", "pick_object", "place_object"]


@pytest.mark.parametrize("origin,reverse", [("start_location", True), ("dishwasher", False)])
def test_book_at_dishwasher_report_in_both_orders(tmp_path, origin, reverse):
    world = World()
    query = [go("dishwasher"), {"action": "count", "target_to_count": "books"},
             go(origin), say("books")]
    move = fetch("book", "bedroom", "counter")
    batch = [query, move] if reverse else [move, query]
    batch.append([go("garage"), {"action": "answer_question"}])
    for mode in ("merged", "sequential"):
        args = SimpleNamespace(apartment=Path(HERE)/"apartment.json", width=832, height=720,
                               fps=30, speed=3, camera="topdown", retries=2, budget=420,
                               no_video=True, print_tree=False, fail=[])
        dst = tmp_path / mode
        run_once({"name": "book-count", "origin": world.locator(origin), "commands": batch}, mode, str(dst), args)
        events = json.loads((dst/"run.json").read_text())["events"]
        assert [e["result"] for e in events if e["phase"] == "end" and e["kind"] == "say_with_context"
                ] == ["About books: 0 books"]


def test_replan_drops_unreachable_observation_and_preserves_other_goals(tmp_path):
    trial = Trial("failed-route", INFORMATION, ("0.0:3",))
    log, metrics = _run(trial, RecoveryPolicy.SKIP_ONLY, tmp_path, replan=True)
    assert metrics["completed_commands"] == [1, 2]
    assert metrics["returned"]
    assert len(log["summary"]["replans"]) == 1
    assert [e["result"] for e in log["events"] if e["kind"] == "pick_object"
            and e["phase"] == "end"] == ["book"]
    assert len([e for e in log["events"] if (e["phase"], e["cmd"], e["idx"]) == ("end", 0, 0)]) == 2


def test_empirical_model_pairs_attempts_across_orders():
    records = [dict(skill="count", success=True, duration_s=4),
               dict(skill="count", success=False, duration_s=2)]
    model = fit_skill_model(records)
    a, b = (SkillModel(1234, skills=model) for _ in range(2))
    for _ in range(10):
        a.sample(1, 2, "pick_object")
        ok, t = a.sample(0, 1, "count")
        assert (ok, t) == b.sample(0, 1, "count")
        assert t == (4 if ok else 2)


def test_oracle_matches_enumerated_two_command_tree():
    # Each single-segment command is one stochastic pick. A successful pick
    # holds the gripper forever. A failed pick permits the other command.
    world = World()
    records = [{"actions": [go("start_location"), {"action": "pick_object"}]}] * 2
    for p in (0, 0.25, 1):
        result = compare(records, world, p)
        one = (1-p) * (3 - 120) + p
        expected = one + p * one
        assert result["exact_cost"] == pytest.approx(expected)
        assert result["replan_cost"] == pytest.approx(expected)


def test_wrong_answer_scores_as_a_missed_goal():
    utterance = "tell me how many cups there are on the bookshelf"
    assert parse_utterance(utterance)[0] == go("bookshelf")
    records = [{"goal": {"type": "report", "target": "cups", "site": "counter", "answer": "1 cups"}}]
    log = {"events": [dict(cmd=0, phase="end", kind="say_with_context", status="EXECUTION_SUCCESS",
                           result="About cups: 0 cups", x=0, y=0, idx=3, held=None)],
           "summary": dict(elapsed_s=1, actual_distance_m=0, holding_time_s=0, returned=True, replans=[])}
    log["events"][0]["purpose"] = "plan"
    assert score(records, log, World())["completed"] == []


def test_failed_pick_can_change_remaining_route(tmp_path):
    def question(site):
        return [go(site), {"action": "answer_question"}]

    commands = [fetch("book", "bedroom", "office"), question("entrance") + question("entrance"),
                question("bookshelf") + question("counter")]
    results = []
    for enabled in (False, True):
        args = SimpleNamespace(apartment=Path(HERE)/"apartment.json", width=832, height=720,
                               fps=30, speed=3, camera="topdown", retries=2, budget=420,
                               no_video=True, print_tree=False, fail=["0.1:1"],
                               recovery_policy=RecoveryPolicy.SKIP_ONLY, replan=enabled)
        dst = tmp_path / str(enabled)
        run_once(dict(name="changed-route", origin=(0, 0), commands=commands), "merged", str(dst), args)
        log = json.loads((dst/"run.json").read_text())
        order = [(e["cmd"], e["idx"]) for e in log["events"] if e["phase"] == "end"
                 and e["kind"] == "answer_question" and e["status"] == "EXECUTION_SUCCESS"]
        results.append((order, log["summary"]["actual_distance_m"]))
    assert sorted(results[0][0]) == sorted(results[1][0]) == [(1, 1), (1, 3), (2, 1), (2, 3)]
    assert results[0][0] != results[1][0]
    assert results[1][1] < results[0][1]


def test_residual_plan_preserves_verified_owner():
    from gpsr_merger.merger import replan_remaining

    commands = build_commands([fetch("cup", "A", "B"), fetch("book", "C", "D")])
    plan = merge(commands)
    pending = [plan.fallback[0], plan.fallback[1][2:]]
    remaining = replan_remaining(pending, initial_holder=1, origin=(0, 0),
                                 locator=make_locator({"A": (0, 0), "B": (1, 0), "D": (10, 0)}))
    assert [(a.source_cmd, a.source_idx) for a in remaining if a.requires_gripper or a.releases_gripper
            ] == [(1, 3), (0, 1), (0, 3)]


def test_failed_delivery_changes_the_ground_truth_for_a_later_report():
    world = World()
    records = [
        {"goal": {"type": "delivery", "object": "cup", "goal": "bookshelf"}},
        {"goal": {"type": "report", "site": "bookshelf", "target": "cups", "answer": "1 cups"}},
    ]
    log = {"events": [
        dict(cmd=0, idx=1, phase="end", kind="pick_object", status="EXECUTION_ERROR",
             held=None, site_changes={}, purpose="plan"),
        dict(cmd=1, idx=1, phase="end", kind="count", status="EXECUTION_SUCCESS",
             result="0 cups", held=None, purpose="plan"),
        dict(cmd=1, idx=3, phase="end", kind="say_with_context", status="EXECUTION_SUCCESS",
             result="About cups: 0 cups", x=0, y=0, held=None, purpose="plan"),
    ], "summary": dict(elapsed_s=1, actual_distance_m=0, holding_time_s=0, returned=True,
                       replans=[], objects_xyz={"cup": world.find_object("cup").home})}
    rated = score(records, log, world)
    assert rated["completed"] == [1]
    assert rated["expected_answers"][1] == "0 cups"
