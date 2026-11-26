import json
from itertools import permutations
from math import ceil
from pathlib import Path

import pytest

from gpsr_merger.endgame import plan_endgame
from gpsr_merger.merger import NoInterleavingSchedule, merge
from sim.endgame import replay
from tests._merger_helpers import build_commands

FIXTURE = Path(__file__).resolve().parents[1] / "scenarios/endgame_25s.json"


@pytest.mark.parametrize("bin_s", [1, 5])
def test_endgame_matches_every_returning_prefix(bin_s):
    fixture = json.loads(FIXTURE.read_text())
    for budget in range(46):
        # Independent time-and-goal oracle. Try stopping immediately and
        # every ordered subset of the two jobs. Each ends five seconds from home.
        choices = []
        for size in range(3):
            for order in permutations([0, 1], size):
                site = 0
                legs = []
                for cmd in order:
                    legs.append((0 if cmd == site else 4) + (22 if cmd == 0 else 5))
                    site = cmd
                legs.append(5)
                if sum(ceil(t / bin_s) * bin_s for t in legs) <= budget:
                    choices.append((-size, sum(legs), order))
        result = replay(fixture, budget, bin_s)
        assert result["return_feasible"] == bool(choices)
        if choices:
            _, seconds, order = min(choices)
            assert result["elapsed_s"] == seconds
            assert result["completed"] == tuple(sorted(order))
            assert result["segment_order"] == tuple((c, 0) for c in order)
            assert result["steps"][-1] == "return_to_instruction_point"


def test_endgame_respects_attention_and_gripper_before_credit():
    # A read would be cheap at the start, but must wait for the earlier find.
    # A second command cannot pick while command 3 holds an object.
    commands = build_commands([
        [{"action": "go_to", "location_to_go": "far"}, {"action": "find_person_by_name"}],
        [{"action": "get_person_info"}],
        [{"action": "pick_object"}],
        [{"action": "place_object"}],
    ])
    plan = plan_endgame(commands, start="here", home="here", remaining_s=1, initial_holder=3,
                        travel_s=lambda a, b: None,
                        skill_durations_s={"find_person_by_name": 1, "get_person_info": 1,
                                          "pick_object": 1, "place_object": 3})
    assert plan.completed == ()
    assert plan.actions == []


def test_endgame_charges_escort_and_returns_from_its_exit():
    commands = build_commands([[{"action": "go_to", "location_to_go": "A"},
                                {"action": "guide_person_to", "destination_room": "B"}]])
    legs = {("home", "A"): 2, ("A", "B"): 4, ("B", "home"): 7}
    kw = dict(start="home", home="home", travel_s=lambda a, b: legs.get((a, b)),
              skill_durations_s={"guide_person_to": 3})
    assert plan_endgame(commands, remaining_s=15, **kw).completed == ()
    exact = plan_endgame(commands, remaining_s=16, **kw)
    assert exact.completed == (0,)
    assert (exact.elapsed_s, exact.return_s) == (16, 7)


def test_bit_zero_can_reorder_whole_commands_and_bit_one_can_be_impossible():
    commands = build_commands([[{"action": "go_to", "location_to_go": x},
                                {"action": "answer_question"}] for x in ("far", "near")])
    options = dict(origin=(0, 0), locator={"far": (10, 0), "near": (1, 0)}.get)
    plan = merge(commands, interleaved=False, **options)
    assert [a.source_cmd for a in plan.actions if a.action.action == "answer_question"] == [1, 0]
    with pytest.raises(NoInterleavingSchedule):
        merge(commands, interleaved=True, **options)
    with pytest.raises(NoInterleavingSchedule):
        merge([], interleaved=True)
