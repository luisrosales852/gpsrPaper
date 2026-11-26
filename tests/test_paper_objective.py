"""Black-box check of the additive objective in 8e77cd4:main.tex, §III–IV."""

from itertools import groupby, permutations
from math import dist

import pytest

from gpsr_merger.merger import merge
from tests._merger_helpers import build_commands


# Each row is an atomic segment. Commands 0 and 2 deliver objects to B.
# Command 1 counts at A after the first pick, then escorts a person from C to D.
SEGMENTS = (
    ("A", ["pick_object"]),
    ("B", ["place_object"]),
    ("A", ["count", "say_with_context"]),
    ("C", ["guide_person_to"]),
    ("C", ["pick_object"]),
    ("B", ["place_object"]),
)
DURATION = {"pick_object": 3, "place_object": 5, "count": 8, "say_with_context": 7,
            "guide_person_to": 2}  # stationary seconds, exclusive of escort travel
SPEED = 0.5  # m/s


def paper_cost(order, points, kappa, weight):
    """Replay the stated segment effects and price the resulting schedule in seconds."""
    # Command prefixes, then the two cross-command read/write conflicts at A and B.
    for first, second in ((0, 1), (2, 3), (4, 5), (0, 2), (1, 5)):
        if order.index(first) > order.index(second):
            return None
    pose = points["start"]
    holder = None
    elapsed = exposure = 0.0
    for segment in order:
        site, skills = SEGMENTS[segment]
        hop = dist(pose, points[site]) / SPEED
        elapsed += hop + (kappa if hop else 0)
        if holder is not None:
            exposure += hop
        pose = points[site]
        for skill in skills:
            seconds = DURATION[skill]
            if skill == "guide_person_to":
                seconds += dist(pose, points["D"]) / SPEED
                pose = points["D"]
            elapsed += seconds
            if holder is not None:
                exposure += seconds
            if skill == "pick_object":
                if holder is not None:
                    return None
                holder = segment // 2
            elif skill == "place_object":
                if holder != segment // 2:
                    return None
                holder = None
    return elapsed + weight * exposure


def emitted_order(plan, commands, segment_for):
    """Recover the order from skills, then verify every drive, skill and source index."""
    labels = [segment_for[step.source_cmd, step.source_idx] for step in plan.actions
              if step.action.action != "go_to"]
    order = tuple(segment for segment, _ in groupby(labels))
    assert sorted(order) == list(range(len(SEGMENTS)))

    expected = []
    pose = "start"
    for segment in order:
        for (cmd, idx), label in segment_for.items():
            if label != segment:
                continue
            action = commands[cmd].commands[idx]
            if action.action == "go_to":
                if pose == action.location_to_go:
                    continue
                pose = action.location_to_go
            elif action.action == "guide_person_to":
                pose = action.destination_room
            expected.append((cmd, idx, action))
    assert [(s.source_cmd, s.source_idx, s.action) for s in plan.actions] == expected
    return order


@pytest.mark.parametrize("coords", [
    [(0, 0), (4, 0), (9, 0), (6, 0), (2, 0)],
    [(0, 0), (8, 1), (2, 3), (1, 0), (8, 2)],
])
@pytest.mark.parametrize("kappa,weight", [(0, 0), (4, 0), (0, 1), (4, 1)])
@pytest.mark.parametrize("interleaved", [None, False, True])
def test_public_merge_minimizes_paper_objective(coords, kappa, weight, interleaved):
    points = dict(zip(("start", "A", "B", "C", "D"), coords))
    raw = [[], [], []]
    segment_for = {}
    for segment, (site, skills) in enumerate(SEGMENTS):
        cmd = segment // 2
        command = raw[cmd]
        start = len(command)
        command.append({"action": "go_to", "location_to_go": site})
        for skill in skills:
            action = {"action": skill}
            if skill == "guide_person_to":
                action["destination_room"] = "D"
            command.append(action)
        for idx in range(start, len(command)):
            segment_for[cmd, idx] = segment
    commands = build_commands(raw)
    plan = merge(commands, locator=points.get, origin=points["start"],
                 navigation_overhead=SPEED * kappa, holding_penalty=weight,
                 skill_durations_s=DURATION, speed_m_s=SPEED, interleaved=interleaved)
    costs = {}
    for order in permutations(range(len(SEGMENTS))):
        # A split command appears in more than one contiguous run. This
        # classifies the completed order independently of the DP transition.
        runs = [c for c, _ in groupby(segment // 2 for segment in order)]
        if interleaved is not None and (len(runs) > len(set(runs))) != interleaved:
            continue
        cost = paper_cost(order, points, kappa, weight)
        if cost is not None:
            costs[order] = cost
    actual = emitted_order(plan, commands, segment_for)
    assert actual in costs, f"infeasible schedule {actual}"
    assert costs[actual] == pytest.approx(min(costs.values()))


def test_later_find_preserves_earlier_commands_person_attention():
    # Tom lies on the shortest route to Angel's observation site. In the
    # original skill table, finding Tom writes the attention read by command 0.
    commands = build_commands([
        [{"action": "go_to", "location_to_go": "couch"},
         {"action": "find_person_by_name", "name": "Angel"},
         {"action": "go_to", "location_to_go": "side_table"},
         {"action": "get_person_info", "info_type": "pose"}],
        [{"action": "go_to", "location_to_go": "living_room"},
         {"action": "find_person_by_name", "name": "Tom"}],
    ])
    sites = {"couch": (0, 0), "living_room": (1, 0), "side_table": (2, 0)}
    plan = merge(commands, origin=sites["couch"], locator=sites.get)
    skills = [(s.source_cmd, s.action.action) for s in plan.actions if s.action.action != "go_to"]
    assert skills == [(0, "find_person_by_name"), (0, "get_person_info"), (1, "find_person_by_name")]


@pytest.mark.parametrize("first,second,conflict", [
    ({"action": "find_person"}, {"action": "find_person_by_name"}, True),
    ({"action": "find_person_by_name"}, {"action": "follow_person_until", "destination": "B"}, True),
    ({"action": "guide_person_to", "destination_room": "A"}, {"action": "find_person"}, True),
    ({"action": "get_person_info"}, {"action": "get_person_info"}, False),
    ({"action": "find_person"}, {"action": "count"}, False),
])
def test_person_attention_across_locations(first, second, conflict):
    # Without a conflict the robot should start with command 1, at its own pose.
    commands = build_commands([
        [{"action": "go_to", "location_to_go": "A"}, first],
        [{"action": "go_to", "location_to_go": "B"}, second],
    ])
    sites = {"A": (5, 0), "B": (0, 0)}
    plan = merge(commands, origin=sites["B"], locator=sites.get)
    owners = [s.source_cmd for s in plan.actions if s.action.action != "go_to"]
    assert owners == ([0, 1] if conflict else [1, 0])
