"""GPSR multi-command interleaving merger.

Single-item, drop-before-pick model. Takes a list of parsed BAML
``CommandListLLM`` objects (one per spoken utterance) and returns a single
``InterleavedPlan`` whose ``actions`` list executes the commands in an
order that:

  * preserves intra-command sequencing,
  * keeps the gripper invariant (one item at a time, drop before pick),
  * keeps conflicting segments of different commands in issue order,
  * minimizes travel between locations (2-D Euclidean by default, or any
    caller-supplied ``distance`` such as a planner's path length) using a
    precedence-constrained
    DP over per-command progress vectors at the *segment* granularity,
    allowing gripper-neutral segments of any command to interleave inside
    another command's atomic block,
  * dedups back-to-back same-location ``go_to`` actions in the final plan.

Pipeline:
  1. Decompose each command into go_to-led segments.
  2. DP over progress vectors k = (k_0, ..., k_{n-1}), k_c = how many of
     command c's segments are done, with two-part eligibility:
       - per-command prefix order (implicit in k),
       - gripper-state check (only neutral segments may interleave while
         another command is mid-block),
       - cross-command dependencies (see _build_needs).
  3. Walk the schedule back into a flat PlanAction list, then collapse
     consecutive same-location go_tos.

The merger is intentionally framework-free: it talks to BAML objects via
``getattr`` so unit tests can pass simple stand-ins.
"""

from dataclasses import dataclass
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Sequence, Tuple

# Action-kind classifications.
_GRIPPER_ACQUIRES = frozenset({"pick_object"})
_GRIPPER_RELEASES = frozenset({"give_object", "place_object"})
# Sentinel cost for any edge touching an unresolved (None) coordinate. Well
# above any realistic indoor navigation distance, so the DP avoids unknown
# locations unless the per-command prefix forces them into a slot.
LARGE_M = 1000.0

# Max DP states (n * prod(L_c + 1)) before falling back to sequential.
_STATE_CAP = 200_000

# Tiebreaker weight: tiny per-cross-command-switch penalty so equal-cost
# schedules prefer per-command sequential order. Must stay well below any
# real distance difference.
_TIEBREAK_EPSILON = 1e-6


@dataclass
class PlanAction:
    """An action with its provenance + gripper effect."""

    action: Any
    source_cmd: int
    source_idx: int
    location: Optional[str]
    requires_gripper: bool
    releases_gripper: bool


@dataclass
class InterleavedPlan:
    actions: List[PlanAction]
    fallback: List[List[PlanAction]]


Locator = Callable[[str], Optional[Tuple[float, float]]]
# (from_location or None for the robot origin, to_location) -> meters, or None
# if unknown. A non-finite return (unreachable) also costs LARGE_M.
DistanceFn = Callable[[Optional[str], str], Optional[float]]
# location name -> containing room ("couch" -> "living room"), or None.
RoomFn = Callable[[str], Optional[str]]
_Var = Tuple[str, Optional[str]]  # (domain, place)


@dataclass
class _Segment:
    cmd: int
    indices: List[int]  # action indices within the source command
    location: Optional[str]  # the navigation target (from a leading go_to)
    # Gripper state after the segment, set by its *last* gripper action
    # (True = held, False = empty); None if it has no gripper action,
    # i.e. the segment is gripper-neutral.
    ends_holding: Optional[bool] = None
    reads: FrozenSet[_Var] = frozenset()
    writes: FrozenSet[_Var] = frozenset()


def _kind(action: Any) -> str:
    return getattr(action, "action", "") or ""


def _decompose(cmd_idx: int, command: Any) -> List[_Segment]:
    """Split a command's action list into go_to-led segments."""
    actions = list(getattr(command, "commands", []))
    segments: List[_Segment] = []
    cur_loc: Optional[str] = None
    cur_indices: List[int] = []
    cur_end: Optional[bool] = None

    def flush() -> None:
        nonlocal cur_indices, cur_end, cur_loc
        if cur_indices:
            segments.append(
                _Segment(
                    cmd=cmd_idx,
                    indices=list(cur_indices),
                    location=cur_loc,
                    ends_holding=cur_end,
                )
            )
        cur_indices = []
        cur_end = None

    for i, action in enumerate(actions):
        kind = _kind(action)
        if kind == "go_to":
            flush()
            cur_loc = getattr(action, "location_to_go", None)
            cur_indices.append(i)
        else:
            cur_indices.append(i)
            if kind in _GRIPPER_ACQUIRES:
                cur_end = True
            if kind in _GRIPPER_RELEASES:
                cur_end = False
    flush()
    return segments


# State is keyed by place, not by name: the parser names objects and people
# inconsistently across commands, but every action has a place. A place None
# (unknown) overlaps every place.
_OBJECTS = "objects"
_PEOPLE = "people"

# kind -> (domains read, domains written) at the current place.
# Unlisted kinds touch no shared state.
_EFFECTS = {
    "pick_object": ({_OBJECTS}, {_OBJECTS}),
    "place_object": (set(), {_OBJECTS}),
    "give_object": ({_PEOPLE}, set()),
    "count": ({_OBJECTS, _PEOPLE}, set()),
    "get_visual_info": ({_OBJECTS}, set()),
    "find_person": ({_PEOPLE}, set()),
    "find_person_by_name": ({_PEOPLE}, set()),
    "get_person_info": ({_PEOPLE}, set()),
    # Escorts also write the destination; see _action_effects.
    "follow_person_until": ({_PEOPLE}, {_PEOPLE}),
    "guide_person_to": ({_PEOPLE}, {_PEOPLE}),
}
_ESCORT_DESTINATION = {
    "follow_person_until": "destination",
    "guide_person_to": "destination_room",
}


def _norm_location(loc: Any) -> Optional[str]:
    if not isinstance(loc, str):
        return None
    loc = loc.strip().lower().replace("_", " ")
    return loc or None


def _action_effects(action: Any, here: Optional[str]) -> Tuple[set, set, Optional[str]]:
    """``(reads, writes, here_after)`` for one action performed at ``here``."""
    kind = _kind(action)
    if kind == "go_to":
        return set(), set(), _norm_location(getattr(action, "location_to_go", None))
    read_domains, write_domains = _EFFECTS.get(kind, (set(), set()))
    reads = {(d, here) for d in read_domains}
    writes = {(d, here) for d in write_domains}
    here_after = here
    if kind in _ESCORT_DESTINATION:
        here_after = _norm_location(getattr(action, _ESCORT_DESTINATION[kind], None))
        writes.add((_PEOPLE, here_after))
    return reads, writes, here_after


def _annotate_effects(segments: Sequence[_Segment], command: Any) -> None:
    actions = list(getattr(command, "commands", []))
    here: Optional[str] = None
    for seg in segments:
        reads: set = set()
        writes: set = set()
        for i in seg.indices:
            r, w, here = _action_effects(actions[i], here)
            reads |= r
            writes |= w
        seg.reads = frozenset(reads)
        seg.writes = frozenset(writes)


def _room_table(segments: Sequence[_Segment], room_of: Optional[RoomFn]) -> Dict[str, Optional[str]]:
    places = {v[1] for seg in segments for v in seg.reads | seg.writes if v[1] is not None}
    if room_of is None:
        return {p: None for p in places}
    return {p: _norm_location(room_of(p)) for p in places}


def _places_overlap(a: Optional[str], b: Optional[str], room: Dict[str, Optional[str]]) -> bool:
    if a is None or b is None or a == b:
        return True
    return room.get(a) == b or room.get(b) == a


def _interferes(s: _Segment, t: _Segment, room: Dict[str, Optional[str]]) -> bool:
    def clash(writes: FrozenSet[_Var], others: FrozenSet[_Var]) -> bool:
        return any(
            w[0] == o[0] and _places_overlap(w[1], o[1], room) for w in writes for o in others
        )

    return clash(s.writes, t.reads | t.writes) or clash(t.writes, s.reads)


def _build_needs(
    segments_per_cmd: Sequence[Sequence[_Segment]],
    room: Dict[str, Optional[str]],
) -> List[List[Tuple[Tuple[int, int], ...]]]:
    """``needs[c][j]`` = ``((d, m), ...)``: segment j of command c waits until
    an earlier command d (d < c) has done m segments."""
    needs: List[List[Tuple[Tuple[int, int], ...]]] = []
    for c, segs in enumerate(segments_per_cmd):
        per_cmd = []
        for seg in segs:
            req = []
            for d in range(c):
                m = 0
                for i, other in enumerate(segments_per_cmd[d]):
                    if _interferes(other, seg, room):
                        m = i + 1
                if m:
                    req.append((d, m))
            per_cmd.append(tuple(req))
        needs.append(per_cmd)
    return needs


def _euclidean(a: Optional[Tuple[float, float]], b: Optional[Tuple[float, float]]) -> float:
    if a is None or b is None:
        return LARGE_M
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return (dx * dx + dy * dy) ** 0.5


def _euclidean_distance(locator: Locator, origin: Optional[Tuple[float, float]]) -> DistanceFn:
    def dist(a: Optional[str], b: str) -> float:
        return _euclidean(origin if a is None else locator(a), locator(b))

    return dist


def _cost_matrix(
    segments: Sequence[_Segment],
    distance: DistanceFn,
    has_origin: bool,
) -> Tuple[List[float], List[List[float]]]:
    """Seed costs and segment-to-segment costs, querying ``distance`` once per
    distinct location pair. Unresolved or unreachable locations cost LARGE_M."""
    memo: dict = {}

    def cost(a: Optional[str], b: Optional[str]) -> float:
        if b is None:
            return LARGE_M
        if a == b:
            return 0.0
        if (a, b) not in memo:
            d = distance(a, b)
            memo[(a, b)] = LARGE_M if d is None or not d < LARGE_M else d
        return memo[(a, b)]

    M = len(segments)
    locs = [seg.location for seg in segments]
    seed = [cost(None, locs[u]) if has_origin else 0.0 for u in range(M)]
    edge = [
        [LARGE_M if locs[a] is None else cost(locs[a], locs[b]) for b in range(M)]
        for a in range(M)
    ]
    return seed, edge


def _build_holding_after(
    segments_per_cmd: Sequence[Sequence[_Segment]],
) -> List[List[bool]]:
    """For each command c, ``holding_after[c][p]`` is True iff scheduling
    ``p`` segments of cmd c (in source order) leaves the gripper held.

    p ranges 0..len(segments_per_cmd[c]).
    """
    result: List[List[bool]] = []
    for segs in segments_per_cmd:
        held = False
        ha = [False]
        for seg in segs:
            # Order within the segment matters: "place, pick" ends held.
            if seg.ends_holding is not None:
                held = seg.ends_holding
            ha.append(held)
        result.append(ha)
    return result


def _gripper_holder(
    k: Sequence[int],
    holding_after: Sequence[Sequence[bool]],
) -> Optional[int]:
    """The command holding the gripper at progress vector ``k``, or None."""
    for c, kc in enumerate(k):
        if holding_after[c][kc]:
            return c
    return None


def _state_count(seg_lens: Sequence[int]) -> int:
    """Upper bound on DP states: n * prod(L_c + 1)."""
    total = 1
    for length in seg_lens:
        total *= length + 1
    return total * max(len(seg_lens), 1)


def _optimal_schedule(
    segments: Sequence[_Segment],
    seg_ids: Sequence[Sequence[int]],
    holding_after: Sequence[Sequence[bool]],
    distance: DistanceFn,
    has_origin: bool,
    needs: Optional[Sequence[Sequence[Sequence[Tuple[int, int]]]]] = None,
) -> List[int]:
    """DP over per-command progress vectors.

    State (k, last): k[c] = segments of command c done, last = command that
    ran the latest segment (-1 at the start). ``seg_ids[c][j]`` is the flat
    index of command c's j-th segment. States are expanded in layers of equal
    sum(k). O(n^2 * prod(L_c + 1)).
    """
    M = len(segments)
    if M == 0:
        return []

    seed, edge = _cost_matrix(segments, distance, has_origin)
    seg_lens = [len(ids) for ids in seg_ids]
    n = len(seg_lens)

    INF = float("inf")
    start: Tuple[Tuple[int, ...], int] = (tuple([0] * n), -1)
    f = {start: 0.0}
    parent: dict = {start: None}

    layer = [start]
    for _ in range(M):
        next_layer: dict = {}  # insertion-ordered set of states in layer + 1
        for state in layer:
            k, last = state
            cur_cost = f[state]
            holder = _gripper_holder(k, holding_after)
            here = seg_ids[last][k[last] - 1] if last >= 0 else -1
            for c in range(n):
                if k[c] == seg_lens[c]:
                    continue
                u = seg_ids[c][k[c]]
                if holder is not None and holder != c and segments[u].ends_holding is not None:
                    continue
                if needs is not None and any(k[d] < m for d, m in needs[c][k[c]]):
                    continue
                if here < 0:
                    step = seed[u] + _TIEBREAK_EPSILON * c
                else:
                    step = edge[here][u] + (_TIEBREAK_EPSILON if c != last else 0.0)
                new_k = list(k)
                new_k[c] += 1
                nxt = (tuple(new_k), c)
                cand = cur_cost + step
                if cand < f.get(nxt, INF):
                    f[nxt] = cand
                    parent[nxt] = state
                    next_layer[nxt] = None
        layer = list(next_layer)

    full = tuple(seg_lens)
    best: Optional[Tuple[Tuple[int, ...], int]] = None
    best_cost = INF
    for c in range(n):
        cost = f.get((full, c), INF)
        if cost < best_cost:
            best_cost = cost
            best = (full, c)

    if best is None:
        # Only when 2+ commands pick without ever releasing (a parse error).
        return _sequential_schedule(segments)

    schedule: List[int] = []
    state = best
    while state is not None and state[1] >= 0:
        k, last = state
        schedule.append(seg_ids[last][k[last] - 1])
        state = parent[state]
    schedule.reverse()
    return schedule


def _sequential_schedule(segments: Sequence[_Segment]) -> List[int]:
    """Per-command sequential order — feasible by construction (preserves
    each command's prefix and never interleaves across atomic blocks).
    The sort is stable, so each command keeps its own segment order."""
    return sorted(range(len(segments)), key=lambda u: segments[u].cmd)


def merge(
    commands: Sequence[Any],
    locator: Optional[Locator] = None,
    origin: Optional[Tuple[float, float]] = None,
    distance: Optional[DistanceFn] = None,
    dependencies: bool = True,
    room_of: Optional[RoomFn] = None,
) -> InterleavedPlan:
    """Merge N parsed commands into one interleaved plan.

    ``commands`` is a sequence of BAML ``CommandListLLM`` objects.
    ``locator`` resolves a free-text location string to ``(x, y)``; pass
    ``None`` to disable distance-based ordering (useful in tests).
    ``origin`` is the robot's current ``(x, y)`` at planning time; pass
    ``None`` to seed the DP with no origin edge (avoids the misleading
    bias toward the SLAM map origin when the real pose is unavailable).
    ``distance`` overrides the straight-line cost (e.g. a planner's path
    length) and takes location names, ``None`` meaning the robot origin;
    it is only asked about the origin when ``origin`` is not None.
    ``dependencies=False`` disables cross-command conflict ordering.
    ``room_of`` maps a location to its room, so a room overlaps the places
    inside it; without it only identical names conflict.
    """
    if distance is None:
        loc_fn: Locator = locator if locator is not None else (lambda _: None)
        distance = _euclidean_distance(loc_fn, origin)

    if not commands:
        return InterleavedPlan(actions=[], fallback=[])

    flat_segments: List[_Segment] = []
    segments_per_cmd: List[List[_Segment]] = []
    for cmd_idx, command in enumerate(commands):
        per_cmd = _decompose(cmd_idx, command)
        _annotate_effects(per_cmd, command)
        segments_per_cmd.append(per_cmd)
        flat_segments.extend(per_cmd)

    if not flat_segments:
        return InterleavedPlan(actions=[], fallback=_build_fallback(commands))

    # seg_ids[c][j] = flat index of command c's j-th segment.
    seg_ids: List[List[int]] = []
    gi = 0
    for per_cmd in segments_per_cmd:
        seg_ids.append(list(range(gi, gi + len(per_cmd))))
        gi += len(per_cmd)

    holding_after = _build_holding_after(segments_per_cmd)
    needs = (
        _build_needs(segments_per_cmd, _room_table(flat_segments, room_of))
        if dependencies
        else None
    )

    if _state_count([len(ids) for ids in seg_ids]) > _STATE_CAP:
        schedule = _sequential_schedule(flat_segments)
    else:
        schedule = _optimal_schedule(
            flat_segments,
            seg_ids,
            holding_after,
            distance,
            origin is not None,
            needs,
        )

    actions: List[PlanAction] = []
    for si in schedule:
        seg = flat_segments[si]
        for ai in seg.indices:
            action = commands[seg.cmd].commands[ai]
            actions.append(_to_plan_action(action, seg.cmd, ai))

    actions = _collapse_redundant_go_tos(actions)
    fallback = [_collapse_redundant_go_tos(per) for per in _build_fallback(commands)]

    return InterleavedPlan(actions=actions, fallback=fallback)


def _to_plan_action(action: Any, cmd_idx: int, action_idx: int) -> PlanAction:
    kind = _kind(action)
    return PlanAction(
        action=action,
        source_cmd=cmd_idx,
        source_idx=action_idx,
        location=getattr(action, "location_to_go", None),
        requires_gripper=(kind in _GRIPPER_ACQUIRES),
        releases_gripper=(kind in _GRIPPER_RELEASES),
    )


def _build_fallback(commands: Sequence[Any]) -> List[List[PlanAction]]:
    out: List[List[PlanAction]] = []
    for cmd_idx, command in enumerate(commands):
        per_cmd: List[PlanAction] = []
        for ai, action in enumerate(getattr(command, "commands", [])):
            per_cmd.append(_to_plan_action(action, cmd_idx, ai))
        out.append(per_cmd)
    return out


def _collapse_redundant_go_tos(
    actions: Sequence[PlanAction],
) -> List[PlanAction]:
    """Drop a leading ``go_to X`` whenever the previous action ended at X."""
    last_loc: Optional[str] = None
    out: List[PlanAction] = []
    for pa in actions:
        if _kind(pa.action) == "go_to":
            tgt = getattr(pa.action, "location_to_go", None)
            if tgt is not None and tgt == last_loc:
                continue
            if tgt is not None:
                last_loc = tgt
        out.append(pa)
    return out
