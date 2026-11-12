"""Reference subset (Held-Karp) DP — the merger's solver before the switch to
progress vectors. Kept only as a test oracle: both DPs search the same feasible
schedules with the same costs, so they must agree on the optimal cost.

O(2^M * M^2): keep M small (<= ~12) when calling it.
"""

from typing import List, Optional, Sequence

from gpsr_merger.merger import _TIEBREAK_EPSILON, _cost_matrix, _Segment


def heldkarp_schedule(
    segments: Sequence[_Segment],
    holding_after: Sequence[Sequence[bool]],
    distance,
    has_origin: bool,
    needs=None,
) -> Optional[List[int]]:
    """Return the optimal flat-index schedule, or None if none is feasible.

    ``segments`` must be in command-major order (as ``merge`` builds them).
    ``needs[c][j]`` (optional) lists ``(d, m)``: segment j of command c needs
    at least m segments of command d done, the same constraint as the merger.
    """
    M = len(segments)
    if M == 0:
        return []
    n = max(s.cmd for s in segments) + 1
    seg_cmd = [s.cmd for s in segments]
    seg_pos: List[int] = []
    cmd_seg_mask = [0] * n
    seen = [0] * n
    for u, s in enumerate(segments):
        seg_pos.append(seen[s.cmd])
        seen[s.cmd] += 1
        cmd_seg_mask[s.cmd] |= 1 << u

    def holder_of(S: int) -> Optional[int]:
        for c in range(n):
            if holding_after[c][(S & cmd_seg_mask[c]).bit_count()]:
                return c
        return None

    seed, edge = _cost_matrix(segments, distance, has_origin)
    INF = float("inf")
    full = (1 << M) - 1
    f = [[INF] * M for _ in range(1 << M)]
    parent = [[-1] * M for _ in range(1 << M)]
    for u in range(M):
        if seg_pos[u] != 0:
            continue
        if needs is not None and needs[seg_cmd[u]][0]:
            continue  # must wait for an earlier command: can't be first
        f[1 << u][u] = seed[u] + _TIEBREAK_EPSILON * seg_cmd[u]

    for S in range(1, 1 << M):
        holder = holder_of(S)
        for last in range(M):
            if not (S >> last) & 1 or f[S][last] == INF:
                continue
            for u in range(M):
                if (S >> u) & 1:
                    continue
                if (S & cmd_seg_mask[seg_cmd[u]]).bit_count() != seg_pos[u]:
                    continue
                if needs is not None and any(
                    (S & cmd_seg_mask[d]).bit_count() < m for d, m in needs[seg_cmd[u]][seg_pos[u]]
                ):
                    continue
                if holder is not None and holder != seg_cmd[u] and segments[u].ends_holding is not None:
                    continue
                cand = (
                    f[S][last]
                    + edge[last][u]
                    + (_TIEBREAK_EPSILON if seg_cmd[u] != seg_cmd[last] else 0.0)
                )
                if cand < f[S | (1 << u)][u]:
                    f[S | (1 << u)][u] = cand
                    parent[S | (1 << u)][u] = last

    best_last = min(range(M), key=lambda u: f[full][u])
    if f[full][best_last] == INF:
        return None
    schedule: List[int] = []
    S, last = full, best_last
    while last != -1:
        schedule.append(last)
        S, last = S ^ (1 << last), parent[S][last]
    schedule.reverse()
    return schedule
