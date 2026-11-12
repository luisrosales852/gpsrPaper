"""Visibility-graph router: exact shortest collision-free paths among rectangles.

Nodes are the corners of every inflated obstacle; two points are connected
when the straight segment between them does not cross any obstacle interior.
For polygonal obstacles the shortest path always bends only at such corners,
so Dijkstra over this graph returns the true geodesic — no grid, no resolution
parameter, bit-for-bit reproducible distances.

Pure Python. People are dynamic obstacles: ``set_people`` rebuilds the graph,
which is a few dozen nodes and takes milliseconds.
"""

import heapq
import math
from typing import Dict, List, Optional, Sequence, Tuple

from sim.routing.obstacles import (
    XY,
    Rect,
    free_bounds,
    inflated_furniture,
    inflated_people,
)

# How far corner nodes are pushed off their obstacle, so a node never sits
# exactly on the boundary it was generated from.
_CORNER_EPS = 1e-4


def _dist(a: XY, b: XY) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def segment_hits_rect(p: XY, q: XY, r: Rect, tol: float = 1e-9) -> bool:
    """Does segment p-q pass through the *interior* of ``r``? (Liang–Barsky.)

    Grazing an edge or a corner does not count: those are the paths the graph
    is made of.
    """
    x0, y0, x1, y1 = r.x0 + tol, r.y0 + tol, r.x1 - tol, r.y1 - tol
    dx, dy = q[0] - p[0], q[1] - p[1]
    t0, t1 = 0.0, 1.0
    for d, lo, hi, s in ((dx, x0, x1, p[0]), (dy, y0, y1, p[1])):
        if abs(d) < 1e-15:
            if not (lo < s < hi):
                return False
            continue
        a, b = (lo - s) / d, (hi - s) / d
        if a > b:
            a, b = b, a
        t0, t1 = max(t0, a), min(t1, b)
        if t0 >= t1:
            return False
    return True


class VisGraphRouter:
    def __init__(self, world, people_xy: Optional[Sequence[XY]] = None):
        self.bounds = free_bounds(world.bounds)
        self.static: List[Rect] = inflated_furniture(world.furniture)
        self._people_key: Optional[Tuple[XY, ...]] = None
        self._dist_cache: Dict[Tuple, float] = {}
        self.set_people(people_xy or [])

    # ---- obstacles -------------------------------------------------------

    def set_people(self, people_xy: Sequence[XY]) -> None:
        key = tuple((round(x, 6), round(y, 6)) for x, y in people_xy)
        if key == self._people_key:
            return
        self._people_key = key
        self.rects = self.static + inflated_people(key)
        self._build_graph()

    def _in_free(self, p: XY) -> bool:
        b = self.bounds
        if not (b.x0 - 1e-9 <= p[0] <= b.x1 + 1e-9 and b.y0 - 1e-9 <= p[1] <= b.y1 + 1e-9):
            return False
        return not any(r.contains(p) for r in self.rects)

    def _visible(self, p: XY, q: XY) -> bool:
        # Both ends are inside the (convex) free bounds, so only obstacles matter.
        return not any(segment_hits_rect(p, q, r) for r in self.rects)

    def _build_graph(self) -> None:
        nodes: List[XY] = []
        e = _CORNER_EPS
        for r in self.rects:
            for cx, cy in r.corners():
                p = (cx + (e if cx == r.x1 else -e), cy + (e if cy == r.y1 else -e))
                if self._in_free(p):
                    nodes.append(p)
        self.nodes = nodes
        n = len(nodes)
        adj: List[List[Tuple[int, float]]] = [[] for _ in range(n)]
        for i in range(n):
            for j in range(i + 1, n):
                if self._visible(nodes[i], nodes[j]):
                    d = _dist(nodes[i], nodes[j])
                    adj[i].append((j, d))
                    adj[j].append((i, d))
        self.adj = adj
        self._dist_cache = {}

    # ---- queries ---------------------------------------------------------

    def free_point(self, p: XY) -> XY:
        """Nearest free point to ``p`` (``p`` itself when already free).

        Candidates are ``p`` clamped into the room and projected just outside
        each edge of each obstacle containing it; projections that land in
        another obstacle are expanded once more. Deterministic, and exact for
        the non-overlapping layouts used here.
        """
        b = self.bounds
        p = (min(max(p[0], b.x0), b.x1), min(max(p[1], b.y0), b.y1))
        if self._in_free(p):
            return p
        frontier, best = [p], None
        e = 2 * _CORNER_EPS
        for _ in range(3):
            nxt = []
            for c in frontier:
                for r in self.rects:
                    if not r.contains(c):
                        continue
                    for q in ((r.x0 - e, c[1]), (r.x1 + e, c[1]),
                              (c[0], r.y0 - e), (c[0], r.y1 + e)):
                        q = (min(max(q[0], b.x0), b.x1), min(max(q[1], b.y0), b.y1))
                        if self._in_free(q):
                            if best is None or _dist(p, q) < _dist(p, best):
                                best = q
                        else:
                            nxt.append(q)
            if best is not None:
                return best
            frontier = nxt
        return p  # boxed in; the caller's straight-line fallback applies

    def plan_path(self, start: XY, goal: XY) -> Optional[List[XY]]:
        goal = self.free_point(goal)
        lead: List[XY] = []
        if not self._in_free(start):
            # Only after an externally imposed pose; step out first.
            s2 = self.free_point(start)
            lead, start = [s2], s2
        if _dist(start, goal) < 1e-9:
            return lead
        if self._visible(start, goal):
            return lead + [goal]

        n = len(self.nodes)
        S, G = n, n + 1
        pts = self.nodes + [start, goal]
        vis_s = [(j, _dist(start, pts[j])) for j in range(n) if self._visible(start, pts[j])]
        vis_g = {j: _dist(goal, pts[j]) for j in range(n) if self._visible(goal, pts[j])}

        dist = {S: 0.0}
        prev: Dict[int, int] = {}
        heap = [(0.0, S)]
        while heap:
            d, u = heapq.heappop(heap)
            if u == G:
                break
            if d > dist.get(u, math.inf):
                continue
            edges = vis_s if u == S else self.adj[u]
            if u in vis_g:
                edges = list(edges) + [(G, vis_g[u])]
            for v, w in edges:
                nd = d + w
                if nd < dist.get(v, math.inf):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(heap, (nd, v))
        if G not in dist:
            return None
        path, u = [], G
        while u != S:
            path.append(pts[u])
            u = prev[u]
        return lead + path[::-1]

    def distance(self, a: XY, b: XY) -> float:
        key = (a, b)
        hit = self._dist_cache.get(key)
        if hit is None:
            path = self.plan_path(a, b)
            hit = math.inf if path is None else path_length(a, path)
            self._dist_cache[key] = hit
        return hit


def path_length(start: XY, path: Sequence[XY]) -> float:
    total, last = 0.0, start
    for w in path:
        total += _dist(last, w)
        last = w
    return total
