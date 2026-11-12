"""The router contract the sim depends on.

Handlers and ``run_sim`` only ever talk to this protocol, so the geometry
behind it (today a visibility graph, later e.g. an occupancy grid + A*) can be
swapped without touching them. ``distance`` is also what ``run_sim`` hands to
``gpsr_merger.merger.merge(distance=...)`` — the merger never sees obstacles,
only a cost between two points.
"""

from typing import List, Optional, Protocol, Sequence, Tuple

XY = Tuple[float, float]


class Router(Protocol):
    def set_people(self, people_xy: Sequence[XY]) -> None:
        """Replace the dynamic obstacles (people currently standing in the way)."""

    def free_point(self, p: XY) -> XY:
        """``p`` itself if the robot fits there, else the nearest point it does."""

    def plan_path(self, start: XY, goal: XY) -> Optional[List[XY]]:
        """Collision-free waypoints from ``start`` to ``free_point(goal)``.

        Excludes ``start``, ends at the (possibly snapped) goal; ``[]`` when
        already there, ``None`` when unreachable.
        """

    def distance(self, a: XY, b: XY) -> float:
        """Length of ``plan_path(a, b)``; ``inf`` when unreachable."""
