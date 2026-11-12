"""Configuration-space obstacles for a point robot.

Every obstacle is an axis-aligned rectangle grown by the robot's footprint
radius (a Minkowski sum, conservative at corners), so planning can treat the
robot as a point. All apartment furniture is already rectangular; a standing
person is modelled as a square around their body.
"""

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

from sim.mjcf import ROBOT_HALF

XY = Tuple[float, float]

# Farthest point of the robot footprint from its centre: the nose box in
# sim/mjcf.py sits at x = hx + 0.06 with half-extents (0.07, 0.10), so its outer
# corner is (hx + 0.13, 0.10). The gripper box stays inside that.
ROBOT_RADIUS = math.hypot(ROBOT_HALF[0] + 0.13, 0.10)
MARGIN_M = 0.05          # safety gap kept from furniture and walls
PERSON_RADIUS = 0.17     # the shirt capsule radius in sim/mjcf.py
PERSON_MARGIN_M = 0.02   # the robot must still get close enough to hand things over

FURNITURE_INFLATE = ROBOT_RADIUS + MARGIN_M
PERSON_INFLATE = ROBOT_RADIUS + PERSON_RADIUS + PERSON_MARGIN_M


@dataclass(frozen=True)
class Rect:
    x0: float
    y0: float
    x1: float
    y1: float

    def grown(self, r: float) -> "Rect":
        return Rect(self.x0 - r, self.y0 - r, self.x1 + r, self.y1 + r)

    def contains(self, p: XY, tol: float = 1e-9) -> bool:
        """Strict interior — the boundary itself is free space."""
        return (self.x0 + tol < p[0] < self.x1 - tol) and (self.y0 + tol < p[1] < self.y1 - tol)

    def corners(self) -> List[XY]:
        return [(self.x0, self.y0), (self.x1, self.y0), (self.x1, self.y1), (self.x0, self.y1)]


def furniture_rects(furniture: Iterable[dict]) -> List[Rect]:
    return [Rect(*f["rect"]) for f in furniture]


def inflated_furniture(furniture: Iterable[dict]) -> List[Rect]:
    return [r.grown(FURNITURE_INFLATE) for r in furniture_rects(furniture)]


def inflated_people(people_xy: Sequence[XY]) -> List[Rect]:
    h = PERSON_INFLATE
    return [Rect(x - h, y - h, x + h, y + h) for x, y in people_xy]


def free_bounds(bounds) -> Rect:
    """Where the robot's centre may be: the room shrunk by the inflation."""
    (bx0, by0), (bx1, by1) = bounds
    r = FURNITURE_INFLATE
    return Rect(bx0 + r, by0 + r, bx1 - r, by1 - r)
