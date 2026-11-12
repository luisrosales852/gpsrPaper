"""Obstacle-aware navigation for the simulated apartment.

Everything about *how the robot gets somewhere* lives here, never in
``gpsr_merger/``: on the real robot Nav2 owns navigation. The merger only
receives ``Router.distance`` as its travel-cost function.
"""

from sim.routing.base import XY, Router
from sim.routing.obstacles import (
    FURNITURE_INFLATE,
    PERSON_INFLATE,
    PERSON_RADIUS,
    ROBOT_RADIUS,
    Rect,
    furniture_rects,
)
from sim.routing.visgraph import VisGraphRouter, path_length, segment_hits_rect

__all__ = [
    "XY", "Router", "VisGraphRouter", "Rect", "furniture_rects", "path_length",
    "segment_hits_rect", "ROBOT_RADIUS", "PERSON_RADIUS", "FURNITURE_INFLATE",
    "PERSON_INFLATE",
]
