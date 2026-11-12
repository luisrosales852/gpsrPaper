"""Apartment model: floor plan, locations, objects and people.

Loaded from ``sim/apartment.json``. The only contract the merger needs is
``World.locator`` — a ``Callable[[str], Optional[Tuple[float, float]]]``,
the same shape as ``tests._merger_helpers.make_locator``.
"""

import json
import math
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_APARTMENT = os.path.join(HERE, "apartment.json")


@dataclass
class SimObject:
    name: str
    kind: str
    category: str
    home: Tuple[float, float, float]  # spawn pose (x, y, z)
    size: float
    color: Tuple[float, float, float, float]
    body: str = ""  # MuJoCo body name


@dataclass
class SimPerson:
    name: str
    home: Tuple[float, float]
    yaw: float
    shirt: Tuple[float, float, float, float]
    attributes: List[str] = field(default_factory=list)
    info: Dict[str, str] = field(default_factory=dict)
    body: str = ""


def _norm(text: str) -> str:
    """Lowercase and strip punctuation-ish noise. No plural handling."""
    t = "".join(c for c in (text or "").lower().strip() if c.isalnum() or c in " _-")
    return t.replace("-", " ").replace("_", " ").strip()


def _forms(text: str) -> set:
    """Singular/plural candidate forms of ``text``.

    Returned as a set rather than one stem so ``apples``/``apple`` and
    ``dishes``/``dish`` both match without ``apples`` collapsing to ``appl``.
    Matching is set intersection, which is symmetric — the caller does not need
    to know which side was written in the plural.
    """
    t = _norm(text)
    if not t:
        return set()
    out = {t}
    if t.endswith("s") and len(t) > 2:
        out.add(t[:-1])
    if t.endswith("es") and len(t) > 3:
        out.add(t[:-2])
    if t.endswith("ies") and len(t) > 4:
        out.add(t[:-3] + "y")
    return out


class World:
    def __init__(self, path: str = DEFAULT_APARTMENT):
        with open(path) as f:
            self.spec: Dict[str, Any] = json.load(f)

        self.bounds = self.spec["bounds"]
        self.zones = self.spec.get("zones", [])
        self.furniture = self.spec.get("furniture", [])
        self.wall_height = float(self.spec.get("wall_height", 1.2))

        self.locations: Dict[str, Tuple[float, float, float]] = {
            k: (float(v[0]), float(v[1]), float(v[2]) if len(v) > 2 else 0.0)
            for k, v in self.spec["locations"].items()
        }

        self.objects: List[SimObject] = []
        for i, o in enumerate(self.spec.get("objects", [])):
            self.objects.append(
                SimObject(
                    name=o["name"],
                    kind=o.get("kind", o["name"]),
                    category=o.get("category", "object"),
                    home=(float(o["at"][0]), float(o["at"][1]), float(o.get("z", 0.6))),
                    size=float(o.get("size", 0.06)),
                    color=tuple(o.get("color", [0.8, 0.3, 0.3, 1])),
                    body=f"obj_{o['name']}",
                )
            )

        self.people: List[SimPerson] = []
        for p in self.spec.get("people", []):
            self.people.append(
                SimPerson(
                    name=p["name"],
                    home=(float(p["at"][0]), float(p["at"][1])),
                    yaw=float(p.get("yaw", 0.0)),
                    shirt=tuple(p.get("shirt", [0.3, 0.3, 0.8, 1])),
                    attributes=list(p.get("attributes", [])),
                    info=dict(p.get("info", {})),
                    body=f"person_{p['name']}",
                )
            )

    # ---- the merger-facing contract -------------------------------------

    def locator(self, name: str) -> Optional[Tuple[float, float]]:
        """Resolve a free-text location to (x, y), or None if unknown.

        Mirrors ``GPSRTM._resolve_xy`` on the real robot: unknown names return
        None so the merger's ``LARGE_M`` sentinel kicks in rather than raising.
        """
        hit = self.locations.get(name)
        if hit is None:
            hit = self.locations.get(_norm(name).replace(" ", "_"))
        return None if hit is None else (hit[0], hit[1])

    def room_of(self, name: str) -> Optional[str]:
        """The zone (room) a location lies in, or None if unknown.

        The merger's ``room_of`` hook: lets a room-level reference ("count the
        people in the living room") conflict with a spot inside that room
        ("guide the person from the couch"). Mirrors the robot, where a
        location's room is ``query_location(name).area`` from areas.json.
        """
        xy = self.locator(name)
        if xy is None:
            return None
        for zone in self.zones:
            x0, y0, x1, y1 = zone["rect"]
            if x0 <= xy[0] <= x1 and y0 <= xy[1] <= y1:
                return zone["name"]
        return None

    def pose(self, name: str) -> Optional[Tuple[float, float, float]]:
        """Full (x, y, yaw) target pose for a location name."""
        hit = self.locations.get(name)
        if hit is None:
            hit = self.locations.get(_norm(name).replace(" ", "_"))
        return hit

    # ---- lookups used by the leaf handlers -------------------------------

    def find_object(self, text: str) -> Optional[SimObject]:
        want = _forms(text)
        for o in self.objects:
            if want & (_forms(o.kind) | _forms(o.name)):
                return o
        return None

    def objects_matching(self, text: str) -> List[SimObject]:
        """Every object whose kind or category matches ``text`` (plural-safe)."""
        want = _forms(text)
        if not want:
            return []
        return [o for o in self.objects if want & (_forms(o.kind) | _forms(o.category))]

    def person_by_name(self, name: str) -> Optional[SimPerson]:
        want = _norm(name)
        for p in self.people:
            if _norm(p.name) == want:
                return p
        return None

    def person_by_attribute(self, attribute: str) -> Optional[SimPerson]:
        want = _norm(attribute)
        if not want:
            return None
        for p in self.people:
            for a in p.attributes:
                na = _norm(a)
                if na == want or want in na or na in want:
                    return p
        return None

    @staticmethod
    def distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])
