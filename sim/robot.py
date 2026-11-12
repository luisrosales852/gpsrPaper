"""Kinematic base, gripper and pedestrians.

Nothing here is dynamic. We write ``qpos`` and call ``mj_forward``; there are no
actuators, no contacts and no solver. That is a deliberate choice: a paper figure
has to be bit-for-bit reproducible, and a PD-controlled base would introduce
tuning noise that has nothing to do with the planner under test.

Every motion funnels through :meth:`SimRobot._advance`, which is also the single
place the recorder is ticked — so sim time, robot pose, carried-object pose and
captured frames can never drift out of sync.
"""

import bisect
import math
from typing import Callable, List, Optional, Sequence, Tuple

import mujoco
import numpy as np

from sim.world import SimObject, SimPerson, World

DT = 1.0 / 60.0
EPS_M = 1e-4       # 1/10 mm: arrival tolerance, far below any real localisation error
LIN_SPEED = 0.55   # m/s   — comfortable indoor service-robot cruise
ANG_SPEED = 1.20   # rad/s
PERSON_SPEED = 0.42
FOLLOW_GAP = 1.15  # m, how far behind a followed person the robot trails


class SimTimeout(Exception):
    """Raised inside ``_advance`` when a handler's sim-time budget is spent.

    This is how ``timeouts.timeout_for`` is actually enforced. py_trees'
    ``Timeout`` decorator only checks *between* ticks and the leaves block, so
    on the real robot the primary enforcement lives inside the subtask managers
    (see ``leaf_behaviours`` module docstring). The sim reproduces that rather
    than faking a wall-clock abort.
    """


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class SimRobot:
    def __init__(self, model, data, world: World):
        self.m, self.d, self.world = model, data, world
        self.t = 0.0
        self.deadline: Optional[float] = None
        self.distance_m = 0.0
        self.held: Optional[SimObject] = None
        self.on_tick: Optional[Callable[["SimRobot"], None]] = None
        self.speech: Optional[str] = None      # current speech-bubble text
        self.speech_anchor: Optional[str] = None  # body name to anchor it to
        self.badges: List[dict] = []           # transient in-world annotations

        self._qadr = {}
        for jname in ("rob_x", "rob_y", "rob_yaw"):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            self._qadr[jname] = model.jnt_qposadr[jid]
        for p in world.people:
            for suffix in ("x", "y", "yaw"):
                jn = f"{p.body}_{suffix}"
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                self._qadr[jn] = model.jnt_qposadr[jid]
        self._obj_qadr = {}
        for o in world.objects:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{o.body}_free")
            self._obj_qadr[o.name] = model.jnt_qposadr[jid]

        self._site_gripper = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripper")
        self._gid_shell = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "robot_shell")
        self._base_rgba = {
            p.name: model.geom_rgba[
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{p.body}_shirt")
            ].copy()
            for p in world.people
        }
        self.sync()

    # ---- state ----------------------------------------------------------

    @property
    def xy(self) -> Tuple[float, float]:
        return (float(self.d.qpos[self._qadr["rob_x"]]), float(self.d.qpos[self._qadr["rob_y"]]))

    @property
    def yaw(self) -> float:
        return float(self.d.qpos[self._qadr["rob_yaw"]])

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        self.d.qpos[self._qadr["rob_x"]] = x
        self.d.qpos[self._qadr["rob_y"]] = y
        self.d.qpos[self._qadr["rob_yaw"]] = yaw

    def person_xy(self, p: SimPerson) -> Tuple[float, float]:
        return (float(self.d.qpos[self._qadr[f"{p.body}_x"]]) + p.home[0],
                float(self.d.qpos[self._qadr[f"{p.body}_y"]]) + p.home[1])

    def set_person_xy(self, p: SimPerson, x: float, y: float, yaw: Optional[float] = None) -> None:
        self.d.qpos[self._qadr[f"{p.body}_x"]] = x - p.home[0]
        self.d.qpos[self._qadr[f"{p.body}_y"]] = y - p.home[1]
        if yaw is not None:
            self.d.qpos[self._qadr[f"{p.body}_yaw"]] = yaw - p.yaw

    def object_xyz(self, o: SimObject) -> Tuple[float, float, float]:
        a = self._obj_qadr[o.name]
        return tuple(float(v) for v in self.d.qpos[a:a + 3])

    def set_object_xyz(self, o: SimObject, x: float, y: float, z: float) -> None:
        a = self._obj_qadr[o.name]
        self.d.qpos[a:a + 3] = (x, y, z)
        self.d.qpos[a + 3:a + 7] = (1.0, 0.0, 0.0, 0.0)

    # ---- visual state ---------------------------------------------------

    def set_shell_color(self, rgba) -> None:
        self.m.geom_rgba[self._gid_shell] = rgba

    def highlight_person(self, p: SimPerson, on: bool) -> None:
        gid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, f"{p.body}_shirt")
        self.m.geom_rgba[gid] = (0.15, 0.95, 0.45, 1.0) if on else self._base_rgba[p.name]

    def clear_annotations(self) -> None:
        self.speech = None
        self.speech_anchor = None
        self.badges = []
        for p in self.world.people:
            self.highlight_person(p, False)

    # ---- the single stepping point --------------------------------------

    def sync(self) -> None:
        mujoco.mj_forward(self.m, self.d)

    def _advance(self, dt: float) -> None:
        """Commit one tick: clock, kinematics, carried object, recorder."""
        self.t += dt
        if self.deadline is not None and self.t > self.deadline:
            raise SimTimeout()
        self.sync()
        if self.held is not None:
            gp = self.d.site_xpos[self._site_gripper]
            self.set_object_xyz(self.held, float(gp[0]), float(gp[1]), float(gp[2]))
            self.sync()
        if self.on_tick is not None:
            self.on_tick(self)

    def idle(self, duration: float) -> None:
        """Hold still for ``duration`` sim-seconds (still recording)."""
        n = max(1, int(round(duration / DT)))
        for _ in range(n):
            self._advance(DT)

    # ---- motion primitives ----------------------------------------------

    def turn_to(self, target_yaw: float) -> None:
        while True:
            err = wrap(target_yaw - self.yaw)
            if abs(err) < 1e-3:
                break
            step = math.copysign(min(ANG_SPEED * DT, abs(err)), err)
            self.set_pose(*self.xy, self.yaw + step)
            self._advance(DT)

    def drive_to(self, x: float, y: float, yaw: Optional[float] = None,
                 stop_within: float = 0.0) -> float:
        """Rotate toward the goal, translate, then settle on ``yaw``.

        Returns the distance actually travelled, which is also accumulated into
        ``self.distance_m`` — the quantity the merged-vs-sequential comparison
        is measured in.
        """
        sx, sy = self.xy
        goal_dist = math.hypot(x - sx, y - sy)
        if goal_dist > 1e-6:
            self.turn_to(math.atan2(y - sy, x - sx))
        travelled = 0.0
        while True:
            cx, cy = self.xy
            rem = math.hypot(x - cx, y - cy)
            # Distance still to cover before the stand-off is satisfied. Tested
            # against EPS_M rather than 0: `rem` converges to `stop_within` from
            # above and can sit one ULP over it forever, which would spin here
            # taking denormal steps until the handler's budget expired.
            slack = rem - stop_within
            if slack <= EPS_M:
                break
            step = min(LIN_SPEED * DT, slack)
            hx, hy = (x - cx) / rem, (y - cy) / rem
            self.set_pose(cx + hx * step, cy + hy * step, math.atan2(hy, hx))
            travelled += step
            self.distance_m += step
            self._advance(DT)
        if yaw is not None:
            self.turn_to(yaw)
        return travelled

    def drive_path(self, waypoints: Sequence[Tuple[float, float]], yaw: Optional[float] = None,
                   stop_within: float = 0.0) -> float:
        """``drive_to`` each waypoint in turn (a path from ``sim.routing``).

        ``stop_within`` and ``yaw`` apply to the final waypoint only; distance,
        deadline and recorder ticking are exactly those of ``drive_to``.
        """
        travelled = 0.0
        last = len(waypoints) - 1
        for i, (x, y) in enumerate(waypoints):
            if i < last:
                travelled += self.drive_to(x, y)
            else:
                travelled += self.drive_to(x, y, yaw=yaw, stop_within=stop_within)
        if not waypoints and yaw is not None:
            self.turn_to(yaw)
        return travelled

    def scan(self, sweep: float = 1.05, duration: float = 2.4) -> None:
        """Yaw sweep ±``sweep``/2, the visible signature of a perception leaf."""
        start = self.yaw
        n = max(2, int(round(duration / DT)))
        for i in range(n):
            phase = math.sin(2.0 * math.pi * (i / n))
            self.set_pose(*self.xy, start + phase * sweep / 2.0)
            self._advance(DT)
        self.turn_to(start)

    # ---- gripper --------------------------------------------------------

    def grasp(self, o: SimObject, duration: float = 1.2) -> None:
        """Lerp the object into the gripper, then latch it."""
        x0, y0, z0 = self.object_xyz(o)
        n = max(1, int(round(duration / DT)))
        for i in range(1, n + 1):
            gp = self.d.site_xpos[self._site_gripper]
            a = i / n
            self.set_object_xyz(o, x0 + (gp[0] - x0) * a, y0 + (gp[1] - y0) * a,
                                z0 + (gp[2] - z0) * a)
            self._advance(DT)
        self.held = o
        self._advance(DT)

    def release(self, to_xyz: Tuple[float, float, float], duration: float = 1.0) -> None:
        o = self.held
        self.held = None
        if o is None:
            return
        x0, y0, z0 = self.object_xyz(o)
        n = max(1, int(round(duration / DT)))
        for i in range(1, n + 1):
            a = i / n
            self.set_object_xyz(o, x0 + (to_xyz[0] - x0) * a, y0 + (to_xyz[1] - y0) * a,
                                z0 + (to_xyz[2] - z0) * a)
            self._advance(DT)

    # ---- pedestrians ----------------------------------------------------
    #
    # Both escort modes move robot and person along one shared polyline
    # ``track`` (planned by sim.routing), each tracked by its arclength, so the
    # trailing party retraces the leader's collision-free path instead of
    # cutting corners toward it.

    def follow_person(self, p: SimPerson, track: Sequence[Tuple[float, float]],
                      person_from: float) -> None:
        """follow_person_until: ``p`` walks ``track`` from arclength
        ``person_from`` to its end; the robot, starting at ``track[0]``, stays
        ``FOLLOW_GAP`` behind along the same track."""
        tr = _Track(track)
        sp, sr = person_from, 0.0
        while True:
            sp = min(tr.length, sp + PERSON_SPEED * DT)
            (px, py), ph = tr.at(sp)
            self.set_person_xy(p, px, py, ph)
            target = max(sr, sp - FOLLOW_GAP)
            step = min(LIN_SPEED * DT, target - sr)
            if sp >= tr.length and step <= EPS_M:
                break
            if step > 0.0:
                sr += step
                (rx, ry), rh = tr.at(sr)
                self.set_pose(rx, ry, rh)
                self.distance_m += step
            self._advance(DT)

    def lead_person(self, p: SimPerson, track: Sequence[Tuple[float, float]],
                    robot_from: float) -> None:
        """guide_person_to: the robot drives ``track`` from arclength
        ``robot_from`` to its end; ``p``, starting at ``track[0]``, trails
        ``FOLLOW_GAP`` behind along the same track."""
        tr = _Track(track)
        sr, sp = robot_from, 0.0
        while sr < tr.length - EPS_M:
            step = min(LIN_SPEED * DT, tr.length - sr)
            sr += step
            (rx, ry), rh = tr.at(sr)
            self.set_pose(rx, ry, rh)
            self.distance_m += step
            target = sr - FOLLOW_GAP
            if target > sp:
                sp = min(target, sp + PERSON_SPEED * 1.6 * DT)
                (px, py), ph = tr.at(sp)
                self.set_person_xy(p, px, py, ph)
            self._advance(DT)


class _Track:
    """A polyline addressed by arclength."""

    def __init__(self, pts: Sequence[Tuple[float, float]]):
        self.pts = [tuple(pts[0])]
        for q in pts[1:]:
            if math.hypot(q[0] - self.pts[-1][0], q[1] - self.pts[-1][1]) > 1e-9:
                self.pts.append(tuple(q))
        self.cum = [0.0]
        for a, b in zip(self.pts, self.pts[1:]):
            self.cum.append(self.cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.length = self.cum[-1]

    def at(self, s: float) -> Tuple[Tuple[float, float], float]:
        """Point and heading at arclength ``s`` (clamped to the ends)."""
        if len(self.pts) == 1:
            return self.pts[0], 0.0
        s = min(max(s, 0.0), self.length)
        i = max(0, min(bisect.bisect_right(self.cum, s) - 1, len(self.pts) - 2))
        (ax, ay), (bx, by) = self.pts[i], self.pts[i + 1]
        seg = self.cum[i + 1] - self.cum[i]
        a = 0.0 if seg <= 0.0 else (s - self.cum[i]) / seg
        return (ax + (bx - ax) * a, ay + (by - ay) * a), math.atan2(by - ay, bx - ax)
