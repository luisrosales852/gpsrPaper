"""Frame capture, HUD state and the event log.

Frames are piped straight into ``ffmpeg`` as rawvideo — at 30 fps a full run is
thousands of frames, and writing them as PNGs first would cost gigabytes for no
benefit.

``run.json`` is the point of this module: every number quoted in the paper comes
out of the event log, not off a video.
"""

import json
import os
import subprocess
from typing import Any, Dict, List, Optional, Sequence, Tuple

import mujoco
import numpy as np
from PIL import Image

from sim.overlay import PANEL_W, camera_matrix, cmd_rgba, draw_panel, draw_scene_annotations

TRAIL_DT = 0.08   # sim-seconds between trail samples


def action_label(action: Any) -> str:
    """Compact ``kind(arg)`` label, matching ``_merger_helpers._label``."""
    kind = getattr(action, "action", "?")
    for arg in ("location_to_go", "object_to_pick", "name", "attribute_value",
                "destination", "destination_room", "info_type", "target_to_count",
                "object_category", "user_instruction"):
        v = getattr(action, arg, None)
        if v:
            return f"{kind}({v})"
    return kind


class Recorder:
    def __init__(self, model, data, world, robot, out_dir: str, scenario: str,
                 mode_label: str, commands: Sequence[Sequence[Any]],
                 width: int = 920, height: int = 720, fps: int = 30,
                 speed: float = 2.0, camera: str = "topdown", video: bool = True):
        self.m, self.d, self.world, self.robot = model, data, world, robot
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.scenario, self.mode_label = scenario, mode_label
        self.width, self.height, self.fps, self.speed = width, height, fps, speed
        self.cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
        self.camera = camera

        self.labels = [[action_label(a) for a in cmd] for cmd in commands]
        self.trail: List[Tuple[float, float, int]] = []
        # Where every person stood at each trail sample (same index), so tests
        # can check robot-person clearance from the log alone.
        self.people_trail: List[List[Tuple[float, float]]] = []
        self.waypoints: List[Dict[str, Any]] = []
        self.events: List[Dict[str, Any]] = []
        self.done: Dict[Tuple[int, int], bool] = {}
        self.current: Optional[Tuple[int, int]] = None
        self.leaf_line = ""
        self.branch = "interleaved"

        self._next_capture = 0.0
        self._next_trail = 0.0
        self._frames = 0
        # (sim-time, metres driven) per captured frame; lets sim.compose line
        # two runs up on one clock without trusting frame indices.
        self.frame_log: List[Tuple[float, float]] = []
        self._renderer = mujoco.Renderer(model, height, width) if video else None
        self._proc = self._open_ffmpeg() if video else None

    # ---- video sink ------------------------------------------------------

    def _open_ffmpeg(self):
        path = os.path.join(self.out_dir, "run.mp4")
        cmd = [
            "ffmpeg", "-y", "-loglevel", "error",
            "-f", "rawvideo", "-pixel_format", "rgb24",
            "-video_size", f"{self.width + PANEL_W}x{self.height}",
            "-framerate", str(self.fps), "-i", "-",
            "-c:v", "libx264", "-preset", "medium", "-crf", "20",
            "-pix_fmt", "yuv420p", path,
        ]
        return subprocess.Popen(cmd, stdin=subprocess.PIPE)

    def close(self) -> Optional[str]:
        if self._proc is None:
            return None
        self._proc.stdin.close()
        self._proc.wait()
        return os.path.join(self.out_dir, "run.mp4")

    # ---- per-tick --------------------------------------------------------

    def _body_xyz(self, body_name: str, dz: float = 0.0):
        bid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if bid < 0:
            return None
        p = self.d.xpos[bid]
        return (float(p[0]), float(p[1]), float(p[2]) + dz)

    def _hud_state(self) -> dict:
        cmd = self.current[0] if self.current else 0
        speech_xyz = None
        if self.robot.speech:
            if self.robot.speech_anchor in (None, "robot"):
                x, y = self.robot.xy
                speech_xyz = (x, y, 1.45)
            else:
                speech_xyz = self._body_xyz(self.robot.speech_anchor, dz=0.35)
        badges = [{"xyz": self._body_xyz(b["body"], dz=0.10), "text": b["text"],
                   "color": b["color"]} for b in self.robot.badges]
        rx, ry = self.robot.xy
        return {
            "robot_xyz": (rx, ry, 0.30),
            "mode_label": self.mode_label,
            "scenario": self.scenario,
            "t": self.robot.t,
            "distance": self.robot.distance_m,
            "held": self.robot.held.kind if self.robot.held else None,
            "commands": self.labels,
            "current": self.current,
            "done": self.done,
            "leaf_line": self.leaf_line,
            "trail": self.trail,
            "waypoints": self.waypoints,
            "badges": badges,
            "speech": self.robot.speech,
            "speech_xyz": speech_xyz,
        }

    def tick(self, robot) -> None:
        """Called from ``SimRobot._advance`` — the only capture point."""
        if robot.t >= self._next_trail:
            self._next_trail = robot.t + TRAIL_DT
            x, y = robot.xy
            self.trail.append((x, y, self.current[0] if self.current else 0))
            self.people_trail.append([robot.person_xy(p) for p in self.world.people])

        if self._renderer is None or self._proc is None or robot.t < self._next_capture:
            return
        # Advance on the schedule, not from robot.t: ticks overshoot by up to
        # one DT and re-basing on them would let the frame clock drift.
        self._next_capture += self.speed / self.fps
        self.frame_log.append((round(robot.t, 4), round(robot.distance_m, 4)))

        self._renderer.update_scene(self.d, camera=self.camera)
        img = Image.fromarray(self._renderer.render())
        M = camera_matrix(self.m, self.d, self.cam_id, self.width, self.height)
        state = self._hud_state()
        draw_scene_annotations(img, M, state)
        frame = draw_panel(img, state)
        self._proc.stdin.write(np.asarray(frame, dtype=np.uint8).tobytes())
        self._frames += 1

    # ---- events ----------------------------------------------------------

    def action_start(self, plan_action, robot) -> None:
        key = (plan_action.source_cmd, plan_action.source_idx)
        self.current = key
        kind = getattr(plan_action.action, "action", "?")
        self.leaf_line = f"ActionLeaf({kind})"
        robot.set_shell_color(cmd_rgba(plan_action.source_cmd))
        x, y = robot.xy
        self.events.append({
            "t": round(robot.t, 3), "phase": "start", "kind": kind,
            "cmd": key[0], "idx": key[1], "x": round(x, 3), "y": round(y, 3),
            "held": robot.held.kind if robot.held else None,
            "distance_m": round(robot.distance_m, 3), "branch": self.branch,
        })

    def action_end(self, plan_action, status, result, robot) -> None:
        key = (plan_action.source_cmd, plan_action.source_idx)
        ok = getattr(status, "name", str(status)) == "EXECUTION_SUCCESS"
        self.done[key] = ok
        kind = getattr(plan_action.action, "action", "?")
        x, y = robot.xy
        if kind == "go_to" and ok:
            self.waypoints.append({"x": x, "y": y, "cmd": key[0],
                                   "loc": getattr(plan_action.action, "location_to_go", None)})
        self.events.append({
            "t": round(robot.t, 3), "phase": "end", "kind": kind,
            "cmd": key[0], "idx": key[1],
            "status": getattr(status, "name", str(status)),
            "result": None if result is None else str(result)[:120],
            "x": round(x, 3), "y": round(y, 3),
            "held": robot.held.kind if robot.held else None,
            "distance_m": round(robot.distance_m, 3), "branch": self.branch,
        })

    # ---- output ----------------------------------------------------------

    def summary(self, extra: Optional[dict] = None) -> dict:
        ends = [e for e in self.events if e["phase"] == "end"]
        kinds = sorted({e["kind"] for e in ends})
        out = {
            "scenario": self.scenario,
            "mode": self.mode_label,
            "sim_time_s": round(self.robot.t, 3),
            "actual_distance_m": round(self.robot.distance_m, 3),
            "actions_executed": len(ends),
            "actions_succeeded": sum(1 for e in ends if e["status"] == "EXECUTION_SUCCESS"),
            "go_tos_executed": sum(1 for e in ends if e["kind"] == "go_to"),
            "leaf_kinds_exercised": kinds,
            "n_leaf_kinds": len(kinds),
            "frames": self._frames,
            "video_speed": self.speed,
        }
        out.update(extra or {})
        return out

    def write(self, extra: Optional[dict] = None) -> str:
        path = os.path.join(self.out_dir, "run.json")
        with open(path, "w") as f:
            json.dump({
                "summary": self.summary(extra),
                "events": self.events,
                "waypoints": self.waypoints,
                "labels": self.labels,
                "frames": self.frame_log,
                "trail": [[round(x, 3), round(y, 3), c] for x, y, c in self.trail],
                "people_trail": [[[round(x, 3), round(y, 3)] for x, y in ps]
                                 for ps in self.people_trail],
            }, f, indent=1)
        return path
