"""HUD compositing: PIL annotation on top of MuJoCo RGB frames.

Everything 2-D is drawn here rather than as MuJoCo decorative geoms, because the
trail needs per-source-command colour, alpha and anti-aliasing, and the speech
bubbles need text — none of which MuJoCo's scene geoms give you.

The single most important encoding is **colour by source command**: the robot
shell, the trail behind it and the command list all share one hue per spoken
command. Interleaving therefore becomes readable in a *still* frame — a
multicoloured trail is a merged plan, four monochrome excursions are a
sequential one.
"""

import math
import os
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib
import numpy as np
from PIL import Image, ImageDraw, ImageFont

_FONT_DIR = os.path.join(os.path.dirname(matplotlib.__file__), "mpl-data", "fonts", "ttf")

# Okabe-Ito qualitative palette: colourblind-safe, the scientific-figure default.
CMD_COLORS: List[Tuple[int, int, int]] = [
    (213, 94, 0),     # vermillion
    (0, 114, 178),    # blue
    (0, 158, 115),    # bluish green
    (204, 121, 167),  # reddish purple
    (230, 159, 0),    # orange
    (86, 180, 233),   # sky blue
]
PANEL_W = 380
BG = (22, 24, 30)
FG = (236, 238, 242)
DIM = (140, 146, 158)
OK = (0, 190, 130)
BAD = (226, 84, 70)

_font_cache: Dict[Tuple[str, int], ImageFont.FreeTypeFont] = {}


def font(name: str = "DejaVuSans.ttf", size: int = 16) -> ImageFont.FreeTypeFont:
    key = (name, size)
    if key not in _font_cache:
        _font_cache[key] = ImageFont.truetype(os.path.join(_FONT_DIR, name), size)
    return _font_cache[key]


def cmd_color(i: int) -> Tuple[int, int, int]:
    return CMD_COLORS[i % len(CMD_COLORS)]


def cmd_rgba(i: int, a: float = 1.0) -> Tuple[float, float, float, float]:
    r, g, b = cmd_color(i)
    return (r / 255.0, g / 255.0, b / 255.0, a)


def camera_matrix(model, data, cam_id: int, width: int, height: int) -> np.ndarray:
    """World -> pixel 3x4 matrix for a MuJoCo camera (standard recipe)."""
    pos = data.cam_xpos[cam_id]
    rot = data.cam_xmat[cam_id].reshape(3, 3).T
    fovy = model.cam_fovy[cam_id]

    T = np.eye(4)
    T[:3, 3] = -pos
    R = np.eye(4)
    R[:3, :3] = rot
    f = 0.5 * height / math.tan(fovy * math.pi / 360.0)
    P = np.array([[-f, 0.0, width / 2.0, 0.0],
                  [0.0, f, height / 2.0, 0.0],
                  [0.0, 0.0, 1.0, 0.0]])
    return P @ R @ T


def project(M: np.ndarray, xyz: Sequence[float]) -> Tuple[float, float]:
    v = M @ np.array([xyz[0], xyz[1], xyz[2], 1.0])
    w = v[2] if abs(v[2]) > 1e-9 else 1e-9
    return (float(v[0] / w), float(v[1] / w))


# ---------------------------------------------------------------------------


def _rounded(draw, box, radius, fill, outline=None, width=1):
    draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)


def _bubble(draw, xy, text, fill=(250, 250, 252), fg=(20, 22, 28), max_w=330):
    """Speech bubble with a tail, clamped inside the pane."""
    f = font("DejaVuSans.ttf", 17)
    words, lines, cur = text.split(), [], ""
    for w in words:
        trial = (cur + " " + w).strip()
        if draw.textlength(trial, font=f) > max_w and cur:
            lines.append(cur)
            cur = w
        else:
            cur = trial
    if cur:
        lines.append(cur)
    lh = 22
    tw = max(draw.textlength(ln, font=f) for ln in lines) + 24
    th = lh * len(lines) + 16
    x, y = xy[0] - tw / 2, xy[1] - th - 26
    _rounded(draw, (x, y, x + tw, y + th), 10, fill, outline=(60, 64, 74), width=2)
    draw.polygon([(xy[0] - 9, y + th - 1), (xy[0] + 9, y + th - 1), (xy[0], y + th + 14)],
                 fill=fill, outline=(60, 64, 74))
    for i, ln in enumerate(lines):
        draw.text((x + 12, y + 8 + i * lh), ln, font=f, fill=fg)


def draw_scene_annotations(img: Image.Image, M: np.ndarray, state: dict) -> None:
    """Trail, waypoint pins, badges and the speech bubble, in world space."""
    d = ImageDraw.Draw(img, "RGBA")

    # Trail: one polyline per contiguous same-command run.
    trail = state.get("trail", [])
    run: List[Tuple[float, float]] = []
    run_cmd = None
    for (x, y, c) in trail:
        p = project(M, (x, y, 0.05))
        if c != run_cmd:
            if len(run) > 1:
                d.line(run, fill=cmd_color(run_cmd) + (215,), width=6, joint="curve")
            run = run[-1:] if run else []
            run_cmd = c
        run.append(p)
    if len(run) > 1 and run_cmd is not None:
        d.line(run, fill=cmd_color(run_cmd) + (215,), width=6, joint="curve")

    # Visited waypoints, numbered in execution order.
    for i, wp in enumerate(state.get("waypoints", [])):
        px, py = project(M, (wp["x"], wp["y"], 0.05))
        col = cmd_color(wp["cmd"])
        d.ellipse((px - 13, py - 13, px + 13, py + 13), fill=col + (235,),
                  outline=(255, 255, 255), width=2)
        t = str(i + 1)
        f = font("DejaVuSans-Bold.ttf", 15)
        d.text((px - d.textlength(t, font=f) / 2, py - 9), t, font=f, fill=(255, 255, 255))

    # Robot ring: keeps the base trackable at plan-view zoom and repeats the
    # source-command colour that the trail and the command list already use.
    if state.get("robot_xyz") is not None:
        rx, ry = project(M, state["robot_xyz"])
        col = cmd_color(state["current"][0] if state.get("current") else 0)
        d.ellipse((rx - 26, ry - 26, rx + 26, ry + 26), outline=col + (255,), width=4)
        d.ellipse((rx - 33, ry - 33, rx + 33, ry + 33), outline=(255, 255, 255, 120), width=2)

    # Badges over counted / measured objects.
    for b in state.get("badges", []):
        if b.get("xyz") is None:
            continue
        px, py = project(M, b["xyz"])
        col = tuple(b.get("color", (250, 210, 60)))
        f = font("DejaVuSans-Bold.ttf", 15)
        tw = d.textlength(b["text"], font=f) + 14
        _rounded(d, (px - tw / 2, py - 44, px + tw / 2, py - 20), 8, col + (240,),
                 outline=(30, 30, 34), width=2)
        d.text((px - d.textlength(b["text"], font=f) / 2, py - 42), b["text"],
               font=f, fill=(25, 25, 30))

    if state.get("speech") and state.get("speech_xyz") is not None:
        px, py = project(M, state["speech_xyz"])
        px = min(max(px, 190), img.width - 190)
        py = max(py, 120)
        _bubble(d, (px, py), state["speech"])


def draw_panel(img: Image.Image, state: dict) -> Image.Image:
    """Compose the left HUD panel and paste the 3-D pane beside it."""
    H = img.height
    out = Image.new("RGB", (img.width + PANEL_W, H), BG)
    out.paste(img, (PANEL_W, 0))
    d = ImageDraw.Draw(out, "RGBA")

    fb = font("DejaVuSans-Bold.ttf", 20)
    fm = font("DejaVuSansMono.ttf", 12)
    fmb = font("DejaVuSansMono-Bold.ttf", 12)
    fs = font("DejaVuSans.ttf", 13)
    fsb = font("DejaVuSans-Bold.ttf", 14)
    ROW, CMD_H, CMD_GAP = 18, 21, 6

    y = 14
    d.text((20, y), state.get("mode_label", "MERGED"), font=fb, fill=FG)
    y += 27
    d.text((20, y), state.get("scenario", ""), font=fs, fill=DIM)
    y += 24

    # Metric strip: the numbers the merged-vs-sequential claim rests on.
    for label, value in (
        ("sim time", f"{state.get('t', 0.0):6.1f} s"),
        ("travelled", f"{state.get('distance', 0.0):6.2f} m"),
        ("gripper", state.get("held") or "empty"),
    ):
        d.text((20, y), label, font=fs, fill=DIM)
        d.text((150, y), str(value), font=fmb, fill=FG)
        y += 20
    y += 8
    d.line((20, y, PANEL_W - 20, y), fill=(52, 56, 66), width=1)
    y += 12

    cur = state.get("current")  # (cmd, idx)
    done = state.get("done", {})  # (cmd, idx) -> bool success

    for ci, actions in enumerate(state.get("commands", [])):
        col = cmd_color(ci)
        d.rectangle((20, y + 2, 26, y + 15), fill=col)
        d.text((34, y), f"command {ci}", font=fsb, fill=col)
        y += CMD_H
        for ai, label in enumerate(actions):
            key = (ci, ai)
            is_cur = cur == key
            if is_cur:
                _rounded(d, (16, y - 2, PANEL_W - 14, y + 16), 4, col + (70,))
                d.rectangle((16, y - 2, 19, y + 16), fill=col)
            if key in done:
                mark, mc = ("✓", OK) if done[key] else ("✗", BAD)
            elif is_cur:
                mark, mc = ("▸", FG)
            else:
                mark, mc = ("·", DIM)
            d.text((34, y), mark, font=fm, fill=mc)
            fill = FG if (is_cur or key in done) else DIM
            d.text((54, y), label[:34], font=fmb if is_cur else fm, fill=fill)
            y += ROW
        y += CMD_GAP

    leaf = state.get("leaf_line", "")
    if leaf:
        d.line((20, H - 66, PANEL_W - 20, H - 66), fill=(52, 56, 66), width=1)
        d.text((20, H - 54), "active leaf", font=fs, fill=DIM)
        d.text((20, H - 34), leaf, font=fmb, fill=FG)
    return out
