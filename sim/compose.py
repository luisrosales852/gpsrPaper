"""Merged-vs-sequential "race" video: one 1080p frame, one shared clock.

Reads the two per-mode recordings (``run.mp4`` + ``run.json``) that
``run_sim.run_once`` wrote and composes them into a single side-by-side video:

* a header naming the claim (same commands, same robot, same tree — only the
  ordering differs);
* both plan views with their own clock / distance / current-action caption;
* a Gantt "race" timeline on a shared time axis, one bar per mode, coloured by
  source command, light where the robot is driving (``go_to``) and solid where
  it is working — the interleaving and the saved driving are visible at a glance;
* a finish stamp on the winner while the other keeps going, then an end card.

Every number drawn comes from the event logs, never from pixel measurements and
never hard-coded. Nothing here schedules or executes anything.
"""

import json
import os
import subprocess
from typing import List, Optional, Tuple

import numpy as np
from PIL import Image, ImageDraw

from sim.overlay import BG, DIM, FG, OK, PANEL_W, cmd_color, font

W, H = 1920, 1080
HEADER_H = 92
CAPTION_H = 64
PANE_TOP = HEADER_H + CAPTION_H
PANE_H = 720
FOOTER_TOP = PANE_TOP + PANE_H
GAP = 40
END_CARD_S = 4.5          # real seconds the end card holds
DRIVE_SHADE = 0.5         # go_to segments: command colour at half strength
LINE = (52, 56, 66)
MODES = ("merged", "sequential")
TITLES = {"merged": "MERGED", "sequential": "SEQUENTIAL"}
SUBTITLES = {"merged": "interleaved by the merger", "sequential": "one command after another"}


def fmt_t(t: float) -> str:
    m, s = divmod(max(0.0, t), 60.0)
    return f"{int(m):02d}:{s:04.1f}"


def _blend(c: Tuple[int, int, int], a: float, bg=BG) -> Tuple[int, int, int]:
    return tuple(int(bg[i] + (c[i] - bg[i]) * a) for i in range(3))


class Run:
    """One mode's recording: frame decoder plus the event log it was cut from."""

    def __init__(self, mode: str, run_dir: str):
        self.mode = mode
        log = json.load(open(os.path.join(run_dir, "run.json")))
        self.summary = log["summary"]
        self.labels: List[List[str]] = log["labels"]
        self.frame_t = [f[0] for f in log["frames"]]
        self.frame_d = [f[1] for f in log["frames"]]
        ev = log["events"]
        self.finish_t = max(e["t"] for e in ev if e["phase"] == "end")
        self.finish_d = max(e["distance_m"] for e in ev if e["phase"] == "end")
        # Pair start/end: leaves are synchronous, so events strictly alternate.
        self.segments = []  # (t0, t1, cmd, idx, kind)
        open_ev = None
        for e in ev:
            if e["phase"] == "start":
                open_ev = e
            elif open_ev is not None:
                self.segments.append((open_ev["t"], e["t"], e["cmd"], e["idx"], e["kind"]))
                open_ev = None

        path = os.path.join(run_dir, "run.mp4")
        probe = subprocess.run(
            ["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
             "stream=width,height", "-of", "csv=p=0", path],
            capture_output=True, text=True, check=True).stdout.strip().split(",")
        self.src_w, self.src_h = int(probe[0]), int(probe[1])
        self._proc = subprocess.Popen(
            ["ffmpeg", "-loglevel", "error", "-i", path, "-f", "rawvideo",
             "-pix_fmt", "rgb24", "-"], stdout=subprocess.PIPE)
        self._idx = -1
        self._img: Optional[Image.Image] = None

    def _read(self) -> bool:
        n = self.src_w * self.src_h * 3
        buf = self._proc.stdout.read(n)
        if len(buf) < n:
            return False
        arr = np.frombuffer(buf, np.uint8).reshape(self.src_h, self.src_w, 3)
        self._img = Image.fromarray(arr[:, PANEL_W:])   # drop the per-run HUD
        self._idx += 1
        return True

    def frame_at(self, t: float) -> Image.Image:
        """Latest recorded frame whose sim-time is <= t (holds the last one)."""
        while self._idx + 1 < len(self.frame_t) and self.frame_t[self._idx + 1] <= t + 1e-6:
            if not self._read():
                break
        if self._img is None:
            self._read()
        return self._img

    def distance_at(self, t: float) -> float:
        if t >= self.finish_t:
            return self.finish_d
        i = max(0, min(self._idx, len(self.frame_d) - 1))
        return self.frame_d[i]

    def action_at(self, t: float) -> Optional[Tuple[int, int, str]]:
        for (t0, t1, c, i, _k) in self.segments:
            if t0 <= t < t1:
                return (c, i, self.labels[c][i])
        return None

    def close(self) -> None:
        self._proc.stdout.close()
        self._proc.wait()


def _command_summary(labels: List[str]) -> str:
    """'kitchen → garage' from a command's go_to labels."""
    stops = [lb[len("go_to("):-1] for lb in labels if lb.startswith("go_to(")]
    return " → ".join(stops) if stops else labels[0]


class Composer:
    def __init__(self, runs: dict, scenario: str, fps: int, speed: float):
        self.runs = runs
        self.scenario = scenario
        self.fps, self.speed = fps, speed
        m, s = runs["merged"], runs["sequential"]
        self.t_end = max(m.finish_t, s.finish_t)
        self.t_axis = self.t_end * 1.08   # room for the finish label

        pw = runs["merged"].src_w - PANEL_W
        ph = runs["merged"].src_h
        scale = min(PANE_H / ph, (W - 3 * GAP) / 2 / pw)
        self.pane_w, self.pane_h = int(pw * scale), int(ph * scale)
        x0 = (W - 2 * self.pane_w - GAP) // 2
        self.pane_x = {"merged": x0, "sequential": x0 + self.pane_w + GAP}
        self.pane_y = PANE_TOP + (PANE_H - self.pane_h) // 2

        self.bar_l = x0 + 150
        self.bar_r = W - x0
        self.static = self._static_layer()

    # ---- layers ----------------------------------------------------------

    def _static_layer(self) -> Image.Image:
        img = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(img)
        x0 = self.pane_x["merged"]
        d.text((x0, 18), "GPSR multi-command merging", font=font("DejaVuSans-Bold.ttf", 30), fill=FG)
        n = len(self.runs["merged"].labels)
        d.text((x0, 56), f"Same {n} spoken commands · same robot · same behaviour tree — "
                         f"only the execution order differs",
               font=font("DejaVuSans.ttf", 17), fill=DIM)
        f = font("DejaVuSansMono.ttf", 15)
        label = f"scenario: {self.scenario}   ·   video {self.speed:g}× real time"
        d.text((W - x0 - d.textlength(label, font=f), 62), label, font=f, fill=DIM)
        d.line((x0, HEADER_H - 4, W - x0, HEADER_H - 4), fill=LINE, width=1)

        # Legend: one chip per command, coloured as in the plan views.
        y = FOOTER_TOP + 16
        x = x0
        fl = font("DejaVuSans.ttf", 15)
        flb = font("DejaVuSans-Bold.ttf", 15)
        for ci, labels in enumerate(self.runs["merged"].labels):
            col = cmd_color(ci)
            d.rounded_rectangle((x, y + 2, x + 14, y + 16), radius=3, fill=col)
            head = f"command {ci}"
            d.text((x + 22, y), head, font=flb, fill=col)
            tx = x + 22 + d.textlength(head, font=flb) + 8
            body = _command_summary(labels)
            d.text((tx, y), body, font=fl, fill=FG)
            x = tx + d.textlength(body, font=fl) + 34
        # Shade key, right-aligned.
        kx = W - x0
        for text, a in (("working", 1.0), ("driving", DRIVE_SHADE)):
            tw = d.textlength(text, font=fl)
            kx -= tw
            d.text((kx, y), text, font=fl, fill=DIM)
            kx -= 26
            d.rectangle((kx, y + 3, kx + 18, y + 15), fill=_blend((225, 225, 230), a))
            kx -= 18
        return img

    def _bar_x(self, t: float) -> float:
        return self.bar_l + (self.bar_r - self.bar_l) * min(t, self.t_axis) / self.t_axis

    def _draw_timeline(self, d: ImageDraw.ImageDraw, t: float) -> None:
        x0 = self.pane_x["merged"]
        fb = font("DejaVuSans-Bold.ttf", 17)
        fm = font("DejaVuSansMono-Bold.ttf", 16)
        rows = {"merged": FOOTER_TOP + 56, "sequential": FOOTER_TOP + 104}
        bh = 34
        for mode, y in rows.items():
            run = self.runs[mode]
            d.text((x0, y + 7), TITLES[mode], font=fb, fill=FG)
            d.rectangle((self.bar_l, y, self.bar_r, y + bh), fill=(32, 35, 43))
            for (t0, t1, c, _i, kind) in run.segments:
                if t0 >= t:
                    break
                a = DRIVE_SHADE if kind == "go_to" else 1.0
                xa, xb = self._bar_x(t0), self._bar_x(min(t1, t))
                if xb - xa >= 0.5:
                    d.rectangle((xa, y, xb, y + bh), fill=_blend(cmd_color(c), a))
            if t >= run.finish_t:
                fx = self._bar_x(run.finish_t)
                d.line((fx, y - 4, fx, y + bh + 4), fill=FG, width=3)
                win = mode == min(MODES, key=lambda m: self.runs[m].finish_t)
                d.text((fx + 10, y + 7), f"✓ {fmt_t(run.finish_t)}", font=fm,
                       fill=OK if win else FG)

        # Axis ticks every 30 s of sim time.
        ya = rows["sequential"] + bh + 10
        fs = font("DejaVuSansMono.ttf", 13)
        tick = 0.0
        while tick <= self.t_axis:
            x = self._bar_x(tick)
            d.line((x, ya, x, ya + 5), fill=DIM)
            lb = fmt_t(tick)[:-2]
            d.text((x - d.textlength(lb, font=fs) / 2, ya + 7), lb, font=fs, fill=DIM)
            tick += 30.0
        # Playhead.
        if t < self.t_end:
            px = self._bar_x(t)
            d.line((px, rows["merged"] - 8, px, rows["sequential"] + bh + 6),
                   fill=(255, 255, 255), width=2)

    def _draw_pane(self, img: Image.Image, d: ImageDraw.ImageDraw, mode: str, t: float) -> None:
        run = self.runs[mode]
        x, y = self.pane_x[mode], self.pane_y
        done = t >= run.finish_t
        # Past the finish this holds the recording's last frame, which comes
        # after run_once clears the speech bubble — a clean trail for the still.
        frame = run.frame_at(t)
        if frame.size != (self.pane_w, self.pane_h):
            frame = frame.resize((self.pane_w, self.pane_h), Image.LANCZOS)
        img.paste(frame, (x, y))

        # Caption above the pane.
        cy = HEADER_H + 8
        d.text((x, cy), TITLES[mode], font=font("DejaVuSans-Bold.ttf", 24), fill=FG)
        d.text((x, cy + 32), SUBTITLES[mode], font=font("DejaVuSans.ttf", 15), fill=DIM)
        clock = fmt_t(min(t, run.finish_t))
        fc = font("DejaVuSansMono-Bold.ttf", 34)
        cw = d.textlength(clock, font=fc)
        d.text((x + self.pane_w - cw, cy - 2), clock, font=fc, fill=OK if done else FG)
        dist = f"{run.distance_at(t):5.1f} m driven"
        fd = font("DejaVuSansMono.ttf", 16)
        d.text((x + self.pane_w - cw - 24 - d.textlength(dist, font=fd), cy + 14), dist,
               font=fd, fill=DIM)

        # Current action chip on the pane's bottom edge.
        act = None if done else run.action_at(t)
        if act is not None:
            c, _i, label = act
            f = font("DejaVuSansMono-Bold.ttf", 16)
            text = f"▸ cmd {c} · {label}"
            tw = d.textlength(text, font=f)
            bx, by = x + 14, y + self.pane_h - 44
            d.rounded_rectangle((bx, by, bx + tw + 24, by + 30), radius=8,
                                fill=(20, 22, 28, 220), outline=cmd_color(c) + (255,), width=2)
            d.text((bx + 12, by + 5), text, font=f, fill=FG)

        if done:
            # Stamp on the pane's top edge (over the wall), so it never hides the trail.
            stamp = f"✓ DONE  {fmt_t(run.finish_t)}"
            f = font("DejaVuSans-Bold.ttf", 30)
            tw = d.textlength(stamp, font=f)
            sx, sy = x + (self.pane_w - tw) / 2, y + 10
            d.rounded_rectangle((sx - 22, sy - 8, sx + tw + 22, sy + 44), radius=12,
                                fill=(16, 18, 24, 235), outline=OK + (255,), width=3)
            d.text((sx, sy), stamp, font=f, fill=OK)

    def _draw_end_card(self, img: Image.Image, alpha: float) -> None:
        m, s = self.runs["merged"], self.runs["sequential"]
        dt = s.finish_t - m.finish_t
        dd = s.finish_d - m.finish_d
        pt = 100.0 * dt / s.finish_t if s.finish_t else 0.0
        pd = 100.0 * dd / s.finish_d if s.finish_d else 0.0
        layer = Image.new("RGBA", (W, H), (0, 0, 0, 0))
        d = ImageDraw.Draw(layer)
        d.rectangle((0, PANE_TOP, W, FOOTER_TOP), fill=(8, 9, 12, 130))
        cw, ch = 980, 270
        cx, cy = (W - cw) // 2, PANE_TOP + (PANE_H - ch) // 2
        d.rounded_rectangle((cx, cy, cx + cw, cy + ch), radius=22,
                            fill=(16, 18, 24, 240), outline=OK + (255,), width=3)
        lines = [
            (f"Merged finished {dt:.1f} s sooner", font("DejaVuSans-Bold.ttf", 46), FG),
            (f"{pt:.0f}% less time   ·   {dd:.1f} m less driving ({pd:.0f}%)",
             font("DejaVuSans-Bold.ttf", 28), OK),
            (f"merged {fmt_t(m.finish_t)} / {m.finish_d:.1f} m      "
             f"sequential {fmt_t(s.finish_t)} / {s.finish_d:.1f} m",
             font("DejaVuSansMono.ttf", 20), DIM),
            ("same actions, same outcomes: every action succeeded in both runs"
             if all(r.summary["actions_succeeded"] == r.summary["actions_executed"]
                    for r in (m, s)) else "",
             font("DejaVuSans.ttf", 17), DIM),
        ]
        y = cy + 40
        for text, f, col in lines:
            if text:
                tw = d.textlength(text, font=f)
                d.text(((W - tw) / 2, y), text, font=f, fill=col)
            y += {46: 76, 28: 58, 20: 46}.get(f.size, 40)
        if alpha < 1.0:
            layer.putalpha(layer.getchannel("A").point(lambda v: int(v * alpha)))
        img.paste(layer, (0, 0), layer)

    def frame(self, t: float, card_alpha: float = 0.0) -> Image.Image:
        img = self.static.copy()
        d = ImageDraw.Draw(img, "RGBA")
        for mode in MODES:
            self._draw_pane(img, d, mode, t)
        self._draw_timeline(d, t)
        if card_alpha > 0.0:
            self._draw_end_card(img, card_alpha)
        return img


def compose(out_dir: str, scenario: str, fps: int, speed: float) -> str:
    """Write ``side_by_side.mp4`` (+ a ``side_by_side.png`` still) into ``out_dir``."""
    runs = {m: Run(m, os.path.join(out_dir, m)) for m in MODES}
    comp = Composer(runs, scenario, fps, speed)
    path = os.path.join(out_dir, "side_by_side.mp4")
    proc = subprocess.Popen([
        "ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pixel_format", "rgb24",
        "-video_size", f"{W}x{H}", "-framerate", str(fps), "-i", "-",
        "-c:v", "libx264", "-preset", "medium", "-crf", "19", "-pix_fmt", "yuv420p", path,
    ], stdin=subprocess.PIPE)

    dt = speed / fps
    n_race = int(comp.t_end / dt) + 2
    n_hold = int(0.8 * fps)
    n_card = int(END_CARD_S * fps)
    last = None
    for k in range(n_race + n_hold + n_card):
        # Uncapped: during the hold the panes reach each recording's post-finish
        # frames (bubble cleared); clocks and bars clamp to finish_t themselves.
        t = k * dt
        ck = k - n_race - n_hold
        alpha = 0.0 if ck < 0 else min(1.0, (ck + 1) / (0.5 * fps))
        img = comp.frame(t, alpha)
        if k == n_race + n_hold - 1:
            img.save(os.path.join(out_dir, "side_by_side.png"))
        proc.stdin.write(np.asarray(img, dtype=np.uint8).tobytes())
        last = img
    proc.stdin.close()
    proc.wait()
    if last is not None:
        last.save(os.path.join(out_dir, "side_by_side_endcard.png"))
    for r in runs.values():
        r.close()
    return path
