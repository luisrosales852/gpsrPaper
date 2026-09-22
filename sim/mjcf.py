"""Generate the MuJoCo MJCF XML for an apartment ``World``.

The robot is a **cube** placeholder driven kinematically (slide x, slide y,
hinge z). Nothing here is dynamic: no actuators, no contacts that matter. See
``sim/README`` notes for swapping the cube for a real mesh.
"""

import math
from typing import List, Tuple

from sim.world import World

ROBOT_HALF = (0.22, 0.22, 0.26)
ROBOT_Z = 0.26
GRIPPER_LOCAL = (0.30, 0.0, 0.20)


def _rgba(c) -> str:
    return " ".join(f"{v:.3f}" for v in c)


def _rect_to_box(rect, h: float, z0: float = 0.0) -> Tuple[str, str]:
    """(x0,y0,x1,y1) + height -> MuJoCo ``pos`` and ``size`` attribute text."""
    x0, y0, x1, y1 = rect
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    hx, hy = abs(x1 - x0) / 2.0, abs(y1 - y0) / 2.0
    return f"{cx:.4f} {cy:.4f} {z0 + h / 2.0:.4f}", f"{hx:.4f} {hy:.4f} {h / 2.0:.4f}"


def _topdown_camera(world: World, width: int, height: int) -> str:
    """A fixed overhead camera framing the whole apartment with a margin."""
    (x0, y0), (x1, y1) = world.bounds
    margin = 0.45
    half_x = abs(x1 - x0) / 2.0 + margin
    half_y = abs(y1 - y0) / 2.0 + margin
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0

    aspect = width / float(height)
    # Vertical FOV must cover half_y directly, and half_x once divided by the
    # aspect ratio, so the binding constraint is whichever is larger.
    need = max(half_y, half_x / aspect)
    h = 15.0
    fovy = math.degrees(2.0 * math.atan(need / h))
    # quat "1 0 0 0" -> camera -z along world -z, camera +y along world +y,
    # i.e. a north-up plan view.
    return (
        f'<camera name="topdown" mode="fixed" pos="{cx:.3f} {cy:.3f} {h:.3f}" '
        f'quat="1 0 0 0" fovy="{fovy:.3f}"/>'
    )


def _iso_camera(world: World, eye=(11.0, -13.0, 11.0)) -> str:
    """Three-quarter view, aimed at the apartment centre."""
    (x0, y0), (x1, y1) = world.bounds
    target = ((x0 + x1) / 2.0, (y0 + y1) / 2.0, 0.6)

    def sub(a, b):
        return [a[i] - b[i] for i in range(3)]

    def cross(a, b):
        return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]

    def unit(v):
        n = math.sqrt(sum(c * c for c in v)) or 1.0
        return [c / n for c in v]

    fwd = unit(sub(target, eye))
    right = unit(cross(fwd, [0.0, 0.0, 1.0]))
    up = cross(right, fwd)
    xy = " ".join(f"{v:.5f}" for v in right + up)
    return (
        f'<camera name="iso" mode="fixed" pos="{eye[0]:.3f} {eye[1]:.3f} {eye[2]:.3f}" '
        f'xyaxes="{xy}" fovy="45"/>'
    )


def build_xml(world: World, width: int = 920, height: int = 720) -> str:
    (bx0, by0), (bx1, by1) = world.bounds
    wall_h = world.wall_height
    t = 0.12  # wall thickness

    parts: List[str] = []
    parts.append(
        f"""<mujoco model="gpsr_apartment">
  <compiler angle="radian"/>
  <option timestep="0.0166667" gravity="0 0 0"/>
  <visual>
    <global offwidth="1920" offheight="1080"/>
    <quality shadowsize="4096" offsamples="8"/>
    <headlight ambient="0.28 0.28 0.30" diffuse="0.28 0.28 0.28" specular="0.05 0.05 0.05"/>
    <map znear="0.05" zfar="60"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.32 0.38 0.48"
             rgb2="0.10 0.12 0.16" width="512" height="512"/>
    <material name="floor_mat" rgba="0.52 0.53 0.56 1" reflectance="0.0"/>
  </asset>
  <worldbody>
    <light name="key"  pos="0 0 9"   dir="0 0 -1" directional="true" diffuse="0.38 0.38 0.38"/>
    <light name="fill" pos="6 -8 7"  dir="-0.5 0.7 -0.8" directional="true" diffuse="0.18 0.18 0.20"/>
    <geom name="floor" type="plane" pos="0 0 0" size="{abs(bx1 - bx0) / 2 + 1.5:.3f} {abs(by1 - by0) / 2 + 1.5:.3f} 0.1" material="floor_mat"/>"""
    )

    # Zone floor patches — thin colored slabs, purely visual.
    for z in world.zones:
        pos, size = _rect_to_box(z["rect"], 0.02, z0=0.005)
        parts.append(
            f'    <geom name="zone_{z["name"]}" type="box" pos="{pos}" size="{size}" '
            f'rgba="{_rgba(z["color"])}" contype="0" conaffinity="0"/>'
        )

    # Outer walls.
    cx, cy = (bx0 + bx1) / 2.0, (by0 + by1) / 2.0
    for nm, rect in (
        ("wall_s", (bx0, by0 - t, bx1, by0)),
        ("wall_n", (bx0, by1, bx1, by1 + t)),
        ("wall_w", (bx0 - t, by0 - t, bx0, by1 + t)),
        ("wall_e", (bx1, by0 - t, bx1 + t, by1 + t)),
    ):
        pos, size = _rect_to_box(rect, wall_h)
        parts.append(
            f'    <geom name="{nm}" type="box" pos="{pos}" size="{size}" '
            f'rgba="0.55 0.56 0.60 1" contype="0" conaffinity="0"/>'
        )

    # Furniture islands (no interior doors — the plan stays open).
    for fdef in world.furniture:
        pos, size = _rect_to_box(fdef["rect"], float(fdef["h"]))
        parts.append(
            f'    <geom name="furn_{fdef["name"]}" type="box" pos="{pos}" size="{size}" '
            f'rgba="{_rgba(fdef["color"])}" contype="0" conaffinity="0"/>'
        )

    # Robot: the cube placeholder.
    hx, hy, hz = ROBOT_HALF
    gx, gy, gz = GRIPPER_LOCAL
    parts.append(
        f"""
    <body name="robot" pos="0 0 {ROBOT_Z}">
      <joint name="rob_x" type="slide" axis="1 0 0"/>
      <joint name="rob_y" type="slide" axis="0 1 0"/>
      <joint name="rob_yaw" type="hinge" axis="0 0 1"/>
      <geom name="robot_shell" type="box" size="{hx} {hy} {hz}" rgba="0.25 0.28 0.34 1"
            contype="0" conaffinity="0"/>
      <geom name="robot_nose" type="box" pos="{hx + 0.06} 0 0.05" size="0.07 0.10 0.05"
            rgba="0.95 0.95 0.98 1" contype="0" conaffinity="0"/>
      <geom name="robot_mast" type="cylinder" pos="0 0 {hz + 0.14}" size="0.05 0.14"
            rgba="0.30 0.33 0.40 1" contype="0" conaffinity="0"/>
      <geom name="robot_gripper" type="box" pos="{gx} 0 {gz}" size="0.05 0.09 0.03"
            rgba="0.90 0.72 0.20 1" contype="0" conaffinity="0"/>
      <site name="gripper" pos="{gx} 0 {gz}" size="0.02"/>
    </body>"""
    )

    # Manipulable objects — freejoint so the gripper can carry them.
    for o in world.objects:
        parts.append(
            f"""
    <body name="{o.body}" pos="{o.home[0]:.4f} {o.home[1]:.4f} {o.home[2]:.4f}">
      <freejoint name="{o.body}_free"/>
      <geom name="{o.body}_g" type="sphere" size="{o.size:.4f}" rgba="{_rgba(o.color)}"
            contype="0" conaffinity="0"/>
    </body>"""
        )

    # People — kinematic too, so follow/guide are exactly reproducible.
    for p in world.people:
        parts.append(
            f"""
    <body name="{p.body}" pos="{p.home[0]:.4f} {p.home[1]:.4f} 0">
      <joint name="{p.body}_x" type="slide" axis="1 0 0"/>
      <joint name="{p.body}_y" type="slide" axis="0 1 0"/>
      <joint name="{p.body}_yaw" type="hinge" axis="0 0 1"/>
      <geom name="{p.body}_legs" type="capsule" fromto="0 0 0.08 0 0 0.62" size="0.11"
            rgba="0.22 0.24 0.30 1" contype="0" conaffinity="0"/>
      <geom name="{p.body}_shirt" type="capsule" fromto="0 0 0.62 0 0 1.30" size="0.17"
            rgba="{_rgba(p.shirt)}" contype="0" conaffinity="0"/>
      <geom name="{p.body}_head" type="sphere" pos="0 0 1.50" size="0.13"
            rgba="0.85 0.72 0.60 1" contype="0" conaffinity="0"/>
      <geom name="{p.body}_face" type="sphere" pos="0.10 0 1.52" size="0.05"
            rgba="0.30 0.25 0.22 1" contype="0" conaffinity="0"/>
    </body>"""
        )

    parts.append("    " + _topdown_camera(world, width, height))
    parts.append("    " + _iso_camera(world))
    parts.append("  </worldbody>\n</mujoco>")
    return "\n".join(parts)
