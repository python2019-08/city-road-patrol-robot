#!/usr/bin/env python3
"""Generate ``city_road.world.sdf`` for the city-road patrol / on-ramp scene.

The layout follows ``doc/0.req/0-req02.png`` and ``doc/0.req/0-req03.png``:

  * a straight one-way carriageway with four lanes
      超车道 | 行车道 | 行车道 | 应急车道
  * the patrol robot drives along the emergency lane;
  * an on-ramp joins the emergency lane from the right at a shallow angle;
  * the merge point is the manual-takeover / "通过点"; 50 m later the robot
    resumes autonomous driving.

Coordinate system
-----------------
  +X : direction of travel (patrol direction)
  +Y : left of travel
  +Z : up
  origin: patrol start point, on the centreline of the emergency lane.
          The merge point is therefore at x = 140 m, and the resume point
          at x = 190 m.

The script is intentionally dependency-free (only the Python standard
library).  Sign textures are generated with Pillow when it is available;
otherwise the existing PNGs under ``models/city_road/textures`` are used.

Usage
-----
    python3 src/patrolbot_description/world/generate_city_road_world.py
    gz sdf -k src/patrolbot_description/world/city_road.world.sdf
"""

from __future__ import annotations

import math
import textwrap
from pathlib import Path
from xml.dom import minidom

HERE = Path(__file__).resolve().parent
PACKAGE_ROOT = HERE.parent
TEXTURE_DIR = PACKAGE_ROOT / "models" / "city_road" / "textures"
OUT = HERE / "city_road.world.sdf"

# ---------------------------------------------------------------------------
# Road dimensions (metres).  Standard Chinese expressway lane = 3.75 m;
# the emergency lane is 3.0 m.  These are placeholders: the requirements do
# not give a survey drawing, so keep them in one place and change them here.
# ---------------------------------------------------------------------------
LANE_W = 3.75
EMERG_W = 3.0
SHOULDER_W = 0.60

ROAD_X0 = -40.0          # 40 m behind the patrol start
ROAD_X1 = 320.0          # 180 m beyond the resume point
ROAD_LEN = ROAD_X1 - ROAD_X0
ROAD_CX = 0.5 * (ROAD_X0 + ROAD_X1)

EMERG_Y0 = -0.5 * EMERG_W       # -1.5
EMERG_Y1 = +0.5 * EMERG_W       # +1.5
EMERG_CY = 0.0

DRIVE1_Y0, DRIVE1_Y1 = EMERG_Y1, EMERG_Y1 + LANE_W          # 1.5 .. 5.25
DRIVE2_Y0, DRIVE2_Y1 = DRIVE1_Y1, DRIVE1_Y1 + LANE_W        # 5.25 .. 9.0
OVERTAKE_Y0, OVERTAKE_Y1 = DRIVE2_Y1, DRIVE2_Y1 + LANE_W    # 9.0 .. 12.75

ROAD_Y0 = EMERG_Y0 - SHOULDER_W  # -2.1
ROAD_Y1 = OVERTAKE_Y1            # 12.75
ROAD_W = ROAD_Y1 - ROAD_Y0
ROAD_CY = 0.5 * (ROAD_Y0 + ROAD_Y1)

MERGE_X = 140.0
RESUME_X = MERGE_X + 50.0

LANE_CENTERS = {
    "emergency": 0.5 * (EMERG_Y0 + EMERG_Y1),           # 0.0
    "drive1": 0.5 * (DRIVE1_Y0 + DRIVE1_Y1),            # 3.375
    "drive2": 0.5 * (DRIVE2_Y0 + DRIVE2_Y1),            # 7.125
    "overtake": 0.5 * (OVERTAKE_Y0 + OVERTAKE_Y1),      # 10.875
}

MARK_W = 0.15
MARK_Z = 0.008
DASH_ON = 6.0
DASH_OFF = 9.0

# On-ramp centreline: cubic Bezier, tangent 20 deg at the start and tangent
# 0 deg at the merge.  The last part therefore runs parallel to the mainline:
# a schematic "acceleration / merge" section.
RAMP_W = 3.5
RAMP_P0 = (10.0, -48.0)
RAMP_P1 = (57.0, -30.9)
RAMP_P2 = (110.0, 0.0)
RAMP_P3 = (MERGE_X, 0.0)
RAMP_SAMPLES = 60

# ---------------------------------------------------------------------------
# Colours
# ---------------------------------------------------------------------------
C_ASPHALT = (0.12, 0.12, 0.13)
C_EMERGENCY = (0.30, 0.43, 0.49)
C_WHITE = (0.93, 0.93, 0.90)
C_YELLOW = (0.92, 0.76, 0.18)
C_GREEN = (0.10, 0.42, 0.18)
C_GRASS = (0.16, 0.30, 0.10)
C_STEEL = (0.55, 0.57, 0.60)
C_POST = (0.40, 0.42, 0.44)
C_DARK = (0.18, 0.19, 0.20)
C_TRUNK = (0.28, 0.18, 0.10)
C_LEAF = (0.10, 0.34, 0.10)
C_LAMP = (1.00, 0.92, 0.65)

BUILDING_COLOURS = [
    (0.55, 0.56, 0.58),
    (0.62, 0.60, 0.54),
    (0.48, 0.53, 0.58),
    (0.66, 0.63, 0.60),
    (0.52, 0.50, 0.48),
]


# ---------------------------------------------------------------------------
# Small XML helpers
# ---------------------------------------------------------------------------
def f(value: float) -> str:
    """Compact, stable number formatting."""
    value = 0.0 if abs(value) < 1e-12 else value
    return f"{value:.4f}".rstrip("0").rstrip(".")


def pose(x: float, y: float, z: float,
         roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> str:
    return f"{f(x)} {f(y)} {f(z)} {f(roll)} {f(pitch)} {f(yaw)}"


def box_size(sx: float, sy: float, sz: float) -> str:
    return f"{f(sx)} {f(sy)} {f(sz)}"


def material(rgb: tuple[float, float, float],
             specular: float = 0.05,
             emissive: tuple[float, float, float] | None = None) -> str:
    r, g, b = rgb
    em = ""
    if emissive is not None:
        er, eg, eb = emissive
        em = f"\n    <emissive>{f(er)} {f(eg)} {f(eb)} 1</emissive>"
    return textwrap.dedent(f"""\
        <material>
          <ambient>{f(r)} {f(g)} {f(b)} 1</ambient>
          <diffuse>{f(r)} {f(g)} {f(b)} 1</diffuse>
          <specular>{f(specular)} {f(specular)} {f(specular)} 1</specular>{em}
        </material>""").strip()


def texture_material(uri: str) -> str:
    # SDF 1.7's metal PBR workflow has albedo_map but no scalar albedo
    # element, so the fallback colour is carried by the texture itself.
    return textwrap.dedent(f"""\
        <material>
          <pbr>
            <metal>
              <albedo_map>{uri}</albedo_map>
              <roughness>0.55</roughness>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
        </material>""").strip()


def collision_box(name: str, size: str, p: str, mu: float = 1.0) -> str:
    # Collisions and visuals share one name space inside a <link>; the
    # suffix keeps a collision from clashing with its visual twin.
    return textwrap.dedent(f"""\
        <collision name="{name}_collision">
          <pose>{p}</pose>
          <geometry>
            <box><size>{size}</size></box>
          </geometry>
          <surface>
            <friction>
              <ode><mu>{f(mu)}</mu><mu2>{f(mu)}</mu2></ode>
            </friction>
          </surface>
        </collision>""")


def collision_cylinder(name: str, radius: float, length: float, p: str) -> str:
    return textwrap.dedent(f"""\
        <collision name="{name}_collision">
          <pose>{p}</pose>
          <geometry>
            <cylinder><radius>{f(radius)}</radius><length>{f(length)}</length></cylinder>
          </geometry>
        </collision>""")


def visual_box(name: str, size: str, p: str, mat: str,
               cast_shadows: bool = True) -> str:
    return textwrap.dedent(f"""\
        <visual name="{name}">
          <pose>{p}</pose>
          <cast_shadows>{str(cast_shadows).lower()}</cast_shadows>
          <geometry>
            <box><size>{size}</size></box>
          </geometry>
          {material_or_texture(mat)}
        </visual>""")


def visual_cylinder(name: str, radius: float, length: float, p: str,
                    mat: str, cast_shadows: bool = True) -> str:
    return textwrap.dedent(f"""\
        <visual name="{name}">
          <pose>{p}</pose>
          <cast_shadows>{str(cast_shadows).lower()}</cast_shadows>
          <geometry>
            <cylinder><radius>{f(radius)}</radius><length>{f(length)}</length></cylinder>
          </geometry>
          {material_or_texture(mat)}
        </visual>""")


def visual_sphere(name: str, radius: float, p: str, mat: str,
                  cast_shadows: bool = True) -> str:
    return textwrap.dedent(f"""\
        <visual name="{name}">
          <pose>{p}</pose>
          <cast_shadows>{str(cast_shadows).lower()}</cast_shadows>
          <geometry>
            <sphere><radius>{f(radius)}</radius></sphere>
          </geometry>
          {material_or_texture(mat)}
        </visual>""")


def material_or_texture(mat: str) -> str:
    """``mat`` is either a complete <material> block or a texture URI."""
    if mat.startswith("<material>"):
        return mat
    return texture_material(mat)


def indent(text: str, spaces: int) -> str:
    pad = " " * spaces
    return "\n".join(pad + line if line.strip() else line
                     for line in text.splitlines())


def make_link(name: str,
              collisions: list[str] | None = None,
              visuals: list[str] | None = None) -> str:
    body = []
    for c in collisions or []:
        body.append(c)
    for v in visuals or []:
        body.append(v)
    body_xml = "\n".join(body)
    if body_xml:
        body_xml = "\n" + indent(body_xml, 2) + "\n"
    return f'<link name="{name}">{body_xml}</link>'


def make_model(name: str, link: str, include_pose: str | None = None) -> str:
    pose_xml = f"\n  <pose>{include_pose}</pose>" if include_pose else ""
    return textwrap.dedent(f"""\
        <model name="{name}">
          <static>true</static>{pose_xml}
        {indent(link, 2)}
        </model>""")


# ---------------------------------------------------------------------------
# Sign textures
# ---------------------------------------------------------------------------
SIGNS = {
    "sign_overtake.png": "超车道",
    "sign_drive.png": "行车道",
    "sign_emergency.png": "应急车道",
}


def ensure_sign_textures() -> None:
    """Draw the green overhead lane signs (512 x 240 px)."""
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        print("Pillow not available; keeping existing sign textures.")
        return

    font_candidates = [
        ("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 0),
        ("/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf", 0),
    ]
    font = None
    for path, index in font_candidates:
        try:
            font = ImageFont.truetype(path, 76, index=index)
            break
        except OSError:
            continue
    if font is None:
        print("No CJK font found; keeping existing sign textures.")
        return

    TEXTURE_DIR.mkdir(parents=True, exist_ok=True)
    white = (246, 246, 240)
    green = (9, 105, 52)
    for filename, label in SIGNS.items():
        img = Image.new("RGB", (512, 240), green)
        draw = ImageDraw.Draw(img)
        draw.rectangle([8, 8, 503, 231], outline=white, width=9)
        draw.text((256, 78), label, font=font, fill=white, anchor="mm")
        # Downward arrow
        draw.rectangle([244, 140, 268, 180], fill=white)
        draw.polygon([(218, 176), (294, 176), (256, 224)], fill=white)
        img.save(TEXTURE_DIR / filename)


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------
def bezier(t: float) -> tuple[float, float]:
    mt = 1.0 - t
    x = (mt ** 3 * RAMP_P0[0] + 3 * mt ** 2 * t * RAMP_P1[0]
         + 3 * mt * t ** 2 * RAMP_P2[0] + t ** 3 * RAMP_P3[0])
    y = (mt ** 3 * RAMP_P0[1] + 3 * mt ** 2 * t * RAMP_P1[1]
         + 3 * mt * t ** 2 * RAMP_P2[1] + t ** 3 * RAMP_P3[1])
    return x, y


def ramp_samples() -> list[tuple[float, float, float, float, float, float]]:
    """Return (t0, t1, x0, y0, x1, y1) for each short ramp segment."""
    ts = [i / RAMP_SAMPLES for i in range(RAMP_SAMPLES + 1)]
    return [(ts[i], ts[i + 1], *bezier(ts[i]), *bezier(ts[i + 1]))
            for i in range(RAMP_SAMPLES)]


def build_lane_markings() -> str:
    visuals: list[str] = []
    mat_white = material(C_WHITE, specular=0.08)
    mat_yellow = material(C_YELLOW, specular=0.08)

    # Solid lines.  The two emergency-lane lines are interrupted where the
    # on-ramp merges (100 m .. 145 m); the ramp's own edge lines take over.
    for name, y, x0, x1, mat in [
        ("edge_right_a", EMERG_Y0 + 0.05, ROAD_X0, 125.0, mat_white),
        ("edge_right_b", EMERG_Y0 + 0.05, 145.0, ROAD_X1, mat_white),
        ("edge_inner_a", EMERG_Y1 - 0.05, ROAD_X0, 138.0, mat_white),
        ("edge_inner_b", EMERG_Y1 - 0.05, 145.0, ROAD_X1, mat_white),
        ("edge_left", OVERTAKE_Y1 - 0.10, ROAD_X0, ROAD_X1, mat_white),
    ]:
        cx = 0.5 * (x0 + x1)
        visuals.append(visual_box(
            name,
            box_size(x1 - x0, MARK_W, 0.002),
            pose(cx, y, MARK_Z),
            mat,
            cast_shadows=False,
        ))

    # Dashed lines between the driving lanes.
    for lane_name, y in (("drive1", DRIVE1_Y1), ("drive2", DRIVE2_Y1)):
        x = ROAD_X0
        i = 0
        while x < ROAD_X1:
            x1 = min(x + DASH_ON, ROAD_X1)
            length = x1 - x
            if length > 0.5:
                visuals.append(visual_box(
                    f"dash_{lane_name}_{i:03d}",
                    box_size(length, MARK_W, 0.002),
                    pose(0.5 * (x + x1), y, MARK_Z),
                    mat_white,
                    cast_shadows=False,
                ))
            x += DASH_ON + DASH_OFF
            i += 1

    # Stop line at the manual-takeover point (across the emergency lane).
    visuals.append(visual_box(
        "stop_line_merge",
        box_size(0.30, EMERG_W, 0.002),
        pose(MERGE_X, EMERG_CY, MARK_Z),
        mat_white,
        cast_shadows=False,
    ))

    # A yellow line on the left shoulder makes the road edge obvious.
    visuals.append(visual_box(
        "edge_left_yellow",
        box_size(ROAD_LEN, 0.10, 0.002),
        pose(ROAD_CX, OVERTAKE_Y1 - 0.22, MARK_Z + 0.001),
        mat_yellow,
        cast_shadows=False,
    ))

    return make_model("lane_markings", make_link("markings", visuals=visuals))


def build_ramp() -> str:
    visuals: list[str] = []
    mat_ramp = material(C_EMERGENCY, specular=0.04)
    mat_white = material(C_WHITE, specular=0.08)

    segs = ramp_samples()
    for i, (t0, t1, x0, y0, x1, y1) in enumerate(segs):
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue
        angle = math.atan2(dy, dx)
        mx, my = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
        # Slight overlap hides seams between the short rotated boxes.
        visuals.append(visual_box(
            f"surface_{i:03d}",
            box_size(length + 0.30, RAMP_W, 0.003),
            pose(mx, my, 0.0055, yaw=angle),
            mat_ramp,
            cast_shadows=False,
        ))

        nx, ny = -math.sin(angle), math.cos(angle)
        # Right (outer) edge line runs almost to the merge point; the left
        # (inner) edge stops early so the robot can join the mainline.
        for side, sign, t_limit in (("r", -1.0, 0.98), ("l", 1.0, 0.82)):
            if t1 > t_limit:
                continue
            ex = mx + sign * nx * (0.5 * RAMP_W - 0.10)
            ey = my + sign * ny * (0.5 * RAMP_W - 0.10)
            visuals.append(visual_box(
                f"edge_{side}_{i:03d}",
                box_size(length + 0.25, 0.15, 0.002),
                pose(ex, ey, MARK_Z + 0.001, yaw=angle),
                mat_white,
                cast_shadows=False,
            ))

    return make_model("ramp", make_link("ramp", visuals=visuals))


def add_guardrail(collisions: list[str], visuals: list[str],
                  prefix: str, x0: float, y0: float,
                  x1: float, y1: float,
                  post_spacing: float = 8.0) -> None:
    length = math.hypot(x1 - x0, y1 - y0)
    if length < 1e-3:
        return
    angle = math.atan2(y1 - y0, x1 - x0)
    mx, my = 0.5 * (x0 + x1), 0.5 * (y0 + y1)

    collisions.append(collision_box(
        f"{prefix}_rail",
        box_size(length, 0.08, 0.32),
        pose(mx, my, 0.52, yaw=angle),
    ))
    visuals.append(visual_box(
        f"{prefix}_rail",
        box_size(length, 0.08, 0.32),
        pose(mx, my, 0.52, yaw=angle),
        material(C_STEEL, specular=0.45),
    ))

    n_posts = max(1, int(length // post_spacing))
    for j in range(n_posts + 1):
        t = j / n_posts
        px, py = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
        visuals.append(visual_box(
            f"{prefix}_post_{j:03d}",
            box_size(0.10, 0.10, 0.72),
            pose(px, py, 0.36, yaw=angle),
            material(C_POST, specular=0.25),
        ))


def build_guardrails() -> str:
    collisions: list[str] = []
    visuals: list[str] = []

    # Mainline left edge.
    add_guardrail(collisions, visuals, "left", ROAD_X0, ROAD_Y1 + 0.20,
                  ROAD_X1, ROAD_Y1 + 0.20)
    # Mainline right edge, interrupted at the merge mouth.
    add_guardrail(collisions, visuals, "right_a", ROAD_X0, ROAD_Y0 - 0.10,
                  100.0, ROAD_Y0 - 0.10)
    add_guardrail(collisions, visuals, "right_b", 150.0, ROAD_Y0 - 0.10,
                  ROAD_X1, ROAD_Y0 - 0.10)

    # Ramp guardrails: outer side almost to the merge, inner side stops
    # earlier so that the robot can cross into the emergency lane.
    segs = ramp_samples()
    for side, sign, t_limit in (("r", -1.0, 0.98), ("l", 1.0, 0.70)):
        prev: tuple[float, float] | None = None
        idx = 0
        for t0, t1, x0, y0, x1, y1 in segs:
            if t1 > t_limit:
                break
            dx, dy = x1 - x0, y1 - y0
            length = math.hypot(dx, dy)
            if length < 1e-6:
                continue
            angle = math.atan2(dy, dx)
            nx, ny = -math.sin(angle), math.cos(angle)
            offset = sign * (0.5 * RAMP_W + 0.20)
            px, py = x0 + nx * offset, y0 + ny * offset
            if prev is not None:
                collisions.append(collision_box(
                    f"ramp_{side}_rail_{idx:03d}",
                    box_size(length + 0.15, 0.08, 0.30),
                    pose(0.5 * (prev[0] + px), 0.5 * (prev[1] + py),
                         0.55, yaw=angle),
                ))
                visuals.append(visual_box(
                    f"ramp_{side}_rail_{idx:03d}",
                    box_size(length + 0.15, 0.08, 0.30),
                    pose(0.5 * (prev[0] + px), 0.5 * (prev[1] + py),
                         0.55, yaw=angle),
                    material(C_STEEL, specular=0.45),
                ))
                if idx % 4 == 0:
                    visuals.append(visual_box(
                        f"ramp_{side}_post_{idx:03d}",
                        box_size(0.10, 0.10, 0.72),
                        pose(px, py, 0.36, yaw=angle),
                        material(C_POST, specular=0.25),
                    ))
            prev = (px, py)
            idx += 1

    return make_model("guardrails",
                      make_link("guardrails", collisions=collisions,
                                visuals=visuals))


def build_gantry() -> str:
    collisions: list[str] = []
    visuals: list[str] = []

    gx = 60.0
    post_y = (ROAD_Y0 - 0.40, ROAD_Y1 + 0.40)  # -2.5, 13.15
    post_h = 6.4
    for i, y in enumerate(post_y):
        collisions.append(collision_box(
            f"post_{i}",
            box_size(0.35, 0.35, post_h),
            pose(gx, y, 0.5 * post_h),
        ))
        visuals.append(visual_box(
            f"post_{i}",
            box_size(0.35, 0.35, post_h),
            pose(gx, y, 0.5 * post_h),
            material(C_STEEL, specular=0.4),
        ))

    beam_y0, beam_y1 = post_y[0], post_y[1]
    beam_cy, beam_len = 0.5 * (beam_y0 + beam_y1), beam_y1 - beam_y0
    collisions.append(collision_box(
        "beam",
        box_size(0.35, beam_len, 0.35),
        pose(gx, beam_cy, 6.20),
    ))
    visuals.append(visual_box(
        "beam",
        box_size(0.35, beam_len, 0.35),
        pose(gx, beam_cy, 6.20),
        material(C_STEEL, specular=0.4),
    ))

    signs = [
        ("overtake", "sign_overtake.png", LANE_CENTERS["overtake"]),
        ("drive2", "sign_drive.png", LANE_CENTERS["drive2"]),
        ("drive1", "sign_drive.png", LANE_CENTERS["drive1"]),
        ("emergency", "sign_emergency.png", LANE_CENTERS["emergency"]),
    ]
    for name, texture, y in signs:
        visuals.append(visual_box(
            f"sign_back_{name}",
            box_size(0.18, 3.15, 1.65),
            pose(gx, y, 5.10),
            material(C_DARK, specular=0.2),
        ))
        visuals.append(visual_box(
            f"sign_face_{name}",
            box_size(0.03, 3.00, 1.50),
            pose(gx - 0.10, y, 5.10, yaw=math.pi),
            f"model://city_road/textures/{texture}",
        ))

    return make_model("gantry",
                      make_link("gantry", collisions=collisions,
                                visuals=visuals))


def build_streetlights() -> str:
    collisions: list[str] = []
    visuals: list[str] = []
    pole_h = 8.0
    pole_r = 0.12
    y = ROAD_Y1 + 1.60

    for i, x in enumerate([-20.0, 20.0, 100.0, 180.0, 220.0, 260.0, 300.0]):
        collisions.append(collision_cylinder(
            f"pole_{i}", pole_r, pole_h, pose(x, y, 0.5 * pole_h)))
        visuals.append(visual_cylinder(
            f"pole_{i}", pole_r, pole_h, pose(x, y, 0.5 * pole_h),
            material(C_STEEL, specular=0.35)))
        visuals.append(visual_box(
            f"arm_{i}",
            box_size(0.12, 2.40, 0.12),
            pose(x, y - 1.20, pole_h - 0.25),
            material(C_STEEL, specular=0.35)))
        visuals.append(visual_box(
            f"lamp_{i}",
            box_size(0.55, 0.28, 0.12),
            pose(x, y - 2.35, pole_h - 0.32),
            material(C_LAMP, specular=0.2, emissive=(0.45, 0.38, 0.18))))

    return make_model("streetlights",
                      make_link("streetlights", collisions=collisions,
                                visuals=visuals))


def build_buildings() -> str:
    visuals: list[str] = []

    # (x, y, sx, sy, sz, colour index)
    buildings = [
        (-20.0, 42.0, 24.0, 18.0, 26.0, 0),
        (30.0, 52.0, 28.0, 22.0, 38.0, 1),
        (85.0, 45.0, 22.0, 20.0, 20.0, 2),
        (140.0, 55.0, 30.0, 24.0, 44.0, 0),
        (205.0, 46.0, 24.0, 18.0, 30.0, 3),
        (265.0, 58.0, 32.0, 26.0, 36.0, 1),
        (310.0, 44.0, 20.0, 18.0, 24.0, 4),
        (-10.0, -82.0, 26.0, 20.0, 30.0, 2),
        (50.0, -92.0, 30.0, 24.0, 42.0, 0),
        (110.0, -78.0, 22.0, 18.0, 22.0, 3),
        (175.0, -95.0, 34.0, 26.0, 48.0, 1),
        (235.0, -80.0, 24.0, 20.0, 28.0, 4),
        (295.0, -90.0, 28.0, 22.0, 34.0, 2),
    ]
    for i, (x, y, sx, sy, sz, ci) in enumerate(buildings):
        visuals.append(visual_box(
            f"building_{i}",
            box_size(sx, sy, sz),
            pose(x, y, 0.5 * sz),
            material(BUILDING_COLOURS[ci], specular=0.12),
        ))
        # A slightly darker plinth makes the buildings read as city blocks.
        visuals.append(visual_box(
            f"plinth_{i}",
            box_size(sx + 1.0, sy + 1.0, 1.0),
            pose(x, y, 0.5),
            material(C_DARK, specular=0.1),
        ))

    return make_model("buildings", make_link("buildings", visuals=visuals))


def build_trees() -> str:
    visuals: list[str] = []
    for i in range(13):
        x = -20.0 + i * 27.0
        y = ROAD_Y1 + 4.2
        visuals.append(visual_cylinder(
            f"trunk_{i}", 0.16, 2.4, pose(x, y, 1.2),
            material(C_TRUNK, specular=0.05)))
        visuals.append(visual_sphere(
            f"crown_{i}", 1.35, pose(x, y, 3.25),
            material(C_LEAF, specular=0.05)))
    return make_model("trees", make_link("trees", visuals=visuals))


def build_road_markers() -> str:
    visuals: list[str] = []
    # Flat discs on the emergency-lane centreline: the manual-takeover point
    # and the point where autonomous driving resumes (50 m later).
    visuals.append(visual_cylinder(
        "pass_point", 0.42, 0.004, pose(MERGE_X, EMERG_CY, 0.010),
        material(C_YELLOW, specular=0.2), cast_shadows=False))
    visuals.append(visual_cylinder(
        "resume_point", 0.42, 0.004, pose(RESUME_X, EMERG_CY, 0.010),
        material(C_GREEN, specular=0.2), cast_shadows=False))
    return make_model("road_markers",
                      make_link("markers", visuals=visuals))


def build_ground() -> str:
    collision = textwrap.dedent(f"""\
        <collision name="ground_collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>900 600</size></plane>
          </geometry>
          <surface>
            <friction>
              <ode><mu>1.0</mu><mu2>1.0</mu2></ode>
            </friction>
          </surface>
        </collision>""")
    visual = textwrap.dedent(f"""\
        <visual name="ground">
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane><normal>0 0 1</normal><size>900 600</size></plane>
          </geometry>
          {material(C_GRASS, specular=0.02)}
        </visual>""")
    return make_model("ground_plane", make_link("ground", [collision], [visual]))


def build_road_surface() -> str:
    visuals = [
        visual_box(
            "asphalt",
            box_size(ROAD_LEN, ROAD_W, 0.004),
            pose(ROAD_CX, ROAD_CY, 0.002),
            material(C_ASPHALT, specular=0.03),
            cast_shadows=False,
        ),
        visual_box(
            "emergency_lane",
            box_size(ROAD_LEN, EMERG_W, 0.002),
            pose(ROAD_CX, EMERG_CY, 0.005),
            material(C_EMERGENCY, specular=0.04),
            cast_shadows=False,
        ),
    ]
    return make_model("mainline_road", make_link("road", visuals=visuals))


# ---------------------------------------------------------------------------
# World assembly
# ---------------------------------------------------------------------------
def _drop_whitespace_nodes(node) -> None:
    """Drop indentation text nodes so minidom can re-indent from scratch."""
    for child in list(node.childNodes):
        if child.nodeType == child.TEXT_NODE:
            if not child.data.strip():
                node.removeChild(child)
        elif child.nodeType == child.ELEMENT_NODE:
            _drop_whitespace_nodes(child)


def pretty_xml(text: str) -> str:
    """Return consistently indented XML with the declaration at column 0."""
    doc = minidom.parseString(text.lstrip())
    _drop_whitespace_nodes(doc.documentElement)
    return doc.toprettyxml(indent="  ")


def build_world() -> str:
    ensure_sign_textures()

    models = "\n".join([
        build_ground(),
        build_road_surface(),
        build_ramp(),
        build_lane_markings(),
        build_guardrails(),
        build_gantry(),
        build_streetlights(),
        build_trees(),
        build_buildings(),
        build_road_markers(),
    ])

    return textwrap.dedent(f"""\
        <?xml version="1.0" ?>
        <!--
          city_road.world.sdf

          City-road patrol / on-ramp merge scene for the smart patrol robot.
          Generated by generate_city_road_world.py; edit that script (or the
          parameters at its top) and re-run it instead of hand-editing this
          file.

          Layout (from doc/0.req/0-req02.png and doc/0.req/0-req03.png):
            - four-lane one-way carriageway, right-hand traffic:
                超车道 | 行车道 | 行车道 | 应急车道
            - the emergency lane is y in [{f(EMERG_Y0)}, {f(EMERG_Y1)}];
              the robot starts at (0, 0) heading +X;
            - an on-ramp joins from the right at the merge point x = {f(MERGE_X)};
            - the yellow disc is the manual-takeover ("通过点"), the green disc
              50 m later is where autonomous driving resumes.

          Coordinate system: +X = travel direction, +Y = left, +Z = up.
        -->
        <sdf version="1.7">
          <world name="city_road">
            <physics name="1ms" type="ignored">
              <max_step_size>0.001</max_step_size>
              <real_time_factor>1.0</real_time_factor>
            </physics>

            <gravity>0 0 -9.8</gravity>
            <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
            <atmosphere type="adiabatic"/>

            <plugin filename="gz-sim-physics-system"
                    name="gz::sim::systems::Physics"/>
            <plugin filename="gz-sim-user-commands-system"
                    name="gz::sim::systems::UserCommands"/>
            <plugin filename="gz-sim-scene-broadcaster-system"
                    name="gz::sim::systems::SceneBroadcaster"/>
            <plugin filename="gz-sim-contact-system"
                    name="gz::sim::systems::Contact"/>
            <plugin filename="gz-sim-sensors-system"
                    name="gz::sim::systems::Sensors">
              <render_engine>ogre2</render_engine>
            </plugin>
            <plugin filename="gz-sim-imu-system"
                    name="gz::sim::systems::Imu"/>
            <plugin filename="gz-sim-navsat-system"
                    name="gz::sim::systems::NavSat"/>

            <!-- GNSS origin.  Replace with the real site if known. -->
            <spherical_coordinates>
              <surface_model>EARTH_WGS84</surface_model>
              <world_frame_orientation>ENU</world_frame_orientation>
              <latitude_deg>31.2304</latitude_deg>
              <longitude_deg>121.4737</longitude_deg>
              <elevation>0</elevation>
            </spherical_coordinates>

            <light name="sun" type="directional">
              <cast_shadows>true</cast_shadows>
              <pose>0 0 20 0 0 0</pose>
              <diffuse>0.85 0.85 0.82 1</diffuse>
              <specular>0.20 0.20 0.20 1</specular>
              <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
              </attenuation>
              <direction>-0.40 0.30 -0.87</direction>
            </light>

            <scene>
              <ambient>0.45 0.45 0.45 1</ambient>
              <background>0.55 0.72 0.88 1</background>
              <shadows>true</shadows>
            </scene>

            <wind/>

        {indent(models, 4)}
          </world>
        </sdf>""")


def main() -> None:
    text = pretty_xml(build_world())
    OUT.write_text(text, encoding="utf-8")
    print(f"wrote {OUT} ({len(text)} bytes)")


if __name__ == "__main__":
    main()
