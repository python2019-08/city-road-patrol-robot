#!/usr/bin/env python3
"""Generate the STL meshes used by patrolbot_description.

The geometry mirrors the dimensions in ``urdf/common.xacro``:

* ``body_cone.stl``      -- traffic-cone shell, height 0.74 m
* ``band_{low,mid,high}.stl`` -- reflective bands wrapped around the cone
* ``led_bezel.stl`` / ``led_screen.stl`` -- curved front warning screen

The body link frame is at the bottom centre of the cone, +x forward,
+z up.  Run this script after changing any cone dimension::

    python3 generate_meshes.py

No external dependencies are required.
"""

import math
import os

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# --- keep in sync with urdf/common.xacro -------------------------------------
BODY_HEIGHT = 0.740
BODY_R_BOTTOM = 0.165
BODY_R_TOP = 0.100


def body_radius(z):
    """Cone radius at height ``z`` above the body bottom."""
    return BODY_R_BOTTOM + (BODY_R_TOP - BODY_R_BOTTOM) * z / BODY_HEIGHT


# reflective bands: (name, centre height, height)
BANDS = (
    ("band_low", 0.090, 0.035),
    ("band_mid", 0.610, 0.035),
    ("band_high", 0.700, 0.035),
)
BAND_OFFSET = 0.0025

# LED warning screen, curved around the cone's front (+x) side
SCREEN_Z0 = 0.140
SCREEN_Z1 = 0.500
SCREEN_WIDTH = 0.150
SCREEN_CENTER_Z = 0.5 * (SCREEN_Z0 + SCREEN_Z1)
SCREEN_HALF_ANGLE = math.asin(
    (0.5 * SCREEN_WIDTH) / body_radius(SCREEN_CENTER_Z)
)

BEZEL_MARGIN_Z = 0.012
BEZEL_MARGIN_ANGLE = 0.045


def _normal(a, b, c):
    ux, uy, uz = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    vx, vy, vz = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length == 0.0:
        return None
    return (nx / length, ny / length, nz / length)


class Mesh:
    def __init__(self):
        self.triangles = []

    def add_triangle(self, a, b, c):
        if _normal(a, b, c) is not None:
            self.triangles.append((a, b, c))

    def add_quad(self, a, b, c, d):
        self.add_triangle(a, b, c)
        self.add_triangle(a, c, d)

    def add_quad_outward(self, a, b, c, d, reference):
        """Add a quad, flipping it if its normal points at ``reference``."""
        n = _normal(a, b, c)
        if n is None:
            return
        centre = (
            (a[0] + b[0] + c[0] + d[0]) * 0.25,
            (a[1] + b[1] + c[1] + d[1]) * 0.25,
            (a[2] + b[2] + c[2] + d[2]) * 0.25,
        )
        outward = (
            centre[0] - reference[0],
            centre[1] - reference[1],
            centre[2] - reference[2],
        )
        if n[0] * outward[0] + n[1] * outward[1] + n[2] * outward[2] < 0.0:
            a, b, c, d = a, d, c, b
        self.add_quad(a, b, c, d)

    def translate(self, offset):
        dx, dy, dz = offset
        self.triangles = [
            (
                (a[0] + dx, a[1] + dy, a[2] + dz),
                (b[0] + dx, b[1] + dy, b[2] + dz),
                (c[0] + dx, c[1] + dy, c[2] + dz),
            )
            for a, b, c in self.triangles
        ]

    def write(self, path):
        name = os.path.splitext(os.path.basename(path))[0]
        with open(path, "w", encoding="ascii") as f:
            f.write("solid %s\n" % name)
            for a, b, c in self.triangles:
                n = _normal(a, b, c)
                f.write(
                    "  facet normal %.9g %.9g %.9g\n"
                    "    outer loop\n"
                    "      vertex %.9g %.9g %.9g\n"
                    "      vertex %.9g %.9g %.9g\n"
                    "      vertex %.9g %.9g %.9g\n"
                    "    endloop\n"
                    "  endfacet\n" % (n + a + b + c)
                )
            f.write("endsolid %s\n" % name)


def frustum(mesh, r0, r1, z0, z1, segments=96):
    """Closed truncated cone around the z axis."""
    for i in range(segments):
        a0 = 2.0 * math.pi * i / segments
        a1 = 2.0 * math.pi * (i + 1) / segments
        p0b = (r0 * math.cos(a0), r0 * math.sin(a0), z0)
        p1b = (r0 * math.cos(a1), r0 * math.sin(a1), z0)
        p0t = (r1 * math.cos(a0), r1 * math.sin(a0), z1)
        p1t = (r1 * math.cos(a1), r1 * math.sin(a1), z1)
        mesh.add_quad(p0b, p1b, p1t, p0t)
        mesh.add_triangle((0.0, 0.0, z0), p1b, p0b)
        mesh.add_triangle((0.0, 0.0, z1), p0t, p1t)


def conical_panel(mesh, z0, z1, half_angle, off_in, off_out, n_phi=28, n_z=8):
    """Closed thin panel following the cone, centred on the +x direction."""
    def point(phi, z, offset):
        r = body_radius(z) + offset
        return (r * math.cos(phi), r * math.sin(phi), z)

    ref = (0.0, 0.0, 0.5 * (z0 + z1))

    for iz in range(n_z):
        za = z0 + (z1 - z0) * iz / n_z
        zb = z0 + (z1 - z0) * (iz + 1) / n_z
        for ip in range(n_phi):
            pa = -half_angle + 2.0 * half_angle * ip / n_phi
            pb = -half_angle + 2.0 * half_angle * (ip + 1) / n_phi

            # outer skin (normal away from the cone axis)
            mesh.add_quad_outward(
                point(pa, za, off_out),
                point(pb, za, off_out),
                point(pb, zb, off_out),
                point(pa, zb, off_out),
                ref,
            )
            # inner skin
            mesh.add_quad_outward(
                point(pa, za, off_in),
                point(pb, za, off_in),
                point(pb, zb, off_in),
                point(pa, zb, off_in),
                ref,
            )

    # top / bottom rims
    pa, pb = -half_angle, half_angle
    mesh.add_quad_outward(
        point(pa, z1, off_in), point(pb, z1, off_in),
        point(pb, z1, off_out), point(pa, z1, off_out), ref,
    )
    mesh.add_quad_outward(
        point(pa, z0, off_in), point(pa, z0, off_out),
        point(pb, z0, off_out), point(pb, z0, off_in), ref,
    )

    # side rims at the two arc ends
    for phi in (-half_angle, half_angle):
        mesh.add_quad_outward(
            point(phi, z0, off_in), point(phi, z0, off_out),
            point(phi, z1, off_out), point(phi, z1, off_in), ref,
        )


def main():
    # body shell
    body = Mesh()
    frustum(body, BODY_R_BOTTOM, BODY_R_TOP, 0.0, BODY_HEIGHT)
    body.write(os.path.join(OUT_DIR, "body_cone.stl"))

    # reflective bands
    for name, centre, height in BANDS:
        z0 = centre - 0.5 * height
        z1 = centre + 0.5 * height
        band = Mesh()
        frustum(
            band,
            body_radius(z0) + BAND_OFFSET,
            body_radius(z1) + BAND_OFFSET,
            z0,
            z1,
        )
        band.write(os.path.join(OUT_DIR, name + ".stl"))

    # curved LED bezel and screen; both meshes share the same origin
    centre = (body_radius(SCREEN_CENTER_Z), 0.0, SCREEN_CENTER_Z)

    bezel = Mesh()
    conical_panel(
        bezel,
        SCREEN_Z0 - BEZEL_MARGIN_Z,
        SCREEN_Z1 + BEZEL_MARGIN_Z,
        SCREEN_HALF_ANGLE + BEZEL_MARGIN_ANGLE,
        off_in=-0.004,
        off_out=0.001,
    )
    bezel.translate((-centre[0], -centre[1], -centre[2]))
    bezel.write(os.path.join(OUT_DIR, "led_bezel.stl"))

    screen = Mesh()
    conical_panel(
        screen,
        SCREEN_Z0,
        SCREEN_Z1,
        SCREEN_HALF_ANGLE,
        off_in=0.002,
        off_out=0.006,
    )
    screen.translate((-centre[0], -centre[1], -centre[2]))
    screen.write(os.path.join(OUT_DIR, "led_screen.stl"))

    for name in ("body_cone", "band_low", "band_mid", "band_high",
                 "led_bezel", "led_screen"):
        path = os.path.join(OUT_DIR, name + ".stl")
        print("%-14s %6.1f kB" % (name + ".stl", os.path.getsize(path) / 1024.0))


if __name__ == "__main__":
    main()
