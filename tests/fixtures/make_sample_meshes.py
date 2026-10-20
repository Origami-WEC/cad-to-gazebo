#!/usr/bin/env python3
"""make_sample_meshes.py -- author the C4 sample assembly (meshes + IR).

The SDF-ready artefact is only trustworthy if it is regenerated from a *known*
input: this script is the provenance of ``tests/fixtures/meshes/`` and of
``tests/fixtures/cad_manifest_sample_assembly.json``. It is deterministic and
uses only numpy (plus the STL writer of ``cad.sdf_artifacts``), so the fixtures
never depend on FreeCAD or on trimesh.

It exercises every branch of the approximation ladder and of the material
mapping in one assembly:

======================  ==========================  ===================
solid                   geometry                    expected artefact
======================  ==========================  ===================
``hull__shell``         axis-aligned box            ``box`` primitive
``pto_shaft__mass``     z-axis cylinder             ``cylinder``
``float__buoy``         sphere                      ``sphere``
``float__ballast``      2nd solid of the same body  multi-solid inertia
``keel__fin``           dense ellipsoid             ``convex_hull`` + budget
``mystery_composite``   unmapped material card      PBR ``fallback``
======================  ==========================  ===================

Usage::

    python tests/fixtures/make_sample_meshes.py [--out tests/fixtures]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from cad.sdf_artifacts import write_binary_stl  # noqa: E402

ROOT = os.path.dirname(os.path.abspath(__file__))


# --------------------------------------------------------------------------- #
# primitive meshes (triangle soup, outward normals, closed)
# --------------------------------------------------------------------------- #


def _box(size):
    """Closed box mesh centred at the origin, axis-aligned, size (x, y, z)."""
    hx, hy, hz = (s / 2.0 for s in size)
    corners = np.array([
        [-hx, -hy, -hz], [+hx, -hy, -hz], [+hx, +hy, -hz], [-hx, +hy, -hz],
        [-hx, -hy, +hz], [+hx, -hy, +hz], [+hx, +hy, +hz], [-hx, +hy, +hz],
    ], dtype=np.float64)
    quads = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    tris = []
    for a, b, c, d in quads:
        tris += [corners[a], corners[b], corners[c]]
        tris += [corners[a], corners[c], corners[d]]
    return np.asarray(tris, dtype=np.float64)


def _orient(tris):
    """Return a closed triangle soup with outward winding.

    The generators below build their faces in whatever order is convenient; a
    negative signed volume means the winding is inward, so every triangle is
    flipped at once. Keeping this in one place is what lets the primitives stay
    trivially simple -- and the fitted properties of the artefact never depend on
    it, because :func:`cad.sdf_artifacts.mesh_volume` takes the absolute value.
    """
    t = np.asarray(tris, dtype=np.float64).reshape(-1, 3, 3)
    vol = float((t[:, 0] * np.cross(t[:, 1], t[:, 2])).sum() / 6.0)
    if vol < 0.0:
        t = t[:, [0, 2, 1]]
    return t.reshape(-1, 3)


def _closed_check(name, tris, expected, tol=0.03):
    """Fail the fixture build if a closed soup does not enclose ``expected``.

    The divergence formula below is exact for any *consistently wound* closed
    surface, so a mismatch means the mesh is open or two face families were
    wound against each other -- a fixture bug that would otherwise only surface
    as a wrong collision volume in the artefact.
    """
    t = np.asarray(tris, dtype=np.float64).reshape(-1, 3, 3)
    vol = abs(float((t[:, 0] * np.cross(t[:, 1], t[:, 2])).sum() / 6.0))
    if abs(vol - expected) / expected > tol:
        raise SystemExit("fixture %s: soup volume %.6f != expected %.6f (tol %.0f%%)"
                         % (name, vol, expected, tol * 100.0))
    return vol


def _cylinder(radius, length, segments=32):
    """Closed cylinder along +z, centred at the origin.

    Every vertex lies *on* the surface: the caps are fans anchored on the first
    rim vertex, never on an interior centre point, because the cylinder fit of
    :mod:`cad.sdf_artifacts` checks the radial coverage of the mesh vertices.
    """
    ang = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
    ring = np.stack([radius * np.cos(ang), radius * np.sin(ang),
                     np.full(segments, -length / 2.0)], axis=1)
    ring_top = ring.copy()
    ring_top[:, 2] = +length / 2.0
    tris = []
    for i in range(segments):
        j = (i + 1) % segments
        tris += [ring[i], ring[j], ring_top[j]]
        tris += [ring[i], ring_top[j], ring_top[i]]
    for i in range(1, segments - 1):
        # fan from rim vertex 0: range(1, n-1) covers the disc exactly once and
        # never emits the degenerate closing triangle (0, 0, n-1). The bottom cap
        # faces -z and the top cap +z, so their windings are opposite: the rim
        # runs counter-clockwise seen from +z.
        tris += [ring[0], ring[i + 1], ring[i]]
        tris += [ring_top[0], ring_top[i], ring_top[i + 1]]
    return _orient(tris)


def _sphere(radius, meridians=24, parallels=12):
    """Closed UV sphere centred at the origin (pole fans, no degenerate rows)."""

    def pt(theta, phi):
        return [radius * math.sin(theta) * math.cos(phi),
                radius * math.sin(theta) * math.sin(phi),
                radius * math.cos(theta)]

    tris = []
    for i in range(1, parallels - 1):
        t0 = math.pi * i / parallels
        t1 = math.pi * (i + 1) / parallels
        for j in range(meridians):
            p0 = 2.0 * math.pi * j / meridians
            p1 = 2.0 * math.pi * (j + 1) / meridians
            a, b, c, d = pt(t0, p0), pt(t0, p1), pt(t1, p1), pt(t1, p0)
            tris += [a, b, c]
            tris += [a, c, d]
    north, south = [0.0, 0.0, radius], [0.0, 0.0, -radius]
    t_n, t_s = math.pi / parallels, math.pi * (parallels - 1) / parallels
    for j in range(meridians):
        p0 = 2.0 * math.pi * j / meridians
        p1 = 2.0 * math.pi * (j + 1) / meridians
        tris += [north, pt(t_n, p1), pt(t_n, p0)]
        tris += [south, pt(t_s, p0), pt(t_s, p1)]
    return _orient(tris)


def _ellipsoid_sheet(semi, n_theta=26, n_phi=52):
    """Dense triangle soup on an ellipsoid surface (drives hull + budget).

    Deliberately dense (thousands of vertices) so the raw convex hull blows past
    the face budget and the vertex-clustering rung of
    :func:`cad.sdf_artifacts._hull_within_budget` is exercised for real. The
    polar rows degenerate to slivers; they are dropped so the STL stays clean.
    """
    a, b, c = semi
    theta = np.linspace(0.0, math.pi, n_theta)
    phi = np.linspace(0.0, 2.0 * math.pi, n_phi, endpoint=False)
    grid = np.stack([
        a * np.sin(theta)[:, None] * np.cos(phi)[None, :],
        b * np.sin(theta)[:, None] * np.sin(phi)[None, :],
        c * np.cos(theta)[:, None] * np.ones_like(phi)[None, :],
    ], axis=-1)
    tris = []
    for i in range(1, n_theta - 2):
        for j in range(n_phi):
            k = (j + 1) % n_phi
            tris += [grid[i, j], grid[i, k], grid[i + 1, k]]
            tris += [grid[i, j], grid[i + 1, k], grid[i + 1, j]]
    north, south = grid[0, 0], grid[-1, 0]
    for j in range(n_phi):
        k = (j + 1) % n_phi
        tris += [north, grid[1, k], grid[1, j]]
        tris += [south, grid[-2, j], grid[-2, k]]
    return _orient(tris)


# --------------------------------------------------------------------------- #
# analytic mass properties (the IR's reference_* values)
# --------------------------------------------------------------------------- #


def _box_props(size, rho):
    a, b, c = size
    vol = a * b * c
    mass = rho * vol
    i = mass / 12.0
    return vol, mass, [0.0, 0.0, 0.0], [
        [i * (b * b + c * c), 0.0, 0.0],
        [0.0, i * (a * a + c * c), 0.0],
        [0.0, 0.0, i * (a * a + b * b)],
    ]


def _cylinder_props(radius, length, rho):
    vol = math.pi * radius * radius * length
    mass = rho * vol
    ix = mass / 12.0 * (3.0 * radius * radius + length * length)
    iz = mass * radius * radius / 2.0
    return vol, mass, [0.0, 0.0, 0.0], [
        [ix, 0.0, 0.0], [0.0, ix, 0.0], [0.0, 0.0, iz],
    ]


def _sphere_props(radius, rho):
    vol = 4.0 / 3.0 * math.pi * radius ** 3
    mass = rho * vol
    i = 2.0 / 5.0 * mass * radius * radius
    return vol, mass, [0.0, 0.0, 0.0], [
        [i, 0.0, 0.0], [0.0, i, 0.0], [0.0, 0.0, i],
    ]


def _ellipsoid_props(semi, rho):
    a, b, c = semi
    vol = 4.0 / 3.0 * math.pi * a * b * c
    mass = rho * vol
    k = mass / 5.0
    return vol, mass, [0.0, 0.0, 0.0], [
        [k * (b * b + c * c), 0.0, 0.0],
        [0.0, k * (a * a + c * c), 0.0],
        [0.0, 0.0, k * (a * a + b * b)],
    ]


def _round(value):
    return round(float(value), 9) if abs(float(value)) > 1e-12 else 0.0


def _mat(m):
    return [[_round(v) for v in row] for row in m]


# --------------------------------------------------------------------------- #
# assembly description
# --------------------------------------------------------------------------- #

SHELL_SIZE = (2.0, 1.2, 0.8)
SHUTTLE_RADIUS = 0.12
SHUTTLE_LENGTH = 1.4
BUOY_RADIUS = 0.5
BALLAST_SIZE = (0.6, 0.6, 0.3)
KEEL_SEMI = (0.9, 0.25, 0.5)


def build(out_dir):
    mesh_dir = os.path.join(out_dir, "meshes", "visual")
    os.makedirs(mesh_dir, exist_ok=True)

    # --- geometry ---------------------------------------------------------- #
    meshes = {
        "hull__shell": _box(SHELL_SIZE),
        "pto_shaft__mass": _cylinder(SHUTTLE_RADIUS, SHUTTLE_LENGTH),
        "float__buoy": _sphere(BUOY_RADIUS),
        "float__ballast": _box(BALLAST_SIZE),
        # the dense ellipsoid: no primitive fit, so it drives the convex-hull +
        # face-budget rung of the ladder.
        "keel__fin": _ellipsoid_sheet(KEEL_SEMI),
    }
    for name, tris in meshes.items():
        verts = tris.reshape(-1, 3)
        faces = np.arange(verts.shape[0], dtype=np.int64).reshape(-1, 3)
        write_binary_stl(os.path.join(mesh_dir, name + ".stl"), verts, faces)

    # every fixture mesh is a closed solid, so the divergence volume below must
    # agree with the analytic volume the IR declares: if it does not, the mesh
    # is open or wound inconsistently and the artefact would carry a wrong
    # collision volume. The build fails here instead.
    _closed_check("hull__shell", meshes["hull__shell"], _box_props(SHELL_SIZE, 1.0)[0])
    _closed_check("pto_shaft__mass", meshes["pto_shaft__mass"],
                  _cylinder_props(SHUTTLE_RADIUS, SHUTTLE_LENGTH, 1.0)[0])
    _closed_check("float__buoy", meshes["float__buoy"], _sphere_props(BUOY_RADIUS, 1.0)[0])
    _closed_check("float__ballast", meshes["float__ballast"], _box_props(BALLAST_SIZE, 1.0)[0])
    _closed_check("keel__fin", meshes["keel__fin"], _ellipsoid_props(KEEL_SEMI, 1.0)[0])

    # --- mass properties --------------------------------------------------- #
    vol, mass, com, inertia = _box_props(SHELL_SIZE, 7850.0)
    shell = dict(vol=vol, mass=mass, com=com, inertia=inertia, rho=7850.0, card="Steel")
    vol, mass, com, inertia = _cylinder_props(SHUTTLE_RADIUS, SHUTTLE_LENGTH, 2700.0)
    shaft = dict(vol=vol, mass=mass, com=com, inertia=inertia, rho=2700.0,
                 card="Alluminio 6061")
    vol, mass, com, inertia = _sphere_props(BUOY_RADIUS, 950.0)
    buoy = dict(vol=vol, mass=mass, com=com, inertia=inertia, rho=950.0, card="HDPE")
    vol, mass, com, inertia = _box_props(BALLAST_SIZE, 2400.0)
    ballast = dict(vol=vol, mass=mass, com=com, inertia=inertia, rho=2400.0,
                   card="Concrete")
    vol, mass, com, inertia = _ellipsoid_props(KEEL_SEMI, 1500.0)
    keel = dict(vol=vol, mass=mass, com=com, inertia=inertia, rho=1500.0,
                card="mystery_composite")

    identity7 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    # ballast rotated 90 deg about x and pushed down: exercises the inertia
    # rotation and the parallel-axis step of the multi-solid aggregation.
    half_sqrt = math.sqrt(0.5)
    ballast_pose = [0.0, 0.0, -0.45, half_sqrt, 0.0, 0.0, half_sqrt]

    def solid(name, label, props, pose=identity7):
        return {
            "name": name,
            "label": label,
            "pose": list(pose),
            "density_kg_m3": _round(props["rho"]),
            "density_source": "analytic",
            "reference_mass_kg": _round(props["mass"]),
            "reference_volume_m3": _round(props["vol"]),
            "reference_com_m": [_round(v) for v in props["com"]],
            "reference_inertia_kg_m2": _mat(props["inertia"]),
            "visual_mesh": "model://meshes/visual/{}.stl".format(name),
            "collision_mesh": "model://meshes/collision/{}_convex.stl".format(name),
            "stl_path": "meshes/visual/{}.stl".format(name),
            "stl_collision_path": "meshes/collision/{}_convex.stl".format(name),
            "material": {
                "card": props["card"],
                "diffuse": [0.6, 0.6, 0.6, 1.0],
                "ambient": [0.6, 0.6, 0.6, 1.0],
            },
        }

    ir = {
        "schema": "cad-to-gazebo/cad-ir/1",
        "meta": {
            "generator": "make_sample_meshes.py",
            "emitter_version": "cad-to-gazebo/0.1.0",
            "device_type": "generic",
            "source_document": "sample assembly (C4 fixtures)",
            "source_file": "",
            "source_hash": "",
            "units": "SI (m, kg, rad)",
            "created": "2026-10-20T00:00:00",
            "root": "hull",
        },
        "bodies": [
            {
                "name": "hull",
                "label": "Hull",
                "pose": list(identity7),
                "solids": [solid("hull__shell", "shell", shell)],
                "num_solids": 1,
            },
            {
                "name": "pto_shaft",
                "label": "PTO shaft",
                "pose": [0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0],
                "solids": [solid("pto_shaft__mass", "mass", shaft)],
                "num_solids": 1,
            },
            {
                "name": "float",
                "label": "Float",
                "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                "solids": [
                    solid("float__buoy", "buoy", buoy),
                    solid("float__ballast", "ballast", ballast, ballast_pose),
                ],
                "num_solids": 2,
            },
            {
                "name": "keel",
                "label": "Keel",
                "pose": [0.0, 0.0, -0.4, 0.0, 0.0, 0.0, 1.0],
                "solids": [solid("keel__fin", "fin", keel)],
                "num_solids": 1,
            },
        ],
        "joints": [
            {
                "name": "pto_joint",
                "label": "pto_joint",
                "fc_type": "Slider",
                "parent": "hull",
                "child": "pto_shaft",
                "origin": [0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0],
                "axis": [0.0, 0.0, 1.0],
                "limits": {"lower": -0.6, "upper": 0.6, "lower_enabled": True,
                           "upper_enabled": True, "unit": "m", "offset_m": 0.0},
                "activated": True,
            },
            {
                "name": "keel_joint",
                "label": "keel_joint",
                "fc_type": "Fixed",
                "parent": "hull",
                "child": "keel",
                "origin": [0.0, 0.0, -0.4, 0.0, 0.0, 0.0, 1.0],
                "axis": [0.0, 0.0, 1.0],
                "limits": {"lower": 0.0, "upper": 0.0, "lower_enabled": False,
                           "upper_enabled": False, "unit": "m", "offset_m": 0.0},
                "activated": True,
            },
        ],
        "skipped_joints": [],
        "frames": [
            {
                "name": "hull__cog_hook",
                "body": "hull",
                "pose": [0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0],
                "description": "LCS di sollevamento, authoring manuale",
            },
        ],
        "anchors": [
            {
                "name": "jet_pto_shaft_bottom",
                "kind": "jet",
                "body": "pto_shaft",
                "frame": "pto_shaft",
                "pose": [0.0, 0.0, -0.7, 0.0, 0.0, 0.0, 1.0],
                "direction": [0.0, 0.0, -1.0],
            },
            {
                "name": "mooring_hull_base",
                "kind": "mooring",
                "body": "hull",
                "frame": "hull",
                "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                "direction": [0.0, 0.0, -1.0],
            },
        ],
        "sensors": [
            {
                "name": "imu_hull",
                "type": "imu",
                "frame": "hull",
                "rate_hz": 100.0,
                "topic": "/wec/hull/imu",
                "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            },
        ],
    }

    ir_path = os.path.join(out_dir, "cad_manifest_sample_assembly.json")
    with open(ir_path, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(ir, fh, indent=2, sort_keys=False)
        fh.write("\n")
    return ir_path, mesh_dir


def main(argv=None):
    parser = argparse.ArgumentParser(description="Author the C4 sample assembly fixtures")
    parser.add_argument("--out", default=ROOT, help="destination dir (default: tests/fixtures)")
    args = parser.parse_args(argv)
    ir_path, mesh_dir = build(os.path.abspath(args.out))
    print("IR   :", ir_path)
    print("mesh :", mesh_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
