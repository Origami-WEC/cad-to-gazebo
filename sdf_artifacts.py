"""sdf_artifacts.py -- the SDF-ready artefacts derived from the CAD IR.

This is block **C4** of the CAD -> SDF chain: it turns ``cad_manifest.json``
(the IR, written by the CAD front-end) into the four artefacts the SDF emitter
in ``maritime_ws`` consumes, in a single, idempotent pass:

======================  =====================================================
Artefact                What is produced
======================  =====================================================
collision hulls         one collision primitive / hull / mesh reference per
                        solid, chosen by a deterministic approximation ladder
                        (:func:`fit_collision`) with a size + quality budget
visual materials        CAD material card -> PBR (albedo / metalness /
                        roughness / emissive) via :func:`resolve_pbr_material`,
                        with a documented default for unmapped cards
mass / inertia          one per-*body* inertial per link, aggregated across the
                        solids of the body (:func:`body_inertial`) -- the
                        multi-solid case URDF cannot express
frames / anchors        the named LCS and the named mount points, carried and
                        completed for the emitter (:func:`derive_frames`,
                        :func:`derive_anchors`)
======================  =====================================================

Everything here is pure Python and FreeCAD-free: the FreeCAD layer writes the
IR, this module reads it. ``scipy`` (qhull) is imported lazily and is only
needed for the convex-hull rung of the ladder; without it the ladder stops at
the primitives and at the ``mesh`` rung, and says so in the artefact.

The artefact is a *closed* document: ``docs/sdf-artifacts.md`` pins every field,
the approximation ladder, the budget and the two deliberate exceptions to byte
idempotency (no wall-clock timestamp, no tool-version string). Nothing in it is
a TODO or a question: an open point belongs in the PR / issue, not in a file the
emitter parses.

Design decisions worth knowing before reading the code
-----------------------------------------------------

* **The ladder is totally ordered and axis-aligned.** ``box`` -> ``cylinder``
  -> ``sphere`` -> ``convex_hull`` -> ``mesh``. The primitive tests are exact
  volume + surface-cover tests on the tessellation, in that order (a cube is
  *also* a sphere by symmetry, so the box test must come first; a cylinder is
  *also* claimed by nothing else). The fit is axis-aligned on purpose: a
  rotated box degrades to a convex hull, which over-approximates a little
  instead of silently mis-placing a primitive. Over-approximation is safe for
  contact, under-approximation is not.
* **The hull budget is enforced deterministically by vertex clustering**, not by
  a quadric decimator (whose output is not guaranteed stable): snap to a grid,
  re-hull, grow the grid until the face count fits. Same input, same grid
  ladder, same bytes.
* **Unmapped materials default, they do not fail.** A missing *visual* is a
  cosmetic problem; a missing *mass* is a physics problem and the density path
  already fails loudly upstream (``material_props``). So the material artefact
  carries ``fallback: true`` and the emitter decides what to do with it.
* **The artefact is byte-idempotent.** No timestamp, no version string, all
  floats rounded, keys sorted, ``\\n`` newline, collision STL written by a
  header-fixed writer. Two runs over the same IR + meshes produce identical
  bytes; regenerating in CI and diffing is the gate (see the workflow).

CLI
---

::

    python -m cad.sdf_artifacts --manifest tests/fixtures/cad_manifest_sample_assembly.json \\
        --ws tests/fixtures --out tests/fixtures/sdf_artifacts_sample.json --write-collision
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple

if __package__ in (None, ""):  # direct execution: make ``cad`` importable
    _ROOT = os.path.dirname(os.path.abspath(__file__))
    _SRC = os.path.dirname(_ROOT)
    if _SRC not in sys.path:
        sys.path.insert(0, _SRC)

from cad import naming
from cad.cad_assembly import MassPoint, aggregate_mass_properties

__all__ = [
    "ARTIFACT_SCHEMA",
    "EMITTER_VERSION",
    "MAX_COLLISION_FACES",
    "MAX_HULL_INFLATION",
    "PRIMITIVE_TOL",
    "PRIMITIVE_VOLUME_TOL",
    "BRDF_TABLE",
    "MATERIAL_FALLBACK_BRDF",
    "SdfArtifactError",
    "read_stl",
    "write_binary_stl",
    "mesh_volume",
    "fit_collision",
    "resolve_pbr_material",
    "body_inertial",
    "derive_frames",
    "derive_anchors",
    "generate_artifacts",
    "build_parser",
    "main",
]

#: Version tag of the artefact document itself (bumped per breaking change).
ARTIFACT_SCHEMA = "cad-to-gazebo/sdf-artifacts/1"

#: ``<package>/<version>`` of this generator, mirrored into the artefact meta.
EMITTER_VERSION = "cad-to-gazebo/0.1.0"

#: Face budget for a generated collision hull. Above this the hull is
#: re-hulled on a coarser vertex grid (deterministically) until it fits.
MAX_COLLISION_FACES = 1500

#: A convex hull may over-approximate the source solid. Beyond this ratio the
#: artefact carries a warning so the emitter / reviewer can refine the hull.
MAX_HULL_INFLATION = 1.35

#: Relative tolerance of the primitive fit (1 %): a tessellated box/cylinder/
#: sphere is *not* exact, so an exact test would never fire. This one guards the
#: *geometric* test (radial spread, boundary contact).
PRIMITIVE_TOL = 0.01

#: Relative tolerance of the primitive *volume* test. Wider than
#: :data:`PRIMITIVE_TOL` on purpose: an inscribed tessellation under-fills its
#: own primitive (a 24-segment sphere is ~2.8 % under the ideal), while the
#: cross-fits this test has to reject (box read as cylinder, sphere read as box)
#: are 27-91 % off. 5 % sits safely between the two.
PRIMITIVE_VOLUME_TOL = 0.05

#: Fraction of vertices that must lie on the fitted primitive surface.
PRIMITIVE_COVER = 0.98

#: Distinct angular positions a point cloud must show around a cylinder axis.
#: A box carries four; a tessellated cylinder carries one per segment.
MIN_CYLINDER_ANGLES = 8

#: CAD material card keyword -> (metalness, roughness). Longest keyword wins,
#: so ``stainless`` beats ``steel`` and ``alluminio`` beats ``alu``.
BRDF_TABLE: Tuple[Tuple[str, float, float], ...] = (
    ("stainless", 0.9, 0.25),
    ("stainless_steel", 0.9, 0.25),
    ("steel", 0.9, 0.35),
    ("acciaio", 0.9, 0.35),
    ("iron", 0.85, 0.45),
    ("ferro", 0.85, 0.45),
    ("alluminio", 0.85, 0.35),
    ("aluminium", 0.85, 0.35),
    ("aluminum", 0.85, 0.35),
    ("alu", 0.85, 0.35),
    ("titan", 0.8, 0.35),
    ("brass", 0.9, 0.30),
    ("bronze", 0.9, 0.35),
    ("copper", 0.95, 0.30),
    ("rame", 0.95, 0.30),
    ("concrete", 0.0, 0.85),
    ("calcestruzzo", 0.0, 0.85),
    ("cement", 0.0, 0.85),
    ("rubber", 0.0, 0.95),
    ("gomma", 0.0, 0.95),
    ("elastomer", 0.0, 0.90),
    ("foam", 0.0, 0.90),
    ("schiuma", 0.0, 0.90),
    ("glass", 0.0, 0.10),
    ("vetro", 0.0, 0.10),
    ("epoxy", 0.0, 0.40),
    ("resin", 0.0, 0.40),
    ("pmma", 0.0, 0.20),
    ("hdpe", 0.0, 0.60),
    ("pvc", 0.0, 0.55),
    ("petg", 0.0, 0.55),
    ("nylon", 0.0, 0.55),
    ("pom", 0.0, 0.50),
    ("abs", 0.0, 0.50),
    ("pla", 0.0, 0.60),
    ("plastic", 0.0, 0.55),
    ("plastica", 0.0, 0.55),
    ("paint", 0.0, 0.60),
    ("vernice", 0.0, 0.60),
    ("wood", 0.0, 0.80),
    ("legno", 0.0, 0.80),
)

#: BRDF used when the CAD material card matches nothing: a neutral dielectric.
MATERIAL_FALLBACK_BRDF: Tuple[float, float] = (0.0, 0.6)

#: Cards that carry no information and must NOT be treated as a match.
_PLACEHOLDER_CARDS = frozenset({"", "default", "demo", "none", "unnamed", "material"})

#: Identity pose [x, y, z, qx, qy, qz, qw].
_IDENTITY7 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

#: Deterministic grid ladder used to decimate an over-budget hull. The cell
#: size is ``bbox_diagonal / K``; K grows geometrically.
_GRID_LADDER = (24, 32, 44, 60, 80, 110, 150, 200, 270, 360, 480, 640)

#: Decimals kept in the emitted floats. 9 is ~nanometre resolution in metres:
#: far below the simulation tolerance and enough to make the JSON stable.
_ROUND = 9

#: Unpadded, fixed header for generated binary STL. A fixed header (no name, no
#: timestamp) is what makes the write byte-idempotent.
_STL_HEADER = b"cad-to-gazebo collision hull"


class SdfArtifactError(RuntimeError):
    """Raised when the IR cannot be turned into a complete artefact."""


# --------------------------------------------------------------------------- #
# small numeric helpers
# --------------------------------------------------------------------------- #


def _r(value: Any) -> float:
    """Round a scalar to the artefact precision (deterministic serialisation)."""
    value = float(value)
    if value == 0.0:  # normalise -0.0
        return 0.0
    out = round(value, _ROUND)
    return 0.0 if out == 0.0 else out


def _rvec(values: Sequence[float]) -> List[float]:
    return [_r(v) for v in values]


def _rmat(values: Sequence[Sequence[float]]) -> List[List[float]]:
    return [_rvec(row) for row in values]


def _pose_rt(pose: Optional[Sequence[float]]) -> Tuple[Any, Any]:
    """Split a pose7 into ``(R, t)`` (3x3 rotation, 3-vector translation)."""
    import numpy as np

    if pose is None:
        pose = _IDENTITY7
    vals = [float(v) for v in pose]
    if len(vals) != 7:
        raise SdfArtifactError("pose deve avere 7 valori [x,y,z,qx,qy,qz,qw]: {!r}".format(pose))
    x, y, z, w = vals[3:]
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 1e-15:
        raise SdfArtifactError("quaternione nullo nella pose: {!r}".format(pose))
    x, y, z, w = x / n, y / n, z / n, w / n
    rot = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return rot, np.asarray(vals[:3], dtype=float)


# --------------------------------------------------------------------------- #
# STL: deterministic read / write
# --------------------------------------------------------------------------- #


def read_stl(path: str) -> Dict[str, Any]:
    """Read a binary or ASCII STL into a triangle soup.

    Binary STL is read directly with :mod:`struct` (no dependency, and it keeps
    the triangle order of the file). An ASCII STL is parsed as a fallback. The
    result keeps one vertex per triangle corner: welding is left to the caller,
    which is what makes the primitive fit independent of the mesh's topology.
    """
    with open(path, "rb") as fh:
        blob = fh.read()
    if len(blob) >= 84:
        (count,) = struct.unpack("<I", blob[80:84])
        if len(blob) == 84 + 50 * count:
            return _read_binary_stl(blob, count)
    text = blob.decode("ascii", errors="replace")
    if text.lstrip()[:5].lower() == "solid":
        return _read_ascii_stl(text)
    raise SdfArtifactError("STL illeggibile (ne' binario coerente ne' ASCII): {}".format(path))


def _read_binary_stl(blob: bytes, count: int) -> Dict[str, Any]:
    import numpy as np

    raw = np.frombuffer(blob[84:84 + 50 * count], dtype=np.uint8).reshape(count, 50)
    floats = raw[:, :48].copy().view("<f4").reshape(count, 4, 3).astype(np.float64)
    vertices = floats[:, 1:, :].reshape(count * 3, 3)
    faces = np.arange(count * 3, dtype=np.int64).reshape(count, 3)
    return {"vertices": vertices, "faces": faces}


def _read_ascii_stl(text: str) -> Dict[str, Any]:
    import numpy as np

    coords: List[Tuple[float, float, float]] = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0].lower() == "vertex":
            coords.append((float(parts[1]), float(parts[2]), float(parts[3])))
    if len(coords) % 3 != 0 or not coords:
        raise SdfArtifactError("STL ASCII senza terne di vertici complete")
    vertices = np.asarray(coords, dtype=np.float64)
    faces = np.arange(len(coords), dtype=np.int64).reshape(-1, 3)
    return {"vertices": vertices, "faces": faces}


def write_binary_stl(path: str, vertices: Any, faces: Any) -> None:
    """Write a binary STL with a fixed header (byte-idempotent output).

    ``trimesh.export`` cannot be used for the committed artefact: its header
    carries the mesh name and, depending on version, a date, which would break
    the idempotency gate. This writer pins the 80-byte header and emits only the
    triangle data, so the same triangle list always gives the same file.
    """
    import numpy as np

    verts = np.asarray(vertices, dtype=np.float64).reshape(-1, 3)
    tris = np.asarray(faces, dtype=np.int64).reshape(-1, 3)
    header = _STL_HEADER[:80].ljust(80, b"\x00")
    out = bytearray(header)
    out += struct.pack("<I", tris.shape[0])
    for tri in tris:
        a, b, c = (verts[int(i)] for i in tri)
        normal = np.cross(b - a, c - a)
        norm = float(np.linalg.norm(normal))
        if norm > 1e-15:
            normal = normal / norm
        else:
            normal = np.zeros(3)
        for vec in (normal, a, b, c):
            out += struct.pack("<3f", *(float(v) for v in vec))
        out += struct.pack("<H", 0)  # attribute byte count: mandatory in binary STL
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    with open(path, "wb") as fh:
        fh.write(bytes(out))


# --------------------------------------------------------------------------- #
# geometry: volume, primitive fit, convex hull + budget
# --------------------------------------------------------------------------- #


def mesh_volume(vertices: Any, faces: Any) -> float:
    """Signed-tetrahedron volume of a closed triangle soup (absolute value)."""
    import numpy as np

    verts = np.asarray(vertices, dtype=np.float64).reshape(-1, 3)
    tris = np.asarray(faces, dtype=np.int64).reshape(-1, 3)
    a, b, c = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
    return float(abs((a * np.cross(b, c)).sum(axis=1).sum() / 6.0))


def _surface_cover(points: Any, axis_radial: int) -> Tuple[float, float, float]:
    """Return ``(radius, cover, length)`` of an axis-aligned cylinder fit.

    ``radius`` is the mean radius in the plane normal to ``axis_radial``,
    ``cover`` the fraction of points within ``PRIMITIVE_TOL`` of that radius and
    ``length`` the extent along the axis.
    """
    import numpy as np

    pts = np.asarray(points, dtype=np.float64)
    others = [i for i in range(3) if i != axis_radial]
    centre = 0.5 * (pts[:, others].min(axis=0) + pts[:, others].max(axis=0))
    off = pts[:, others] - centre
    radial = np.linalg.norm(off, axis=1)
    radius = float(radial.mean())
    if radius <= 0.0:
        return 0.0, 0.0, 0.0
    cover = float((np.abs(radial - radius) <= PRIMITIVE_TOL * radius).mean())
    length = float(pts[:, axis_radial].max() - pts[:, axis_radial].min())
    return radius, cover, length


def _fit_box(points: Any, volume: float) -> Optional[Dict[str, Any]]:
    import numpy as np

    pts = np.asarray(points, dtype=np.float64)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    size = hi - lo
    box_volume = float(np.prod(size))
    if box_volume <= 0.0 or volume <= 0.0:
        return None
    if abs(box_volume / volume - 1.0) > PRIMITIVE_VOLUME_TOL:
        return None
    # every vertex must sit on the box boundary (rejects a sphere, whose AABB
    # volume ratio already fails, and any point cloud inside the box).
    tol = PRIMITIVE_TOL * float(np.linalg.norm(size)) / 3.0
    d_lo = np.abs(pts - lo)
    d_hi = np.abs(pts - hi)
    on_boundary = (np.minimum(d_lo, d_hi) <= tol).any(axis=1)
    if not bool(on_boundary.all()):
        return None
    return {
        "type": "box",
        "size_m": _rvec(size),
        "centre_m": _rvec(0.5 * (lo + hi)),
        "volume_m3": _r(box_volume),
        "cover": 1.0,
    }


def _angular_positions(points: Any, axis: int) -> int:
    """Distinct angular positions around ``axis`` (rounded to 0.05 rad).

    A tessellated cylinder carries many angles; a box only carries four. This is
    what stops an axis-aligned box whose corners happen to share one radius from
    being reported as a cylinder.
    """
    import numpy as np

    pts = np.asarray(points, dtype=np.float64)
    others = [i for i in range(3) if i != axis]
    centre = 0.5 * (pts[:, others].min(axis=0) + pts[:, others].max(axis=0))
    off = pts[:, others] - centre
    ang = np.round(np.arctan2(off[:, 1], off[:, 0]) / 0.05) * 0.05
    return int(np.unique(ang).size)


def _fit_cylinder(points: Any, volume: float) -> Optional[Dict[str, Any]]:
    for axis in range(3):
        radius, cover, length = _surface_cover(points, axis)
        if radius <= 0.0 or length <= 0.0 or cover < PRIMITIVE_COVER:
            continue
        if _angular_positions(points, axis) < MIN_CYLINDER_ANGLES:
            continue
        cyl_volume = math.pi * radius * radius * length
        if cyl_volume <= 0.0 or abs(cyl_volume / volume - 1.0) > PRIMITIVE_VOLUME_TOL:
            continue
        return {
            "type": "cylinder",
            "radius_m": _r(radius),
            "length_m": _r(length),
            "axis": ["x", "y", "z"][axis],
            "centre_m": _rvec(_centre_on_axis(points, axis)),
            "volume_m3": _r(cyl_volume),
            "cover": _r(cover),
        }
    return None


def _centre_on_axis(points: Any, axis: int) -> Any:
    import numpy as np

    pts = np.asarray(points, dtype=np.float64)
    centre = np.zeros(3)
    centre[axis] = 0.5 * (pts[:, axis].min() + pts[:, axis].max())
    others = [i for i in range(3) if i != axis]
    centre[others] = 0.5 * (pts[:, others].min(axis=0) + pts[:, others].max(axis=0))
    return centre


def _fit_sphere(points: Any, volume: float) -> Optional[Dict[str, Any]]:
    import numpy as np

    pts = np.asarray(points, dtype=np.float64)
    centre = 0.5 * (pts.min(axis=0) + pts.max(axis=0))
    radii = np.linalg.norm(pts - centre, axis=1)
    radius = float(radii.mean())
    if radius <= 0.0:
        return None
    if float(radii.std()) / radius > PRIMITIVE_TOL:
        return None
    sph_volume = 4.0 / 3.0 * math.pi * radius ** 3
    if sph_volume <= 0.0 or abs(sph_volume / volume - 1.0) > PRIMITIVE_VOLUME_TOL:
        return None
    return {
        "type": "sphere",
        "radius_m": _r(radius),
        "centre_m": _rvec(centre),
        "volume_m3": _r(sph_volume),
        "cover": 1.0,
    }


class _Hull:
    """Minimal convex-hull result: ``vertices`` (V, 3) and ``faces`` (F, 3)."""

    __slots__ = ("vertices", "faces")

    def __init__(self, vertices: Any, faces: Any):
        self.vertices = vertices
        self.faces = faces


def _hull(points: Any) -> Optional[_Hull]:
    """Convex hull of ``points`` via qhull (``None`` without scipy).

    ``scipy.spatial.ConvexHull`` is the qhull binding, and qhull is
    deterministic for a fixed point order -- no randomised decimator, no
    tessellation-dependent output. scipy ships with the ``freecad`` extra
    precisely for this rung of the ladder; without it the ladder stops at
    ``mesh`` and the artefact says so.
    """
    try:
        from scipy.spatial import ConvexHull
    except ImportError:  # pragma: no cover - exercised via monkeypatch in tests
        return None
    import numpy as np

    pts = np.unique(np.asarray(points, dtype=np.float64), axis=0)
    if pts.shape[0] < 4 or np.linalg.matrix_rank(pts - pts[0]) < 3:
        return None
    hull = ConvexHull(pts)
    faces = np.asarray(hull.simplices, dtype=np.int64)
    # qhull returns the first vertex of each triangle with a fixed orientation;
    # re-orient so every face normal points away from the centroid, which makes
    # the written STL well-behaved for the emitter without recomputing normals.
    verts = pts[hull.vertices]
    remap = -np.ones(pts.shape[0], dtype=np.int64)
    remap[hull.vertices] = np.arange(hull.vertices.size)
    faces = remap[faces]
    centre = verts.mean(axis=0)
    tri = verts[faces]
    normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    inward = ((tri[:, 0] - centre) * normals).sum(axis=1) < 0.0
    faces[inward] = faces[inward][:, [0, 2, 1]]
    return _Hull(verts, faces)


def _hull_within_budget(points: Any, faces_max: int) -> Tuple[Any, Optional[float], bool]:
    """Hull of ``points`` capped to ``faces_max`` faces by vertex clustering.

    Returns ``(hull, cell_size, within_budget)``; ``cell_size`` is ``None`` when
    the raw hull already fitted. The face count is not monotonic in the cell
    size, so every rung is tried and the smallest hull found is kept when no
    rung meets the budget: the caller reports the miss rather than silently
    shipping an over-budget hull. ``False`` is returned with a ``None`` hull when
    no hull could be built at all (no qhull).
    """
    import numpy as np

    pts = np.asarray(points, dtype=np.float64)
    diag = float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)))
    if diag <= 0.0:
        return None, None, False
    hull = _hull(pts)
    if hull is None:
        return None, None, False
    if hull.faces.shape[0] <= faces_max:
        return hull, None, True
    best, best_cell = hull, None
    for k in _GRID_LADDER:
        cell = diag / k
        snapped = np.round(pts / cell) * cell
        uniq = np.unique(snapped, axis=0)
        if uniq.shape[0] < 4:
            continue
        trial = _hull(uniq)
        if trial is None:
            return None, None, False
        if trial.faces.shape[0] <= faces_max:
            return trial, cell, True
        if trial.faces.shape[0] < best.faces.shape[0]:
            best, best_cell = trial, cell
    return best, best_cell, False


def fit_collision(mesh: Dict[str, Any], *, faces_max: int = MAX_COLLISION_FACES,
                  source_mesh: str = "") -> Dict[str, Any]:
    """Choose the collision representation of one solid.

    The ladder is ``box`` -> ``cylinder`` -> ``sphere`` -> ``convex_hull`` ->
    ``mesh``, evaluated in that order (see the module docstring for why). The
    returned dict always carries the chosen ``approximation``, the geometry to
    emit, the volumes and any ``warnings``; it never raises for a mesh it cannot
    fit -- it degrades to the next rung and records why.
    """
    import numpy as np

    vertices = np.asarray(mesh["vertices"], dtype=np.float64)
    faces = np.asarray(mesh["faces"], dtype=np.int64)
    volume = mesh_volume(vertices, faces)
    warnings: List[str] = []
    record: Dict[str, Any] = {
        "approximation": "mesh",
        "geometry": {"type": "mesh", "mesh": source_mesh},
        "source_mesh": source_mesh,
        "collision_mesh": None,
        "volume_m3": _r(volume),
        "hull_volume_m3": None,
        "hull_inflation": None,
        "faces": 0,
        "decimation_cell_m": None,
        "fallback": True,
        "warnings": warnings,
    }
    if volume <= 0.0:
        warnings.append("volume nulla o mesh non chiusa: nessuna primitiva stimabile")

    for name, fitter in (("box", _fit_box), ("cylinder", _fit_cylinder), ("sphere", _fit_sphere)):
        fit = fitter(vertices, volume)
        if fit is not None:
            record.update({
                "approximation": name,
                "geometry": fit,
                "volume_m3": _r(volume),
                "hull_volume_m3": fit["volume_m3"],
                "hull_inflation": _r(fit["volume_m3"] / volume) if volume > 0 else None,
                "faces": 0,
                "fallback": False,
            })
            return record

    hull, cell, within = _hull_within_budget(vertices, faces_max)
    if hull is None:
        warnings.append("scipy assente: collisione = mesh visuale (nessun hull generato)")
        return record

    hull_volume = mesh_volume(hull.vertices, hull.faces)
    record.update({
        "approximation": "convex_hull",
        "geometry": {"type": "mesh", "mesh": naming.collision_mesh_name(
            *_split_body_solid(source_mesh))},
        "collision_mesh": naming.collision_mesh_name(*_split_body_solid(source_mesh)),
        "hull_volume_m3": _r(hull_volume),
        "hull_inflation": _r(hull_volume / volume) if volume > 0 else None,
        "faces": int(hull.faces.shape[0]),
        "decimation_cell_m": _r(cell) if cell else None,
        "fallback": False,
    })
    if not within:
        warnings.append(
            "budget di %d facce non raggiungibile: hull minimo %d facce "
            "(griglia %.4g m)" % (faces_max, record["faces"], record["decimation_cell_m"] or 0.0))
    if record["hull_inflation"] is not None and record["hull_inflation"] > MAX_HULL_INFLATION:
        warnings.append(
            "hull convesso %.2fx il volume del solido (budget %.2f): "
            "over-approximazione ampia" % (record["hull_inflation"], MAX_HULL_INFLATION))
    return record


def _split_body_solid(visual_mesh: str) -> Tuple[str, str]:
    """``meshes/visual/<body>__<solid>.stl`` -> ``(<body>, <solid>)``."""
    stem = os.path.basename(visual_mesh or "")
    if stem.endswith(".stl"):
        stem = stem[:-4]
    if naming.MESH_SEPARATOR in stem:
        body, solid = stem.split(naming.MESH_SEPARATOR, 1)
    else:
        body, solid = stem, stem
    return naming.sanitize(body), naming.sanitize(solid)


def _collision_hull_mesh(mesh: Dict[str, Any], faces_max: int) -> Tuple[Any, Optional[float]]:
    import numpy as np

    hull, cell, _within = _hull_within_budget(
        np.asarray(mesh["vertices"], dtype=np.float64), faces_max)
    return hull, cell


# --------------------------------------------------------------------------- #
# materials -> PBR
# --------------------------------------------------------------------------- #


def _match_card(card: str) -> Optional[Tuple[float, float]]:
    """Longest-keyword match of a CAD material card against :data:`BRDF_TABLE`."""
    text = (card or "").strip().lower()
    if text in _PLACEHOLDER_CARDS:
        return None
    best: Optional[Tuple[str, float, float]] = None
    for keyword, metalness, roughness in BRDF_TABLE:
        if keyword in text and (best is None or len(keyword) > len(best[0])):
            best = (keyword, metalness, roughness)
    if best is None:
        return None
    return (best[1], best[2])


def resolve_pbr_material(material: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Map an IR material card to an SDF visual material (PBR).

    ``diffuse`` becomes ``albedo``, ``ambient`` is carried through for the
    classic emitter, ``metalness``/``roughness`` come from :data:`BRDF_TABLE`
    (longest keyword wins), and ``emissive`` is *not invented*: it is taken from
    an explicit ``emissive`` key in the IR or left black.

    Unmapped cards do **not** fail: they get :data:`MATERIAL_FALLBACK_BRDF` and
    ``fallback: true``, because a missing visual must never block a simulation
    load while the mass path already fails loudly upstream.
    """
    material = material or {}
    card = str(material.get("card") or "")
    diffuse = [float(v) for v in (material.get("diffuse") or (0.6, 0.6, 0.6, 1.0))]
    ambient = [float(v) for v in (material.get("ambient") or tuple(diffuse))]
    emissive = material.get("emissive")
    brdf = _match_card(card)
    if brdf is None:
        metalness, roughness = MATERIAL_FALLBACK_BRDF
        fallback = True
    else:
        metalness, roughness = brdf
        fallback = False
    return {
        "card": card,
        "albedo": _rvec(diffuse),
        "ambient": _rvec(ambient),
        "emissive": _rvec(emissive if emissive is not None else (0.0, 0.0, 0.0, 1.0)),
        "metalness": _r(metalness),
        "roughness": _r(roughness),
        "fallback": fallback,
    }


# --------------------------------------------------------------------------- #
# mass / inertia: per-body aggregation across solids
# --------------------------------------------------------------------------- #


def body_inertial(body: Dict[str, Any]) -> Dict[str, Any]:
    """Aggregate the solids of a body into one SDF ``<inertial>`` block.

    The solids of a body live in their own poses inside the body frame; each one
    carries mass, CoM and an inertia tensor around its own CoM. This folds them
    with the parallel-axis theorem into a single body-level mass, CoM and tensor
    expressed in the **body frame** and *around the body CoM*, and also keeps the
    per-solid breakdown so the multi-solid case (impossible to express in URDF,
    where one link has exactly one inertial) stays inspectable in the artefact.
    """
    solids = body.get("solids") or []
    if not solids:
        raise SdfArtifactError("body '{}' senza solidi".format(body.get("name", "?")))
    warnings: List[str] = []
    points: List[MassPoint] = []
    per_solid: List[Dict[str, Any]] = []
    for solid in solids:
        mass = float(solid.get("reference_mass_kg") or 0.0)
        rot, trans = _pose_rt(solid.get("pose"))
        com = solid.get("reference_com_m") or (0.0, 0.0, 0.0)
        inertia = solid.get("reference_inertia_kg_m2")
        if inertia is None:
            inertia = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            warnings.append(
                "solid '{}' senza tensore d'inerzia: contributo traslazionale "
                "soltanto".format(solid.get("name", "?")))
        points.append(MassPoint(mass, com, inertia, R=rot, t=trans))
        per_solid.append({
            "name": solid.get("name", ""),
            "mass_kg": _r(mass),
            "com_m": _rvec(com),
            "inertia_kg_m2": _rmat(inertia),
            "pose": _rvec(solid.get("pose") or _IDENTITY7),
        })

    mass, com, inertia = aggregate_mass_properties(points)
    if mass <= 0.0:
        raise SdfArtifactError("body '{}' con massa nulla".format(body.get("name", "?")))
    return {
        "mass_kg": _r(mass),
        "com_m": _rvec(com),
        "pose": _rvec(list(com) + [0.0, 0.0, 0.0, 1.0]),
        "inertia_kg_m2": _rmat(inertia),
        "sdf": {
            "ixx": _r(inertia[0][0]), "ixy": _r(inertia[0][1]), "ixz": _r(inertia[0][2]),
            "iyy": _r(inertia[1][1]), "iyz": _r(inertia[1][2]), "izz": _r(inertia[2][2]),
        },
        "num_solids": len(solids),
        "solids": per_solid,
        "warnings": warnings,
    }


# --------------------------------------------------------------------------- #
# frames / anchors
# --------------------------------------------------------------------------- #


def derive_frames(ir: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Return the named LCS of the model: the IR ones plus the joint ones.

    Every *activated* joint defines a joint coordinate system (JCS) on its
    parent (at ``joint.origin``) and on its child (identity). Those two LCS are
    what the emitter attaches a sensor or a jet to *by name*, so they are
    completed here for any joint that lacks them -- and only for those, so a
    frame already present in the IR is carried through untouched (and the run
    stays idempotent). Names come from :func:`cad.naming.frame_name`: the one
    naming implementation, no second variant.
    """
    frames: List[Dict[str, Any]] = []
    taken = set()
    for frame in ir.get("frames") or []:
        entry = dict(frame)
        entry["source"] = "ir"
        frames.append(entry)
        taken.add(entry.get("name"))
    for joint in ir.get("joints") or []:
        if not joint.get("activated", True):
            continue
        local = joint.get("label") or joint.get("name") or ""
        origin = joint.get("origin") or list(_IDENTITY7)
        for body, pose, role in (
            (joint.get("parent"), origin, "padre"),
            (joint.get("child"), list(_IDENTITY7), "figlio"),
        ):
            if not body:
                continue
            name = naming.frame_name(body, local)
            if name in taken:
                continue
            taken.add(name)
            frames.append({
                "name": name,
                "body": body,
                "pose": _rvec(pose),
                "description": "JCS del joint {} sul body {}".format(
                    joint.get("name", local), role),
                "source": "derived",
            })
    return frames


def derive_anchors(ir: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Return the named mount points of the model (sensors, jets, fairleads).

    An anchor is a named pose, not a body: the IR authors them (they carry the
    intent -- what mounts where) and this function carries them through,
    normalised, for the emitter. Their ``frame`` is checked against the frames
    :func:`derive_frames` returns, so the artefact is self-consistent.
    """
    frame_names = {frame.get("name") for frame in derive_frames(ir)}
    known_bodies = {body.get("name") for body in ir.get("bodies") or []}
    anchors: List[Dict[str, Any]] = []
    for anchor in ir.get("anchors") or []:
        entry = dict(anchor)
        frame = entry.get("frame")
        if frame is not None and frame not in frame_names and frame not in known_bodies:
            raise SdfArtifactError(
                "anchor '{}': frame '{}' inesistente".format(entry.get("name"), frame))
        entry["pose"] = _rvec(entry.get("pose") or _IDENTITY7)
        entry["source"] = "ir"
        anchors.append(entry)
    return anchors


# --------------------------------------------------------------------------- #
# the artefact
# --------------------------------------------------------------------------- #


def _canonical(obj: Any) -> str:
    """Canonical JSON text of the artefact (sorted keys, ``\\n``, trailing NL)."""
    return json.dumps(obj, sort_keys=True, indent=2, ensure_ascii=False) + "\n"


def _resolve_mesh(mesh_root: str, solid: Dict[str, Any], body_name: str) -> str:
    rel = solid.get("stl_path") or naming.visual_mesh_name(body_name, solid.get("name", ""))
    return os.path.join(mesh_root, rel.replace("/", os.sep))


def generate_artifacts(ir: Dict[str, Any], mesh_root: str, *,
                       write_collision: bool = False,
                       faces_max: int = MAX_COLLISION_FACES) -> Dict[str, Any]:
    """Build the full SDF-ready artefact document for ``ir``.

    ``mesh_root`` is the directory the IR's ``stl_path`` values are relative to
    (the workspace root in production). With ``write_collision`` the generated
    hulls are written next to the visual meshes, at the path
    :func:`cad.naming.collision_mesh_name` gives -- the same path the IR already
    advertises, so the emitter finds them where it expects them.

    The document contains no absolute path and no timestamp: it is a pure
    function of ``ir`` and of the meshes it points at, which is what makes the
    regeneration gate in CI a byte comparison.
    """
    from cad import ir_schema

    errors = ir_schema.collect_errors(ir)
    if errors:
        raise SdfArtifactError("IR non valido:\n  - " + "\n  - ".join(errors))
    return _generate(ir, mesh_root, write_collision=write_collision, faces_max=faces_max)


def _collision_for(path: str, rel: str, faces_max: int) -> Dict[str, Any]:
    """Fit the collision of one solid, degrading (never raising) if it cannot.

    A source mesh that is not on disk is *not* an error for the artefact: the
    collision falls back to the visual mesh path with a warning, so a partial
    checkout still produces a complete document. A mesh that is there but
    unreadable *is* an error -- that is a producer bug, not a missing input.
    """
    if not os.path.isfile(path):
        return {
            "approximation": "mesh",
            "geometry": {"type": "mesh", "mesh": rel},
            "source_mesh": rel,
            "collision_mesh": None,
            "volume_m3": 0.0,
            "hull_volume_m3": None,
            "hull_inflation": None,
            "faces": 0,
            "decimation_cell_m": None,
            "fallback": True,
            "warnings": ["mesh sorgente assente: {}".format(rel)],
        }
    return fit_collision(read_stl(path), faces_max=faces_max, source_mesh=rel)


def _write_collision(mesh_root: str, source_path: str, collision_rel: str,
                     faces_max: int) -> None:
    hull, _ = _collision_hull_mesh(read_stl(source_path), faces_max)
    if hull is None:  # pragma: no cover - guarded by the convex_hull rung
        raise SdfArtifactError("hull non disponibile per {}".format(source_path))
    write_binary_stl(os.path.join(mesh_root, collision_rel.replace("/", os.sep)),
                     hull.vertices, hull.faces)


def _content_hash(ir: Dict[str, Any]) -> str:
    """``sha256:...`` over the canonical IR, ignoring ``meta.created``.

    The artefact's identity is its input geometry, not the wall-clock moment the
    exporter stamped the IR: two exports of the same STEP an hour apart must
    hash equal, or the CI byte-identity gate would be meaningless.
    """
    import hashlib

    stripped = dict(ir)
    meta = dict(stripped.get("meta") or {})
    meta.pop("created", None)
    stripped["meta"] = meta
    return "sha256:" + hashlib.sha256(_canonical(stripped).encode("utf-8")).hexdigest()


def _assemble(ir: Dict[str, Any], mesh_root: str, artefact_bodies: List[Dict[str, Any]],
              faces_max: int) -> Dict[str, Any]:
    meta_in = ir.get("meta") or {}
    return {
        "schema": ARTIFACT_SCHEMA,
        "meta": {
            "generator": "sdf_artifacts.py",
            "emitter_version": EMITTER_VERSION,
            "units": "SI (m, kg, rad)",
            "ir_schema": ir.get("schema", ""),
            "ir_hash": _content_hash(ir),
            # the front-end's own hash of the STEP it came from, when it has one
            "ir_source_hash": meta_in.get("source_hash", ""),
            "root": meta_in.get("root"),
            "budget": {
                "max_collision_faces": int(faces_max),
                "primitive_tolerance": PRIMITIVE_TOL,
                "primitive_volume_tolerance": PRIMITIVE_VOLUME_TOL,
                "primitive_cover": PRIMITIVE_COVER,
                "max_hull_inflation": MAX_HULL_INFLATION,
            },
        },
        "bodies": artefact_bodies,
        "frames": derive_frames(ir),
        "anchors": derive_anchors(ir),
        "sensors": [dict(sensor, source="ir") for sensor in ir.get("sensors") or []],
    }


def _generate(ir: Dict[str, Any], mesh_root: str, *, write_collision: bool = False,
              faces_max: int = MAX_COLLISION_FACES) -> Dict[str, Any]:
    """Assemble the artefact; :func:`generate_artifacts` is the public wrapper."""
    artefact_bodies: List[Dict[str, Any]] = []
    for body in ir.get("bodies") or []:
        body_name = body.get("name", "")
        visuals: List[Dict[str, Any]] = []
        collisions: List[Dict[str, Any]] = []
        for solid in body.get("solids") or []:
            solid_name = solid.get("name", "")
            rel = solid.get("stl_path") or naming.visual_mesh_name(body_name, solid_name)
            path = _resolve_mesh(mesh_root, solid, body_name)
            visuals.append({
                "name": solid_name,
                "solid": solid_name,
                "mesh": rel,
                "pose": _rvec(solid.get("pose") or _IDENTITY7),
                "material": resolve_pbr_material(solid.get("material")),
            })
            collision = _collision_for(path, rel, faces_max)
            collision["name"] = solid_name
            # geometry and meshes stay in the solid-local frame; `pose` places
            # them in the body frame, so a solid that the IR rotated/translated
            # (as float__ballast is) lands where the CAD put it.
            collision["pose"] = _rvec(solid.get("pose") or _IDENTITY7)
            if write_collision and collision["approximation"] == "convex_hull" \
                    and collision["collision_mesh"]:
                _write_collision(mesh_root, path, collision["collision_mesh"], faces_max)
            collisions.append(collision)
        artefact_bodies.append({
            "name": body_name,
            "label": body.get("label", body_name),
            "pose": _rvec(body.get("pose") or _IDENTITY7),
            "inertial": body_inertial(body),
            "visuals": visuals,
            "collisions": collisions,
        })
    return _assemble(ir, mesh_root, artefact_bodies, faces_max)


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m cad.sdf_artifacts",
        description="Genera gli artefatti SDF-ready (collisioni, materiali/PBR, "
                    "inerzia, frame/anchor) dal manifest IR.")
    parser.add_argument("--manifest", required=True, help="path del manifest IR (JSON)")
    parser.add_argument("--ws", default=None,
                        help="root a cui sono relativi gli stl_path (default: dir del manifest)")
    parser.add_argument("--out", default=None,
                        help="file di uscita; '-' o assente = stdout")
    parser.add_argument("--write-collision", action="store_true",
                        help="scrive gli hull convessi accanto alle mesh visuali")
    parser.add_argument("--faces-max", type=int, default=MAX_COLLISION_FACES,
                        help="budget di facce per hull di collisione (default: %(default)s)")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    with open(args.manifest) as fh:
        ir = json.load(fh)
    mesh_root = args.ws or os.path.dirname(os.path.abspath(args.manifest))
    artefact = generate_artifacts(ir, mesh_root, write_collision=args.write_collision,
                                  faces_max=args.faces_max)
    text = _canonical(artefact)
    if args.out in (None, "-"):
        sys.stdout.write(text)
        return 0
    with open(args.out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
