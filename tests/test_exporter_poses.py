"""Pose dei solidi nell'export FreeCAD (maritime_ws #33, bug B1).

Un assieme sintetico con due componenti in posa non banale (uno ``App::Link``
verso un documento esterno, uno ``App::Part`` diretto) viene esportato con il
vero ``freecad_exporter.py`` sotto ``freecadcmd``. Il CoM globale atteso e'
calcolato qui in modo indipendente (matrici di rotazione, non FreeCAD) dalle
pose note del CAD:

    CoM_mondo = pose_body * CoM_locale_manifest = R_body @ CoM_locale + t_body

Prima del fix, per i componenti ``App::Link`` il frame dei solidi ignorava la
trasformazione del link: tutti i CoM ricadevano nell'origine del documento
linkato.

Si salta con motivo esplicito se manca freecadcmd o ``FREECAD_EXTRA_SITE``.
"""

from __future__ import annotations

import json
import os
import struct
import subprocess

import numpy as np
import pytest
from test_real_freecad_export import FREECADCMD, ROOT, _run_export, _skip_reason

GENERATOR = ROOT / "tests" / "fixtures" / "make_pose_assembly.py"
TOL_M = 1e-6

LINKED_BOX = {"size_mm": [100.0, 200.0, 300.0], "density_kg_m3": 1000.0,
              "placement": {"pos_mm": [10.0, 20.0, 30.0], "axis": [0, 0, 1],
                            "angle_deg": 90.0}}
DIRECT_BOX = {"size_mm": [400.0, 100.0, 50.0], "density_kg_m3": 2000.0,
              "placement": {"pos_mm": [-5.0, 7.0, 11.0], "axis": [1, 0, 0],
                            "angle_deg": 30.0}}
LINK_PLACEMENT = {"pos_mm": [1000.0, 0.0, 0.0], "axis": [1, 0, 0], "angle_deg": 90.0}
DIRECT_PLACEMENT = {"pos_mm": [0.0, -500.0, 200.0], "axis": [0, 1, 0], "angle_deg": 30.0}
SPEC = {"linked_box": LINKED_BOX, "direct_box": DIRECT_BOX,
        "link_placement": LINK_PLACEMENT, "direct_placement": DIRECT_PLACEMENT}


def _rot(axis, angle_deg):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    a = np.radians(angle_deg)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(a) * k + (1 - np.cos(a)) * (k @ k)


def _quat_rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _expected_world_com_m(comp_placement, box):
    """CoM del box nel mondo [m], dalle sole pose note [mm, gradi]."""
    r_box = _rot(box["placement"]["axis"], box["placement"]["angle_deg"])
    local_com_mm = np.asarray(box["size_mm"], dtype=float) / 2.0
    in_comp = r_box @ local_com_mm + np.asarray(box["placement"]["pos_mm"])
    r_comp = _rot(comp_placement["axis"], comp_placement["angle_deg"])
    return (r_comp @ in_comp + np.asarray(comp_placement["pos_mm"])) * 1e-3


def _stl_vertices(path):
    data = open(path, "rb").read()
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)),
                                              ("a", "<u2")]), count=n, offset=84)
    return rec["v"].reshape(-1, 3).astype(float)


@pytest.fixture(scope="module")
def exported(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("pose_assembly")
    reason = _skip_reason(GENERATOR)
    if reason:
        pytest.skip(reason)
    spec_path = tmp / "spec.json"
    spec_path.write_text(json.dumps(SPEC), encoding="utf-8")
    gen = subprocess.run([FREECADCMD, str(GENERATOR)], capture_output=True, text=True,
                         timeout=300, env=dict(os.environ, GEN_SPEC=str(spec_path),
                                               GEN_OUT=str(tmp)))
    asm = tmp / "asm.FCStd"
    assert "Exception while processing" not in gen.stdout + gen.stderr, \
        (gen.stdout + gen.stderr)[-2000:]
    assert asm.is_file(), "generatore FreeCAD non ha prodotto asm.FCStd"
    ws = _run_export(asm, tmp)
    manifest = ws / "config" / "cad_manifest.json"
    assert manifest.is_file(), "export non ha prodotto cad_manifest.json"
    return ws, json.loads(manifest.read_text(encoding="utf-8"))


def _world_com(body, solid):
    r = _quat_rot(body["pose"][3:7])
    return r @ np.asarray(solid["reference_com_m"]) + np.asarray(body["pose"][:3])


@pytest.mark.parametrize("body_name,placement,box", [
    ("linked_body", LINK_PLACEMENT, LINKED_BOX),
    ("direct_body", DIRECT_PLACEMENT, DIRECT_BOX),
])
def test_world_com_matches_cad(exported, body_name, placement, box):
    _, manifest = exported
    body = next(b for b in manifest["bodies"] if b["name"] == body_name)
    assert len(body["solids"]) == 1
    got = _world_com(body, body["solids"][0])
    np.testing.assert_allclose(got, _expected_world_com_m(placement, box),
                               atol=TOL_M)


def test_body_pose_is_component_placement(exported):
    _, manifest = exported
    body = next(b for b in manifest["bodies"] if b["name"] == "linked_body")
    np.testing.assert_allclose(body["pose"][:3],
                               np.asarray(LINK_PLACEMENT["pos_mm"]) * 1e-3, atol=TOL_M)


@pytest.mark.parametrize("body_name,box", [("linked_body", LINKED_BOX),
                                           ("direct_body", DIRECT_BOX)])
def test_mesh_is_in_same_frame_as_com(exported, body_name, box):
    """Il centro del bounding box della mesh (box simmetrico) = CoM locale."""
    ws, manifest = exported
    body = next(b for b in manifest["bodies"] if b["name"] == body_name)
    solid = body["solids"][0]
    verts = _stl_vertices(ws / solid["stl_path"])
    center = (verts.min(axis=0) + verts.max(axis=0)) / 2.0
    np.testing.assert_allclose(center, solid["reference_com_m"], atol=1e-5)
