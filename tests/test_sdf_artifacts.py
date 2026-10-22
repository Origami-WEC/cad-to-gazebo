"""Tests for :mod:`cad.sdf_artifacts` -- the C4 SDF-ready artefact (issue #5).

The suite is built around one golden file, ``fixtures/sdf_artifacts_sample.json``,
regenerated from ``fixtures/cad_manifest_sample_assembly.json`` +
``fixtures/meshes``. Reproducibility is therefore a *byte* comparison against a
committed document, not a shape assertion: if the generator drifts by one float
in one field, these tests fail and CI regeneration in the workflow fails too.
"""

import copy
import json
import math
from pathlib import Path

import numpy as np
import pytest
from cad import sdf_artifacts as sa

FIXTURES = Path(__file__).resolve().parent / "fixtures"
IR_PATH = FIXTURES / "cad_manifest_sample_assembly.json"
GOLDEN = FIXTURES / "sdf_artifacts_sample.json"
MESHES = FIXTURES / "meshes"


def _load(path):
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def _body(artefact, name):
    return next(b for b in artefact["bodies"] if b["name"] == name)


def _collision(artefact, body, solid):
    return next(c for c in _body(artefact, body)["collisions"] if c["name"] == solid)


def _box_soup(x0, x1, y0, y1, z0, z1):
    """Axis-aligned box as a closed triangle soup (36 vertices, 12 triangles)."""
    corners = np.array(
        [[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
         [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]],
        dtype=float,
    )
    quads = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    tris = []
    for a, b, c, d in quads:
        tris.append([corners[a], corners[b], corners[c]])
        tris.append([corners[a], corners[c], corners[d]])
    return np.asarray(tris, dtype=float)


def _soup_mesh(tris):
    verts = np.asarray(tris, dtype=float).reshape(-1, 3)
    return {"vertices": verts, "faces": np.arange(verts.shape[0]).reshape(-1, 3)}


def _run(manifest, ws, out, write_collision=False):
    argv = ["--manifest", str(manifest), "--ws", str(ws), "--out", str(out)]
    if write_collision:
        argv.append("--write-collision")
    assert sa.main(argv) == 0
    return out.read_bytes()


# --------------------------------------------------------------------------- #
# reproducibility / idempotency -- the exit criterion of C4
# --------------------------------------------------------------------------- #


def test_sample_ir_is_schema_valid():
    from cad import ir_schema

    assert ir_schema.collect_errors(_load(IR_PATH)) == []


def _sandbox_ws(root: Path) -> Path:
    """Workspace di scrittura fuori dai fixture: solo ``meshes/visual`` in link.

    I test di riproducibilita' scrivono nel workspace (anche ``meshes/collision``):
    farlo dentro ``FIXTURES`` sovrascriverebbe i file tracciati a ogni run.
    """
    ws = root / "ws"
    (ws / "meshes").mkdir(parents=True)
    (ws / "meshes" / "visual").symlink_to(MESHES / "visual")
    return ws


def test_artifact_is_reproducible_byte_for_byte(tmp_path):
    """One command, run twice, from the committed fixtures: identical bytes."""
    ws = _sandbox_ws(tmp_path)
    first = _run(IR_PATH, ws, tmp_path / "a.json", write_collision=True)
    second = _run(IR_PATH, ws, tmp_path / "b.json", write_collision=True)
    assert first == second
    assert first == GOLDEN.read_bytes()


def test_collision_mesh_is_reproducible_byte_for_byte(tmp_path):
    """Run-to-run: identical bytes in the same environment."""
    ws_a = _sandbox_ws(tmp_path / "a")
    ws_b = _sandbox_ws(tmp_path / "b")
    _run(IR_PATH, ws_a, tmp_path / "a.json", write_collision=True)
    _run(IR_PATH, ws_b, tmp_path / "b.json", write_collision=True)
    rel = "meshes/collision/keel__fin_convex.stl"
    assert (ws_a / rel).read_bytes() == (ws_b / rel).read_bytes()


def test_collision_mesh_matches_the_committed_solid(tmp_path):
    """Against the committed file: same solid, not same bytes.

    qhull's output order and its triangulation of coplanar facets change between
    scipy/qhull releases, so the committed STL is not byte-stable across machines.
    What must hold everywhere is the solid itself: same vertices, same closed
    surface, same area.
    """
    import numpy as np
    import trimesh
    from scipy.spatial import cKDTree

    ws = _sandbox_ws(tmp_path)
    _run(IR_PATH, ws, tmp_path / "a.json", write_collision=True)
    got = trimesh.load(ws / "meshes/collision/keel__fin_convex.stl", force="mesh")
    ref = trimesh.load(MESHES / "collision" / "keel__fin_convex.stl", force="mesh")
    assert got.is_watertight and ref.is_watertight
    assert got.area == pytest.approx(ref.area, rel=1e-9)
    dist, _ = cKDTree(np.asarray(ref.vertices)).query(np.asarray(got.vertices))
    assert dist.max() <= 1e-9
    assert len(got.vertices) == len(ref.vertices)


def test_committed_artifact_is_in_canonical_form():
    text = GOLDEN.read_text(encoding="utf-8")
    assert text.endswith("\n")
    assert "\t" not in text
    assert "\r" not in text
    assert text == sa._canonical(_load(GOLDEN))


def test_generation_does_not_mutate_the_ir(tmp_path):
    ir = _load(IR_PATH)
    before = copy.deepcopy(ir)
    sa.generate_artifacts(ir, str(FIXTURES))
    assert ir == before


def test_artifact_has_no_absolute_path_and_no_timestamp():
    text = GOLDEN.read_text(encoding="utf-8")
    assert str(FIXTURES) not in text
    assert "/opt/" not in text and "/home/" not in text
    meta = _load(GOLDEN)["meta"]
    assert "created" not in meta
    assert meta["ir_hash"].startswith("sha256:")


# --------------------------------------------------------------------------- #
# collision hulls: the approximation ladder and the face budget
# --------------------------------------------------------------------------- #


def test_sample_assembly_exercises_every_ladder_rung():
    artefact = _load(GOLDEN)
    got = {c["approximation"] for b in artefact["bodies"] for c in b["collisions"]}
    assert got == {"box", "cylinder", "sphere", "convex_hull"}
    assert _collision(artefact, "hull", "hull__shell")["approximation"] == "box"
    assert _collision(artefact, "pto_shaft", "pto_shaft__mass")["approximation"] == "cylinder"
    assert _collision(artefact, "float", "float__buoy")["approximation"] == "sphere"
    assert _collision(artefact, "keel", "keel__fin")["approximation"] == "convex_hull"


def test_primitive_rungs_do_not_emit_a_collision_mesh():
    artefact = _load(GOLDEN)
    for body in artefact["bodies"]:
        for col in body["collisions"]:
            if col["approximation"] in ("box", "cylinder", "sphere"):
                assert col["geometry"]["type"] == col["approximation"]
                assert col["collision_mesh"] is None
                assert col["faces"] == 0
                assert col["fallback"] is False


def test_hull_rung_respects_the_face_budget_and_records_the_decimation():
    keel = _collision(_load(GOLDEN), "keel", "keel__fin")
    assert keel["faces"] <= sa.MAX_COLLISION_FACES
    assert keel["faces"] > 0
    assert keel["collision_mesh"] == "meshes/collision/keel__fin_convex.stl"
    assert keel["decimation_cell_m"] is not None, "the dense ellipsoid must trigger decimation"
    assert keel["hull_inflation"] is not None


def test_tight_budget_still_returns_a_hull():
    ellipsoid = sa.read_stl(str(MESHES / "visual" / "keel__fin.stl"))
    record = sa.fit_collision(ellipsoid, faces_max=24, source_mesh="meshes/visual/keel__fin.stl")
    assert record["approximation"] == "convex_hull"
    assert 0 < record["faces"] < 300, "the ladder must return its smallest hull, not the raw one"
    assert record["decimation_cell_m"] is not None
    assert any("budget di 24 facce non raggiungibile" in w for w in record["warnings"])


def test_non_convex_solid_is_flagged_when_the_hull_inflates_too_much():
    # Two unit cubes meeting at a corner: the convex hull is twice the solid.
    soup = np.concatenate([_box_soup(0, 1, 0, 1, 0, 1), _box_soup(1, 2, 1, 2, 0, 1)])
    record = sa.fit_collision(_soup_mesh(soup), source_mesh="meshes/visual/step__mass.stl")
    assert record["approximation"] == "convex_hull"
    assert record["hull_inflation"] > sa.MAX_HULL_INFLATION
    assert any("over-approximazione" in w for w in record["warnings"])


def test_ladder_degrades_to_the_mesh_rung_without_qhull(monkeypatch):
    ellipsoid = sa.read_stl(str(MESHES / "visual" / "keel__fin.stl"))
    monkeypatch.setattr(sa, "_hull", lambda points: None)
    record = sa.fit_collision(ellipsoid, source_mesh="meshes/visual/keel__fin.stl")
    assert record["approximation"] == "mesh"
    assert record["fallback"] is True
    assert record["geometry"] == {"type": "mesh", "mesh": "meshes/visual/keel__fin.stl"}
    assert any("scipy assente" in w for w in record["warnings"])


def test_missing_source_mesh_degrades_instead_of_raising(tmp_path):
    ir = _load(IR_PATH)
    artefact = sa.generate_artifacts(ir, str(tmp_path))  # no meshes copied
    for body in artefact["bodies"]:
        for col in body["collisions"]:
            assert col["approximation"] == "mesh"
            assert col["fallback"] is True
            assert any("assente" in w for w in col["warnings"])


def test_stl_roundtrip_is_exact_through_the_writer(tmp_path):
    soup = _box_soup(0.0, 2.0, 0.0, 1.0, 0.0, 0.5)
    mesh = _soup_mesh(soup)
    path = tmp_path / "box.stl"
    sa.write_binary_stl(str(path), mesh["vertices"], mesh["faces"])
    back = sa.read_stl(str(path))
    assert back["vertices"].shape == mesh["vertices"].shape
    assert np.allclose(back["vertices"], mesh["vertices"], atol=1e-6)
    assert math.isclose(sa.mesh_volume(back["vertices"], back["faces"]), 1.0, rel_tol=1e-5)


# --------------------------------------------------------------------------- #
# inertia: per-body, aggregated over every solid (the multi-solid case)
# --------------------------------------------------------------------------- #


def _expected_solid_mass_body(ir, body_name):
    """Independent re-derivation of mass/CoM from the IR, with numpy only."""
    body = next(b for b in ir["bodies"] if b["name"] == body_name)
    mass = 0.0
    moment = np.zeros(3)
    for solid in body["solids"]:
        m = float(solid["reference_mass_kg"])
        com_local = np.asarray(solid["reference_com_m"], dtype=float)
        pose = np.asarray(solid.get("pose") or [0, 0, 0, 0, 0, 0, 1], dtype=float)
        rot = _quat_to_matrix(pose[3:])
        world = rot @ com_local + pose[:3]
        mass += m
        moment += m * world
    return mass, moment / mass


def _quat_to_matrix(q):
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def test_solid_pose_is_carried_into_visual_and_collision():
    """A rotated/translated solid must reach the emitter with its placement.

    `float__ballast` is authored at z=-0.45 with a 90 deg rotation about x, so
    emitting the mesh or the fitted box without that pose would put the ballast
    in the wrong place in Gazebo -- silently, because the geometry itself is
    correct.
    """
    artefact = _load(GOLDEN)
    ballast = _collision(artefact, "float", "float__ballast")
    assert ballast["pose"][2] == -0.45
    assert ballast["pose"][3:] == [0.707106781, 0.0, 0.0, 0.707106781]

    visual = next(v for v in _body(artefact, "float")["visuals"]
                  if v["name"] == "float__ballast")
    assert visual["pose"] == ballast["pose"]

    # a solid the IR did not move keeps the identity pose
    identity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    assert _collision(artefact, "hull", "hull__shell")["pose"] == identity
    assert _body(artefact, "hull")["visuals"][0]["pose"] == identity


def test_artifact_key_sets_match_the_documented_contract():
    """Pin the field sets of `docs/sdf-artifacts.md` (section 2-6).

    The doc is the contract the Gazebo emitter codes against, so a field added
    or removed here must be a deliberate edit in *both* places: this test fails
    until the doc table is updated too.
    """
    artefact = _load(GOLDEN)
    assert sorted(artefact) == ["anchors", "bodies", "frames", "meta", "schema", "sensors"]
    assert sorted(artefact["meta"]) == [
        "budget", "emitter_version", "generator", "ir_hash", "ir_schema",
        "ir_source_hash", "root", "units"]
    assert sorted(artefact["meta"]["budget"]) == [
        "max_collision_faces", "max_hull_inflation", "primitive_cover",
        "primitive_tolerance", "primitive_volume_tolerance"]

    body = _body(artefact, "hull")
    assert sorted(body) == ["collisions", "inertial", "label", "name", "pose", "visuals"]
    assert sorted(body["visuals"][0]) == ["material", "mesh", "name", "pose", "solid"]
    assert sorted(body["visuals"][0]["material"]) == [
        "albedo", "ambient", "card", "emissive", "fallback", "metalness", "roughness"]
    assert sorted(body["collisions"][0]) == [
        "approximation", "collision_mesh", "decimation_cell_m", "faces", "fallback",
        "geometry", "hull_inflation", "hull_volume_m3", "name", "pose", "source_mesh",
        "volume_m3", "warnings"]
    assert sorted(body["inertial"]) == [
        "com_m", "inertia_kg_m2", "mass_kg", "num_solids", "pose", "sdf", "solids",
        "warnings"]
    assert sorted(artefact["frames"][0]) == ["body", "description", "name", "pose", "source"]
    assert sorted(artefact["anchors"][0]) == [
        "body", "direction", "frame", "kind", "name", "pose", "source"]
    assert set(artefact["sensors"][0]) >= {
        "frame", "name", "pose", "rate_hz", "source", "topic", "type"}

    for collision in (c for b in artefact["bodies"] for c in b["collisions"]):
        assert "type" in collision["geometry"]
        assert len(collision["pose"]) == 7
        if collision["geometry"]["type"] == "mesh":
            # the mesh rung carries the path only: the accepted shape's volume is
            # `hull_volume_m3` at the record level (no analytic volume exists)
            assert set(collision["geometry"]) == {"type", "mesh"}
        else:
            assert set(collision["geometry"]) >= {"type", "cover", "volume_m3"}


def test_multi_solid_body_aggregates_mass_and_centre_of_mass():
    ir = _load(IR_PATH)
    inertial = _body(_load(GOLDEN), "float")["inertial"]
    assert inertial["num_solids"] == 2
    assert len(inertial["solids"]) == 2
    mass, com = _expected_solid_mass_body(ir, "float")
    assert math.isclose(inertial["mass_kg"], mass, rel_tol=1e-9)
    assert np.allclose(inertial["com_m"], com, atol=1e-9)
    assert inertial["com_m"][2] < 0.0, "the heavy ballast sits below the buoy centre"
    assert inertial["pose"][:3] == inertial["com_m"]
    assert inertial["pose"][3:] == [0.0, 0.0, 0.0, 1.0]


def test_multi_solid_inertia_grows_by_the_parallel_axis_term():
    ir = _load(IR_PATH)
    body = next(b for b in ir["bodies"] if b["name"] == "float")
    own = {}
    for solid in body["solids"]:
        tensor = np.asarray(solid["reference_inertia_kg_m2"], dtype=float)
        pose = np.asarray(solid.get("pose") or [0, 0, 0, 0, 0, 0, 1], dtype=float)
        rot = _quat_to_matrix(pose[3:])
        own[solid["name"]] = rot @ tensor @ rot.T
    agg = np.asarray(_body(_load(GOLDEN), "float")["inertial"]["inertia_kg_m2"])
    naive = own["float__buoy"] + own["float__ballast"]
    # both solids are offset only along z, so ixx/iyy gain m*dz^2 and izz does not
    assert agg[0][0] > naive[0][0]
    assert agg[1][1] > naive[1][1]
    assert math.isclose(agg[2][2], naive[2][2], rel_tol=1e-6)
    # the SDF block is the same tensor in the emitter's shape
    sdf = _body(_load(GOLDEN), "float")["inertial"]["sdf"]
    assert sdf["ixx"] == agg[0][0] and sdf["ixy"] == agg[0][1] and sdf["iyy"] == agg[1][1]
    assert sdf["izz"] == agg[2][2] and sdf["iyz"] == agg[1][2] and sdf["ixz"] == agg[0][2]


def test_single_solid_body_matches_its_own_reference_tensor():
    ir = _load(IR_PATH)
    body = next(b for b in ir["bodies"] if b["name"] == "pto_shaft")
    solid = body["solids"][0]
    inertial = _body(_load(GOLDEN), "pto_shaft")["inertial"]
    assert inertial["num_solids"] == 1
    assert math.isclose(inertial["mass_kg"], float(solid["reference_mass_kg"]), rel_tol=1e-9)
    assert np.allclose(inertial["inertia_kg_m2"],
                       np.asarray(solid["reference_inertia_kg_m2"], dtype=float), atol=1e-6)


def test_body_without_an_inertia_tensor_is_reported_not_guessed():
    body = {
        "name": "floater",
        "solids": [{"name": "floater__shell", "reference_mass_kg": 2.0, "reference_com_m": [0, 0, 0]}],
    }
    inertial = sa.body_inertial(body)
    assert inertial["mass_kg"] == 2.0
    assert inertial["inertia_kg_m2"] == [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    assert any("tensore" in w for w in inertial["warnings"])


# --------------------------------------------------------------------------- #
# materials / PBR
# --------------------------------------------------------------------------- #


def test_every_visual_carries_a_pbr_material_and_only_unknown_cards_fallback():
    artefact = _load(GOLDEN)
    materials = [v["material"] for b in artefact["bodies"] for v in b["visuals"]]
    assert len(materials) == 5
    fallback = [m for m in materials if m["fallback"]]
    assert [m["card"] for m in fallback] == ["mystery_composite"]
    assert fallback[0]["metalness"] == sa.MATERIAL_FALLBACK_BRDF[0]
    assert fallback[0]["roughness"] == sa.MATERIAL_FALLBACK_BRDF[1]
    assert all("albedo" in m and "emissive" in m and "ambient" in m for m in materials)


def test_metal_and_dielectric_cards_are_told_apart():
    steel = sa.resolve_pbr_material({"card": "Steel", "diffuse": [0.6, 0.6, 0.62, 1.0]})
    hdpe = sa.resolve_pbr_material({"card": "HDPE", "diffuse": [0.9, 0.9, 0.88, 1.0]})
    assert steel["fallback"] is False and steel["metalness"] >= 0.8
    assert hdpe["fallback"] is False and hdpe["metalness"] == 0.0 and hdpe["roughness"] >= 0.5
    assert steel["albedo"] == [0.6, 0.6, 0.62, 1.0]


def test_longest_keyword_wins_and_placeholder_cards_never_match():
    assert sa.resolve_pbr_material({"card": "Stainless Steel 316"})["metalness"] == 0.9
    assert sa.resolve_pbr_material({"card": "Stainless Steel 316"})["roughness"] == 0.25
    for card in ("", "default", "demo", "none", "unnamed", "material"):
        out = sa.resolve_pbr_material({"card": card})
        assert out["fallback"] is True, card
        assert out["metalness"] == 0.0 and out["roughness"] == 0.6


def test_missing_material_block_is_a_documented_default_not_a_crash():
    out = sa.resolve_pbr_material(None)
    assert out["fallback"] is True
    assert out["card"] == ""
    assert out["albedo"] == [0.6, 0.6, 0.6, 1.0]
    assert out["emissive"] == [0.0, 0.0, 0.0, 1.0]
    assert (out["metalness"], out["roughness"]) == sa.MATERIAL_FALLBACK_BRDF


# --------------------------------------------------------------------------- #
# frames / anchors / sensors
# --------------------------------------------------------------------------- #


def test_joint_frames_are_derived_on_parent_and_child():
    frames = {f["name"]: f for f in _load(GOLDEN)["frames"]}
    for joint, parent, child, origin in (
        ("pto_joint", "hull", "pto_shaft", [0.0, 0.0, 0.4]),
        ("keel_joint", "hull", "keel", [0.0, 0.0, -0.4]),
    ):
        p, c = frames[parent + "__" + joint], frames[child + "__" + joint]
        assert p["source"] == "derived" and c["source"] == "derived"
        assert p["pose"][:3] == origin and p["pose"][3:] == [0.0, 0.0, 0.0, 1.0]
        assert c["pose"] == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        assert p["body"] == parent and c["body"] == child


def test_authored_frames_are_carried_through_and_not_duplicated():
    frames = _load(GOLDEN)["frames"]
    names = [f["name"] for f in frames]
    assert len(names) == len(set(names))
    authored = [f for f in frames if f["source"] == "ir"]
    assert [f["name"] for f in authored] == ["hull__cog_hook"]
    assert "hull__cog_hook" in names


def test_extra_derived_frames_do_not_leak_back_into_the_ir():
    ir = _load(IR_PATH)
    assert "hull__pto_joint" not in {f["name"] for f in ir["frames"]}
    assert "hull__pto_joint" in {f["name"] for f in sa.derive_frames(ir)}


def test_anchors_and_sensors_point_at_a_body_or_a_frame():
    artefact = _load(GOLDEN)
    known = {f["name"] for f in artefact["frames"]} | {b["name"] for b in artefact["bodies"]}
    for anchor in artefact["anchors"]:
        assert anchor["frame"] in known
        assert anchor["source"] == "ir"
        assert len(anchor["pose"]) == 7
    assert [s["name"] for s in artefact["sensors"]] == ["imu_hull"]
    for sensor in artefact["sensors"]:
        assert sensor["frame"] in known
        assert sensor["source"] == "ir"


def test_anchor_on_an_unknown_frame_is_rejected():
    ir = _load(IR_PATH)
    ir["anchors"] = [{"name": "bad", "kind": "jet", "body": "hull", "frame": "nope__nowhere"}]
    with pytest.raises(sa.SdfArtifactError):
        sa.derive_anchors(ir)


def test_generator_rejects_an_invalid_ir_before_writing_anything(tmp_path):
    ir = _load(IR_PATH)
    del ir["bodies"][0]["solids"]
    with pytest.raises(sa.SdfArtifactError):
        sa.generate_artifacts(ir, str(FIXTURES))


def test_cli_prints_to_stdout_when_no_out_is_given(capsys):
    assert sa.main(["--manifest", str(IR_PATH), "--ws", str(FIXTURES)]) == 0
    assert json.loads(capsys.readouterr().out)["schema"] == sa.ARTIFACT_SCHEMA
