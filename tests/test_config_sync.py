"""Unit tests for cad.config_sync -- pure config reconciliation from the CAD IR.

Runs without FreeCAD. Covers the mass/joint/mooring reconciliation logic, the
v1 IR gate (``load_ir``) and the end-to-end ``sync_from_manifest`` on a
throwaway workspace.
"""

import json

import pytest
from cad import config_sync as cs
from cad import ir_schema


def _pose(x=0.0, y=0.0, z=0.0):
    return [x, y, z, 0.0, 0.0, 0.0, 1.0]


def _solid(name, mass):
    """A valid v1 solid: unique name inside the body, pose, mass, material."""
    return {"name": name, "pose": _pose(), "density_kg_m3": 1000.0,
            "reference_mass_kg": mass, "material": {"card": "Steel"}}


def _meta(root="hull"):
    return {
        "generator": "tests/test_config_sync.py",
        "emitter_version": "cad-to-gazebo/0.0.0",
        "device_type": "test_fixture",
        "source_hash": "sha256:" + "0" * 64,
        "units": "SI (m, kg, rad)",
        "created": "2026-01-01T00:00:00Z",
        "root": root,
    }


def _limits(lower=0.0, upper=1.0, unit="m", offset=0.0,
            lower_enabled=True, upper_enabled=True):
    return {
        "lower": lower, "upper": upper,
        "lower_enabled": lower_enabled, "upper_enabled": upper_enabled,
        "unit": unit, "offset_m": offset,
    }


def _manifest():
    """A valid v1 CAD IR document (``schema`` + ``meta`` + bodies + joints)."""
    return {
        "schema": ir_schema.IR_SCHEMA,
        "meta": _meta("hull"),
        "bodies": [
            {"name": "hull", "label": "Hull", "pose": _pose(),
             "solids": [_solid("hull__shell", 100.0),
                        _solid("hull__ballast", 50.0)]},
            {"name": "flap", "label": "Flap", "pose": _pose(),
             "solids": [_solid("flap__plate", 10.0)]},
        ],
        "joints": [
            {"name": "j_hinge", "fc_type": "Revolute", "activated": True,
             "parent": "hull", "child": "flap",
             "axis": [0.0, 0.0, 1.0], "limits": _limits(unit="rad")},
            {"name": "j_weld", "fc_type": "Fixed", "activated": True,
             "parent": "hull", "child": "flap", "axis": [0.0, 0.0, 1.0],
             "limits": _limits()},
            {"name": "j_dist", "fc_type": "Distance", "activated": True,
             "parent": "hull", "child": "flap", "axis": [0.0, 0.0, 1.0],
             "limits": _limits()},
            {"name": "j_off", "fc_type": "Slider", "activated": False,
             "parent": "hull", "child": "flap", "axis": [1.0, 0.0, 0.0],
             "limits": _limits()},
        ],
    }


def _legacy_manifest():
    """A pre-v1 three-key manifest: `meta` + flat `links` + `joints`."""
    return {
        "meta": {"root": "hull"},
        "links": [{"name": "hull", "reference_mass_kg": 42.0}],
        "joints": [],
    }


# --- IR gate (load_ir) -------------------------------------------------------

def test_load_ir_accepts_v1_document():
    ir = cs.load_ir(ir=_manifest())
    assert ir["schema"] == ir_schema.IR_SCHEMA


def test_load_ir_rejects_legacy_manifest():
    with pytest.raises(ir_schema.IRSchemaError) as excinfo:
        cs.load_ir(ir=_legacy_manifest())
    assert "--migrate" in str(excinfo.value)


def test_load_ir_migrates_legacy_only_on_request():
    ir = cs.load_ir(ir=_legacy_manifest(), allow_legacy=True)
    assert ir["schema"] == ir_schema.IR_SCHEMA
    assert [b["name"] for b in ir["bodies"]] == ["hull"]


def test_load_ir_rejects_broken_v1_document():
    bad = _manifest()
    bad["bodies"][0]["solids"][0].pop("reference_mass_kg")
    with pytest.raises(ir_schema.IRSchemaError):
        cs.load_ir(ir=bad)


def test_load_ir_reads_and_validates_a_file(tmp_path):
    path = tmp_path / "cad_manifest.json"
    path.write_text(json.dumps(_manifest()))
    assert cs.load_ir(path=str(path))["meta"]["root"] == "hull"


def test_load_ir_missing_file_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        cs.load_ir(path=str(tmp_path / "nope.json"))


# --- total_mass_kg / body_names ---------------------------------------------

def test_total_mass_kg_bodies():
    assert cs.total_mass_kg(_manifest()) == pytest.approx(160.0)


def test_total_mass_kg_ignores_links_residue():
    """A v1 document may still carry `links[]` for old readers: we ignore it."""
    ir = _manifest()
    ir["links"] = [{"name": "hull", "reference_mass_kg": 42.0}]
    assert cs.total_mass_kg(ir) == pytest.approx(160.0)


def test_body_names_ignores_links_residue():
    ir = _manifest()
    ir["links"] = [{"name": "ballast"}]
    assert cs.body_names(ir) == ["hull", "flap"]


# --- movable_joints ----------------------------------------------------------

def test_movable_joints_only_real_dofs():
    joints = cs.movable_joints(_manifest())
    names = [j["name"] for j in joints]
    # Revolute is a real DOF; Fixed merges; Distance unmapped; Slider deactivated
    assert names == ["j_hinge"]


# --- _merge_dynamics ---------------------------------------------------------

def test_merge_dynamics_defaults():
    merged = cs._merge_dynamics(None)
    assert merged == cs.DEFAULT_JOINT_DYNAMICS


def test_merge_dynamics_preserves_values():
    merged = cs._merge_dynamics({"damping": 3.0})
    assert merged["damping"] == 3.0
    assert merged["friction"] == 0.0


def test_merge_dynamics_replaces_legacy_zero():
    merged = cs._merge_dynamics({"limit_stiffness": 0.0})
    # 0.0 was the pre-2026 "disabled" default -> replaced by the new value
    assert merged["limit_stiffness"] == cs.DEFAULT_JOINT_DYNAMICS["limit_stiffness"]


# --- _limits_snapshot / _sdf_type -------------------------------------------

def test_limits_snapshot_roundtrip():
    snap = cs._limits_snapshot({"limits": _limits(lower=-0.5, upper=0.5)})
    assert snap["lower"] == pytest.approx(-0.5)
    assert snap["upper"] == pytest.approx(0.5)
    assert snap["unit"] == "m"


def test_sdf_type_revolute():
    assert cs._sdf_type({"fc_type": "Revolute"}) == "revolute"
    assert cs._sdf_type({"fc_type": "Slider"}) == "prismatic"


# --- sync_pipeline_joints ----------------------------------------------------

def test_sync_pipeline_joints_regenerates():
    report = cs.SyncReport()
    pipeline = {"model": {}}
    changed = cs.sync_pipeline_joints(_manifest(), pipeline, report)
    assert changed is True
    joints = pipeline["model"]["joints"]
    assert [j["name"] for j in joints] == ["j_hinge"]
    assert joints[0]["type"] == "revolute"
    # a dynamics entry exists for the movable joint only
    assert "j_hinge" in pipeline["model"]["joint_dynamics"]
    assert "j_weld" not in pipeline["model"]["joint_dynamics"]


def test_sync_pipeline_joints_preserves_dynamics():
    report = cs.SyncReport()
    pipeline = {"model": {"joint_dynamics": {"j_hinge": {"damping": 9.0}}}}
    cs.sync_pipeline_joints(_manifest(), pipeline, report)
    assert pipeline["model"]["joint_dynamics"]["j_hinge"]["damping"] == 9.0
    assert any("j_hinge" in p for p in report.preserved)


def test_sync_pipeline_joints_warns_on_removed_dynamics():
    report = cs.SyncReport()
    pipeline = {"model": {"joint_dynamics": {"ghost": {"damping": 1.0}}}}
    cs.sync_pipeline_joints(_manifest(), pipeline, report)
    assert any("ghost" in w for w in report.warnings)


# --- sync_root_link ----------------------------------------------------------

def test_sync_root_link_sets_and_preserves():
    report = cs.SyncReport()
    pipeline = {"model": {}}
    cs.sync_root_link(_manifest(), pipeline, report)
    assert pipeline["model"]["root_link"] == "hull"
    # second run preserves
    report2 = cs.SyncReport()
    cs.sync_root_link(_manifest(), pipeline, report2)
    assert "pipeline.model.root_link" in report2.preserved


# --- sync_enable_links -------------------------------------------------------

def test_sync_enable_links_drops_unknown():
    report = cs.SyncReport()
    hydro = {"enable_links": ["hull", "ghost"]}
    cs.sync_enable_links(_manifest(), hydro, report)
    assert hydro["enable_links"] == ["hull"]
    assert any("ghost" in w for w in report.warnings)


def test_sync_enable_links_noop_when_manifest_empty():
    report = cs.SyncReport()
    hydro = {"enable_links": ["hull"]}
    cs.sync_enable_links({}, hydro, report)
    assert hydro["enable_links"] == ["hull"]


# --- derived_break_tension / update_mooring_derived -------------------------

def test_derived_break_tension_formula():
    mooring = {}
    # default k=2.5, g=9.81, total mass 160 kg
    expected = 2.5 * 160.0 * 9.81
    assert cs.derived_break_tension(_manifest(), mooring) == pytest.approx(expected)


def test_update_mooring_derived_writes():
    report = cs.SyncReport()
    mooring = {}
    cs.update_mooring_derived(_manifest(), mooring, report, write=True)
    assert mooring["break_tension_n"] == pytest.approx(2.5 * 160.0 * 9.81, rel=1e-3)
    assert any("break_tension_n" in u for u in report.updated)


def test_update_mooring_derived_dry_run_warns_when_desynced():
    report = cs.SyncReport()
    mooring = {"break_tension_n": 1.0}  # far below the heuristic
    cs.update_mooring_derived(_manifest(), mooring, report, write=False)
    assert any("desincronizzato" in w for w in report.warnings)


def test_update_mooring_derived_zero_mass_warns():
    report = cs.SyncReport()
    mooring = {}
    cs.update_mooring_derived({"bodies": [{"name": "h", "solids": []}]},
                              mooring, report, write=True)
    assert any("massa totale nulla" in w for w in report.warnings)


# --- validate_joint_dynamics -------------------------------------------------

def test_validate_joint_dynamics_spring_out_of_range():
    report = cs.SyncReport()
    pipeline = {"model": {"joint_dynamics": {"j_hinge": {"spring_reference": 99.0}}}}
    cs.validate_joint_dynamics(_manifest(), pipeline, report)
    assert any("spring_reference" in w for w in report.warnings)


def test_validate_joint_dynamics_in_range_ok():
    report = cs.SyncReport()
    pipeline = {"model": {"joint_dynamics": {"j_hinge": {"spring_reference": 0.5}}}}
    cs.validate_joint_dynamics(_manifest(), pipeline, report)
    assert report.warnings == []


# --- sync_from_manifest integration -----------------------------------------

def _seed_ws(tmp_path):
    ws = tmp_path / "ws"
    cfg = ws / "config"
    cfg.mkdir(parents=True)
    manifest = _manifest()
    (cfg / "cad_manifest.json").write_text(json.dumps(manifest))
    (cfg / "pipeline.json").write_text(json.dumps({"model": {}}))
    (cfg / "hydrodynamics_params.json").write_text(
        json.dumps({"enable_links": ["hull", "ghost"]}))
    (cfg / "mooring.json").write_text(json.dumps({"lines": []}))
    return ws, manifest


def test_sync_from_manifest_end_to_end(tmp_path):
    ws, manifest = _seed_ws(tmp_path)
    report = cs.sync_from_manifest(manifest=manifest, ws=str(ws), write=True)
    pipeline = json.loads((ws / "config" / "pipeline.json").read_text())
    hydro = json.loads((ws / "config" / "hydrodynamics_params.json").read_text())
    assert pipeline["model"]["root_link"] == "hull"
    assert [j["name"] for j in pipeline["model"]["joints"]] == ["j_hinge"]
    assert hydro["enable_links"] == ["hull"]
    assert isinstance(report, cs.SyncReport)


def test_sync_from_manifest_is_idempotent(tmp_path):
    ws, manifest = _seed_ws(tmp_path)
    cs.sync_from_manifest(manifest=manifest, ws=str(ws), write=True)
    first = (ws / "config" / "pipeline.json").read_text()
    report2 = cs.SyncReport()
    cs.sync_from_manifest(manifest=manifest, ws=str(ws), write=True,
                          report=report2)
    second = (ws / "config" / "pipeline.json").read_text()
    assert first == second
    assert "pipeline.model.joints" in report2.preserved


def test_sync_from_manifest_missing_manifest_raises(tmp_path):
    ws = tmp_path / "empty"
    (ws / "config").mkdir(parents=True)
    with pytest.raises(FileNotFoundError):
        cs.sync_from_manifest(ws=str(ws))
