"""Contract tests for the IR v1 schema + fail-fast validator (issue #4, C3).

The fixtures are real producer output: ``cad_manifest_demo.json`` is written by
``create_demo_model.py`` through the new pre-write gate (regenerated in
``test_demo_generator_end_to_end``); ``cad_manifest_full.json`` is the same
document plus the LCS/anchor/sensor sections; ``cad_manifest_legacy.json`` is a
pre-v1 manifest as the pre-C3 exporter wrote it.
"""

import copy
import importlib.util
import json
import subprocess
import sys
import types
from pathlib import Path

import pytest
from cad import ir_schema
from jsonschema import Draft202012Validator

ROOT = Path(__file__).resolve().parent.parent
FIXTURES = Path(__file__).resolve().parent / "fixtures"
DEMO = FIXTURES / "cad_manifest_demo.json"
FULL = FIXTURES / "cad_manifest_full.json"
LEGACY = FIXTURES / "cad_manifest_legacy.json"


def _load(path):
    return json.loads(Path(path).read_text())


@pytest.fixture()
def demo():
    return _load(DEMO)


def _errors(ir):
    return ir_schema.collect_errors(ir)


def _joined(errors):
    return "\n".join(errors)


# --------------------------------------------------------------------------- #
# the schema document itself
# --------------------------------------------------------------------------- #


def test_schema_is_a_valid_draft202012_schema():
    Draft202012Validator.check_schema(ir_schema.load_schema())


def test_schema_path_is_shipped_next_to_the_package():
    assert Path(ir_schema.SCHEMA_PATH).is_file()
    assert Path(ir_schema.SCHEMA_PATH).parent.name == "schema"


def test_schema_declares_every_ir_section():
    schema = ir_schema.load_schema()
    for section in ("meta", "bodies", "joints", "frames", "anchors", "sensors"):
        assert section in schema["properties"], section
    meta_required = schema["$defs"]["meta"]["required"]
    for key in ("source_hash", "emitter_version", "device_type"):
        assert key in meta_required


def test_schema_tag_is_the_module_constant():
    assert ir_schema.load_schema()["properties"]["schema"]["const"] == ir_schema.IR_SCHEMA


def test_version_constants():
    assert ir_schema.IR_SCHEMA == "cad-to-gazebo/cad-ir/1"
    assert ir_schema.emitter_version() == "cad-to-gazebo/{}".format(
        ir_schema.PACKAGE_VERSION)


def test_package_version_matches_pyproject():
    """emitter_version() must not drift from the released version."""
    pyproject = (ROOT / "pyproject.toml").read_text(encoding="utf-8")
    assert 'version = "{}"'.format(ir_schema.PACKAGE_VERSION) in pyproject


def test_readme_pins_the_contract_documents():
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert "docs/naming-contract.md" in readme
    assert "docs/ir-v1.md" in readme
    assert "schema/cad_ir_v1.schema.json" in readme


def test_documents_exist():
    assert (ROOT / "docs" / "naming-contract.md").is_file()
    assert (ROOT / "docs" / "ir-v1.md").is_file()


# --------------------------------------------------------------------------- #
# the produced manifests validate  (exit criterion: no regression)
# --------------------------------------------------------------------------- #


def test_demo_manifest_validates():
    assert _errors(_load(DEMO)) == []
    ir_schema.validate_file(str(DEMO))


def test_full_manifest_validates_every_section():
    ir = _load(FULL)
    ir_schema.validate(ir)
    assert ir["frames"] and ir["anchors"] and ir["sensors"]


def test_all_fixture_names_are_valid_ir_names():
    ir = _load(FULL)
    names = []
    for body in ir["bodies"]:
        names.append(body["name"])
        names.extend(s["name"] for s in body["solids"])
    names.extend(j["name"] for j in ir["joints"])
    names.extend(f["name"] for f in ir.get("frames", []))
    names.extend(a["name"] for a in ir.get("anchors", []))
    names.extend(s["name"] for s in ir.get("sensors", []))
    assert names and all(naming_ok(n) for n in names)


def naming_ok(name):
    from cad import naming

    return naming.is_valid_name(name)


def test_empty_optional_sections_are_accepted(demo):
    for section in ("frames", "anchors", "sensors"):
        demo[section] = []
    ir_schema.validate(demo)
    del demo["anchors"]
    ir_schema.validate(demo)


# --------------------------------------------------------------------------- #
# invalid IR is rejected, with the reason  (exit criterion)
# --------------------------------------------------------------------------- #

BAD_CASES = []


def _bad(case_id, mutate, expected):
    BAD_CASES.append(pytest.param(mutate, expected, id=case_id))


_bad("missing-schema-tag", lambda ir: ir.pop("schema"), "<root>")
_bad("missing-emitter-version",
     lambda ir: ir["meta"].pop("emitter_version"), "meta")
_bad("undocumented-meta-key",
     lambda ir: ir["meta"].__setitem__("mystery", 1), "meta")
_bad("short-pose", lambda ir: ir["bodies"][0].__setitem__("pose", [0.0] * 6),
     "bodies[0].pose")
_bad("non-unit-quaternion",
     lambda ir: ir["bodies"][1].__setitem__("pose", [0, 0, 0, 0, 0, 0, 2.0]),
     "bodies[1].pose: quaternione non unitario")
_bad("bad-name", lambda ir: ir["bodies"][0].__setitem__("name", "Hull Shell!"),
     "bodies[0].name")
_bad("duplicate-body", lambda ir: ir["bodies"].append(copy.deepcopy(ir["bodies"][0])),
     "bodies: nome duplicato 'hull'")
_bad("negative-mass",
     lambda ir: ir["bodies"][0]["solids"][0].__setitem__("reference_mass_kg", -1.0),
     "bodies[0].solids[0].reference_mass_kg")
_bad("no-solids", lambda ir: ir["bodies"][0].__setitem__("solids", []),
     "bodies[0].solids")
_bad("num-solids-mismatch",
     lambda ir: ir["bodies"][1].__setitem__("num_solids", 7),
     "bodies[1].num_solids")
_bad("unknown-joint-parent",
     lambda ir: ir["joints"][0].__setitem__("parent", "ghost"), "joints[0].parent")
_bad("unknown-meta-root", lambda ir: ir["meta"].__setitem__("root", "ghost"),
     "meta.root")
_bad("null-axis", lambda ir: ir["joints"][0].__setitem__("axis", [0.0, 0.0, 0.0]),
     "joints[0].axis")
_bad("inverted-limits", lambda ir: ir["joints"][0]["limits"].__setitem__("lower", 9.0),
     "joints[0].limits")
_bad("bad-limit-unit", lambda ir: ir["joints"][0]["limits"].__setitem__("unit", "mm"),
     "joints[0].limits")
_bad("bad-sensor-frame",
     lambda ir: ir["sensors"][0].__setitem__("frame", "ghost"), "sensors[0].frame")
_bad("bad-anchor-kind",
     lambda ir: ir["anchors"][0].__setitem__("kind", "boom"), "anchors[0].kind")
_bad("bad-frame-body",
     lambda ir: ir["frames"][0].__setitem__("body", "ghost"), "frames[0].body")
_bad("skipped-joint-without-reason",
     lambda ir: ir.__setitem__("skipped_joints", [{"label": "Frame001", "reason": ""}]),
     "skipped_joints[0].reason")
_bad("empty-document", lambda ir: ir.clear(), "schema")


@pytest.mark.parametrize("mutate,expected", BAD_CASES)
def test_invalid_manifest_is_rejected_with_a_reason(full, mutate, expected):
    ir = copy.deepcopy(full)
    mutate(ir)
    errors = _errors(ir)
    assert errors, "il validatore ha accettato un IR non valido"
    assert expected in _joined(errors), _joined(errors)
    with pytest.raises(ir_schema.IRSchemaError) as excinfo:
        ir_schema.validate(ir)
    assert excinfo.value.errors == errors


@pytest.fixture()
def full():
    return _load(FULL)


def test_error_report_lists_every_violation(full):
    """The semantic pass reports everything at once, not one problem per run."""
    ir = copy.deepcopy(full)
    ir["joints"][0]["child"] = "ghost"
    ir["joints"][0]["axis"] = [0.0, 0.0, 0.0]
    ir["bodies"][0]["solids"].append(copy.deepcopy(ir["bodies"][0]["solids"][0]))
    errors = _errors(ir)
    assert len(errors) >= 3
    assert "joints[0].child" in _joined(errors)
    assert "joints[0].axis" in _joined(errors)
    assert "bodies[0].solids: nome duplicato 'hull__shell'" in _joined(errors)


def test_non_object_document_is_rejected():
    assert _errors([]) == ["<root>: il documento IR deve essere un oggetto JSON"]
    assert _errors(None)


def test_semantic_pass_is_skipped_when_structure_is_broken():
    """fail-fast: a structurally broken document reports structure, not noise."""
    errors = _errors({"meta": {}})
    assert all("is a required property" in e or "should be" in e
               for e in errors), errors


# --------------------------------------------------------------------------- #
# migration of a pre-v1 manifest  (deliverable 4)
# --------------------------------------------------------------------------- #


def test_legacy_manifest_is_not_v1():
    legacy = _load(LEGACY)
    assert ir_schema.looks_legacy(legacy)
    assert "schema" in _joined(_errors(legacy))
    with pytest.raises(ir_schema.IRSchemaError):
        ir_schema.validate_file(str(LEGACY))


def test_migrate_legacy_produces_a_valid_v1_document():
    migrated = ir_schema.migrate_legacy(_load(LEGACY))
    ir_schema.validate(migrated)
    assert not ir_schema.looks_legacy(migrated)
    assert "links" not in migrated
    assert [b["name"] for b in migrated["bodies"]] == ["spar", "float"]
    assert migrated["meta"]["emitter_version"] == ir_schema.emitter_version()
    assert migrated["meta"]["device_type"] == "generic"
    assert migrated["meta"]["source_hash"].startswith("sha256:")
    assert migrated["meta"]["tessellation_tolerance_mm"] == 2.0
    assert migrated["meta"]["root"] == "spar"
    assert migrated["joints"] == _load(LEGACY)["joints"]
    assert migrated["skipped_joints"] == _load(LEGACY)["skipped_joints"]


def test_migrate_legacy_folds_a_flat_link_into_one_solid():
    migrated = ir_schema.migrate_legacy(_load(LEGACY))
    spar = migrated["bodies"][0]
    assert len(spar["solids"]) == 1
    solid = spar["solids"][0]
    assert solid["name"] == "spar"
    assert solid["density_kg_m3"] == 7850.0
    assert solid["reference_mass_kg"] == 1200.0
    assert solid["material"]["card"] == "steel"
    assert solid["pose"] == spar["pose"]


def test_migrate_legacy_keeps_existing_solids():
    migrated = ir_schema.migrate_legacy(_load(LEGACY))
    float_body = migrated["bodies"][1]
    assert [s["name"] for s in float_body["solids"]] == ["float__shell"]


def test_migrate_legacy_does_not_mutate_its_input():
    legacy = _load(LEGACY)
    snapshot = copy.deepcopy(legacy)
    ir_schema.migrate_legacy(legacy)
    assert legacy == snapshot


def test_migrate_legacy_is_a_no_op_on_v1():
    ir = ir_schema.migrate_legacy(_load(DEMO))
    ir_schema.validate(ir)
    assert ir["schema"] == ir_schema.IR_SCHEMA
    assert [b["name"] for b in ir["bodies"]] == ["hull", "pto_shaft"]


def test_migrate_legacy_rejects_duplicate_names():
    legacy = _load(LEGACY)
    legacy["links"].append(copy.deepcopy(legacy["links"][0]))
    with pytest.raises(ir_schema.IRSchemaError) as excinfo:
        ir_schema.migrate_legacy(legacy)
    assert "duplicati" in str(excinfo.value)


def test_validate_file_allow_legacy_migrates_in_memory(tmp_path):
    ir = ir_schema.validate_file(str(LEGACY), allow_legacy=True)
    assert ir["schema"] == ir_schema.IR_SCHEMA
    assert _load(LEGACY).get("schema") is None  # the file on disk is untouched


def test_validate_file_reports_unreadable_and_malformed(tmp_path):
    with pytest.raises(ir_schema.IRSchemaError) as excinfo:
        ir_schema.validate_file(str(tmp_path / "nope.json"))
    assert "non leggibile" in str(excinfo.value)
    broken = tmp_path / "broken.json"
    broken.write_text("{not json")
    with pytest.raises(ir_schema.IRSchemaError) as excinfo:
        ir_schema.validate_file(str(broken))
    assert "non e' JSON valido" in str(excinfo.value)


# --------------------------------------------------------------------------- #
# the export gate  (deliverable 2: validator runs on every export)
# --------------------------------------------------------------------------- #


def test_export_gate_accepts_v1_and_legacy_and_rejects_broken(tmp_path, capsys):
    from cad import export_manifest

    good = tmp_path / "good.json"
    good.write_text(json.dumps(_load(FULL)))
    assert export_manifest.validate_manifest(str(good)) is True

    legacy = tmp_path / "legacy.json"
    legacy.write_text(json.dumps(_load(LEGACY)))
    assert export_manifest.validate_manifest(str(legacy)) is True
    assert "pre-v1" in capsys.readouterr().out

    broken = _load(FULL)
    broken["joints"][0]["child"] = "ghost"
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps(broken))
    assert export_manifest.validate_manifest(str(bad)) is False
    assert "joints[0].child" in capsys.readouterr().out


def test_export_manifest_runs_the_gate_before_syncing():
    source = (ROOT / "export_manifest.py").read_text(encoding="utf-8")
    assert "validate_manifest(manifest_path)" in source
    assert source.index("validate_manifest(manifest_path)") < source.index(
        "sync_configs(ws,")


def test_exporter_validates_before_writing():
    """freecad_exporter: the pre-write gate, not just the runner."""
    source = (ROOT / "freecad_exporter.py").read_text(encoding="utf-8")
    assert "_validate_ir(manifest)" in source
    assert source.index("_validate_ir(manifest)") < source.index(
        'os.path.join(ws, "config", "cad_manifest.json")')


# --------------------------------------------------------------------------- #
# the demo generator, end to end through the gate
# --------------------------------------------------------------------------- #


def test_demo_generator_end_to_end(tmp_path, monkeypatch):
    pytest.importorskip("trimesh")
    monkeypatch.setitem(sys.modules, "ws_root", types.SimpleNamespace(
        workspace_root=lambda: str(tmp_path)))
    path = ROOT / "create_demo_model.py"
    spec = importlib.util.spec_from_file_location("cad_demo_under_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert module.create_demo_model(force=True) == 0
    produced = json.loads((tmp_path / "config" / "cad_manifest.json").read_text())
    ir_schema.validate(produced)
    assert produced["schema"] == ir_schema.IR_SCHEMA
    assert produced["meta"]["generator"] == "create_demo_model.py"

    shipped = _load(DEMO)
    assert sorted(b["name"] for b in produced["bodies"]) == sorted(
        b["name"] for b in shipped["bodies"])
    assert [j["name"] for j in produced["joints"]] == [
        j["name"] for j in shipped["joints"]]
    # the shipped fixture must stay the shape the generator produces
    assert produced["meta"]["device_type"] == shipped["meta"]["device_type"]
    assert produced["bodies"][0]["solids"][0]["material"] == \
        shipped["bodies"][0]["solids"][0]["material"]


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #


def test_cli_main_ok_and_failure(tmp_path, capsys):
    assert ir_schema.main([str(DEMO), str(FULL)]) == 0
    out = capsys.readouterr().out
    assert out.count("OK") == 2
    assert ir_schema.main([]) == 1
    broken = _load(DEMO)
    broken["joints"][0]["parent"] = "ghost"
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps(broken))
    assert ir_schema.main([str(bad)]) == 1
    assert "joints[0].parent" in capsys.readouterr().err


def test_cli_allow_legacy_and_migrate(tmp_path, capsys):
    assert ir_schema.main([str(LEGACY)]) == 1
    capsys.readouterr()
    assert ir_schema.main(["--allow-legacy", str(LEGACY)]) == 0
    out = tmp_path / "migrated.json"
    assert ir_schema.main(["--allow-legacy", "--migrate", str(out), str(LEGACY)]) == 0
    ir_schema.validate_file(str(out))
    assert ir_schema.main(["--migrate", str(out), str(DEMO), str(FULL)]) == 1


def test_cli_print_schema():
    parser = ir_schema.build_parser()
    args = parser.parse_args(["--print-schema"])
    assert args.print_schema
    assert ir_schema.main(["--print-schema"]) == 0


def test_cli_as_a_script_from_the_repo_root():
    """`python ir_schema.py <manifest>` works without the package installed."""
    proc = subprocess.run(
        [sys.executable, str(ROOT / "ir_schema.py"), str(FULL)],
        cwd=str(ROOT), capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    assert "OK" in proc.stdout

    proc = subprocess.run(
        [sys.executable, str(ROOT / "ir_schema.py"), "--print-schema"],
        cwd=str(ROOT), capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    assert json.loads(proc.stdout)["title"] == ir_schema.load_schema()["title"]
