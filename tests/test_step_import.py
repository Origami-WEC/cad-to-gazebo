"""STEP front-end, kernel-free half of the suite.

Everything here runs with **or without** the OCCT kernel installed: these are
the contract checks on header detection, error handling, laziness and the CLI,
all of which must fail cleanly *before* the kernel is ever touched. The tests
that actually need the kernel live in ``test_step_import_ocp.py``.
"""

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest

import step_import

ROOT = Path(__file__).resolve().parent.parent
OCP_INSTALLED = importlib.util.find_spec("OCP") is not None

AP242_HEADER = (
    "ISO-10303-21;\n"
    "HEADER;\n"
    "FILE_DESCRIPTION(('fixture'),'2;1');\n"
    "FILE_NAME('fixture.step','2026-01-01T00:00:00',(''),(''),'','','');\n"
    "FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF "
    "{ 1 0 10303 442 1 1 4 }'));\n"
    "ENDSEC;\n"
    "DATA;\n"
    "ENDSEC;\n"
    "END-ISO-10303-21;\n"
)


def _write(tmp_path: Path, name: str, text: str) -> Path:
    target = tmp_path / name
    target.write_text(text, encoding="utf-8")
    return target


# --------------------------------------------------------------------------- #
# laziness / no GUI
# --------------------------------------------------------------------------- #


def test_module_never_imports_a_gui_or_freecad():
    source = (ROOT / "step_import.py").read_text(encoding="utf-8")
    for module in step_import._GUI_MODULES:
        assert "import %s" % module not in source, module
        assert "from %s" % module not in source, module


def test_module_imports_in_a_child_interpreter_without_ocp():
    # The kernel is imported lazily: a host that cannot install cadquery-ocp
    # must still be able to import, lint and unit-test the package.
    code = (
        "import sys, types, builtins\n"
        "ROOT = r'%s'\n"
        "real = builtins.__import__\n"
        "def guard(name, *a, **k):\n"
        "    if name.split('.')[0] in ('OCP', 'FreeCAD', 'PySide6', 'PyQt6'):\n"
        "        raise ImportError('blocked ' + name)\n"
        "    return real(name, *a, **k)\n"
        "builtins.__import__ = guard\n"
        "sys.path.insert(0, ROOT)\n"
        "pkg = types.ModuleType('cad'); pkg.__path__ = [ROOT]\n"
        "sys.modules['cad'] = pkg\n"
        "from cad import step_import\n"
        "print(step_import.IR_SCHEMA)\n"
    ) % str(ROOT)
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert step_import.IR_SCHEMA in result.stdout


@pytest.mark.skipif(OCP_INSTALLED, reason="OCP is installed on this host")
def test_missing_kernel_is_reported_with_the_install_hint(tmp_path):
    step = _write(tmp_path, "head_only.step", AP242_HEADER)
    with pytest.raises(step_import.StepImportError) as excinfo:
        step_import.import_step(step)
    assert "cadquery-ocp" in str(excinfo.value)


# --------------------------------------------------------------------------- #
# header / protocol detection
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    ("file_schema", "expected"),
    [
        ("AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 1 1 4 }", "AP242"),
        ("AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }", "AP214"),
        ("CONFIG_CONTROL_DESIGN { 1 0 10303 203 1 1 }", "AP203"),
        ("SOMETHING_ELSE { 1 0 1 }", "AP_UNKNOWN"),
        ("", "AP_UNKNOWN"),
    ],
)
def test_classify_schema(file_schema, expected):
    assert step_import.classify_schema(file_schema) == expected


def test_detect_protocol_reads_ap242_header(tmp_path):
    step = _write(tmp_path, "a.step", AP242_HEADER)
    protocol = step_import.detect_protocol(step)
    assert protocol["application_protocol"] == "AP242"
    assert protocol["file_schema"].startswith("AP242_MANAGED_MODEL_BASED")
    assert protocol["file_description"] == "fixture"


def test_detect_protocol_reads_a_multiline_file_schema(tmp_path):
    # Real exporters wrap FILE_SCHEMA across lines; the detection must not
    # depend on the header being on a single line.
    text = AP242_HEADER.replace(
        "FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF "
        "{ 1 0 10303 442 1 1 4 }'));",
        "FILE_SCHEMA((\n'AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF {1 0 10303 442 1 1 4 }'\n    ));",
    )
    step = _write(tmp_path, "wrapped.step", text)
    assert step_import.detect_protocol(step)["application_protocol"] == "AP242"


def test_detect_protocol_reads_ap214_header(tmp_path):
    step = _write(
        tmp_path, "ap214.step",
        AP242_HEADER.replace(
            "AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 1 1 4 }",
            "AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }",
        ),
    )
    assert step_import.detect_protocol(step)["application_protocol"] == "AP214"


# --------------------------------------------------------------------------- #
# error handling
# --------------------------------------------------------------------------- #


def test_missing_file_is_rejected(tmp_path):
    with pytest.raises(step_import.StepImportError) as excinfo:
        step_import.import_step(tmp_path / "absent.step")
    assert "not found" in str(excinfo.value)


def test_non_step_file_is_rejected_before_the_kernel_is_touched(tmp_path):
    step = _write(tmp_path, "garbage.step", "this is a text file, not a STEP file\n")
    with pytest.raises(step_import.StepImportError) as excinfo:
        step_import.import_step(step)
    assert "not a STEP exchange file" in str(excinfo.value)


def test_header_without_file_schema_is_rejected(tmp_path):
    step = _write(tmp_path, "noschema.step",
                  "ISO-10303-21;\nHEADER;\nFILE_NAME('x','y',(''),(''),'','','');\nENDSEC;\n")
    with pytest.raises(step_import.StepImportError) as excinfo:
        step_import.import_step(step)
    assert "FILE_SCHEMA" in str(excinfo.value)


def test_sha256_file_matches_a_reference_digest(tmp_path):
    target = tmp_path / "blob.bin"
    target.write_bytes(b"")
    assert step_import.sha256_file(target) == (
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
    )


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #


def test_cli_requires_the_step_argument():
    result = subprocess.run(
        [sys.executable, "-m", "step_import"], capture_output=True, text=True, cwd=str(ROOT),
    )
    assert result.returncode == 2
    assert "--step" in result.stderr


def test_cli_reports_a_bad_file_with_status_1(tmp_path):
    step = _write(tmp_path, "garbage.step", "not step\n")
    result = subprocess.run(
        [sys.executable, "-m", "step_import", "--step", str(step), "--out", str(tmp_path / "ir.json")],
        capture_output=True, text=True, cwd=str(ROOT),
    )
    assert result.returncode == 1
    assert "not a STEP exchange file" in result.stderr
    assert not (tmp_path / "ir.json").exists()


def test_cli_help_documents_the_documented_invocation():
    result = subprocess.run(
        [sys.executable, "-m", "step_import", "--help"], capture_output=True, text=True, cwd=str(ROOT),
    )
    assert result.returncode == 0
    assert "python -m cad.step_import" in result.stdout


def test_ir_schema_constant_is_json_stable():
    # The IR schema tag is part of the contract with the downstream stages.
    assert json.dumps({"schema": step_import.IR_SCHEMA}) == '{"schema": "cad-to-gazebo/step-ir/1"}'
