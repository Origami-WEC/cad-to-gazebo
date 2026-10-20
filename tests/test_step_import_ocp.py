"""STEP front-end, kernel half of the suite (needs ``cadquery-ocp``).

The whole module skips with an explicit, motivating reason when OCCT's
bindings are absent -- CI installs them only in the dedicated ``step-headless``
job, and a local ``pip install 'cad-to-gazebo[dev]'`` does not pull them in.
A skip here means "not verified on this host", never "verified green".
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip(
    "OCP",
    reason="OCCT bindings absent: install with `python -m pip install 'cad-to-gazebo[step]'`",
)

from step_fixtures import find_mechanism_step, write_ap242_assembly  # noqa: E402

import step_import  # noqa: E402  (import order is forced by the importorskip above)

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def ap242_step(tmp_path_factory) -> Path:
    return write_ap242_assembly(tmp_path_factory.mktemp("fixture") / "assembly_ap242.step")


@pytest.fixture(scope="module")
def ap242_ir(ap242_step) -> dict:
    return step_import.import_step(ap242_step)


# --------------------------------------------------------------------------- #
# generated AP242 fixture
# --------------------------------------------------------------------------- #


def test_fixture_really_declares_ap242(ap242_step):
    assert "AP242" in ap242_step.read_text(encoding="utf-8", errors="replace")
    assert step_import.detect_protocol(ap242_step)["application_protocol"] == "AP242"


def test_ap242_import_produces_the_ir(ap242_ir):
    assert ap242_ir["schema"] == "cad-to-gazebo/step-ir/1"
    assert ap242_ir["generator"]["headless"] is True
    assert ap242_ir["source"]["application_protocol"] == "AP242"
    assert ap242_ir["source"]["format"] == "STEP"
    assert ap242_ir["units"]["length"] == "MM"
    assert len(ap242_ir["source"]["sha256"]) == 64
    assert ap242_ir["source"]["bytes"] > 0


def test_ap242_import_counts_assemblies_and_solids(ap242_ir):
    counts = ap242_ir["counts"]
    assert counts["assemblies"] == 1
    assert counts["parts"] == 2
    assert counts["solids"] == 2
    assert counts["faces"] == 12  # two boxes


def test_ap242_part_names_come_from_the_products(ap242_ir):
    names = sorted(part["name"] for part in ap242_ir["parts"])
    assert names == ["frame", "shaft"]
    assert ap242_ir["assemblies"][0]["name"] == "fixture_assembly"
    for part in ap242_ir["parts"]:
        assert part["path"] == "%s/%s" % (part["parent"], part["name"])


def test_ap242_geometry_is_in_millimetres_and_placed(ap242_ir):
    by_name = {part["name"]: part for part in ap242_ir["parts"]}

    frame = by_name["frame"]
    assert frame["bbox"]["min"] == pytest.approx([0.0, 0.0, 0.0], abs=1e-6)
    assert frame["bbox"]["max"] == pytest.approx([20.0, 10.0, 5.0], abs=1e-6)

    shaft = by_name["shaft"]
    assert shaft["bbox"]["min"][0] == pytest.approx(50.0, abs=1e-6)
    assert shaft["volume"] == pytest.approx(4.0 * 4.0 * 60.0, rel=1e-6)
    assert frame["volume"] == pytest.approx(20.0 * 10.0 * 5.0, rel=1e-6)

    # Row-major 4x4, translation in the last column of the first three rows.
    assert shaft["transform"][3] == pytest.approx(50.0, abs=1e-6)
    assert shaft["transform"][12:] == [0.0, 0.0, 0.0, 1.0]


def test_names_are_unique_and_stable_across_runs(ap242_step, ap242_ir):
    again = step_import.import_step(ap242_step)
    assert [part["path"] for part in again["parts"]] == [
        part["path"] for part in ap242_ir["parts"]
    ]
    assert [asm["path"] for asm in again["assemblies"]] == [
        asm["path"] for asm in ap242_ir["assemblies"]
    ]
    paths = [asm["path"] for asm in ap242_ir["assemblies"]] + [
        part["path"] for part in ap242_ir["parts"]
    ]
    assert len(paths) == len(set(paths))


def test_ir_is_json_serialisable(ap242_ir):
    # No kernel object may leak into the IR: it has to survive json.dumps.
    assert json.loads(json.dumps(ap242_ir))["counts"] == ap242_ir["counts"]


def test_truncated_step_raises_a_clear_error(ap242_step, tmp_path):
    text = ap242_step.read_text(encoding="utf-8", errors="replace")
    broken = tmp_path / "truncated.step"
    broken.write_text(text[: int(len(text) * 0.4)], encoding="utf-8")
    with pytest.raises(step_import.StepImportError) as excinfo:
        step_import.import_step(broken)
    message = str(excinfo.value)
    assert "truncated" in message or "no shape" in message or "no product" in message


# --------------------------------------------------------------------------- #
# the documented CLI, exercised the way the README documents it
# --------------------------------------------------------------------------- #


def test_documented_cli_regenerates_the_ir(ap242_step, tmp_path):
    # The README documents `python -m cad.step_import ...`, which requires the
    # repository to be reachable as a package named `cad` (that is exactly how
    # maritime_ws mounts it, as src/cad).
    sandbox = tmp_path / "sandbox"
    sandbox.mkdir()
    (sandbox / "cad").symlink_to(ROOT, target_is_directory=True)
    out = tmp_path / "ir.json"

    env = dict(os.environ)
    env["PYTHONPATH"] = str(sandbox)
    result = subprocess.run(
        [sys.executable, "-m", "cad.step_import", "--step", str(ap242_step), "--out", str(out)],
        capture_output=True, text=True, cwd=str(sandbox), env=env,
    )
    assert result.returncode == 0, result.stderr
    assert out.is_file()
    ir = json.loads(out.read_text(encoding="utf-8"))
    assert ir["schema"] == step_import.IR_SCHEMA
    assert ir["counts"]["parts"] == 2
    assert "2 parts" in result.stdout


# --------------------------------------------------------------------------- #
# the real FreeCAD-authored assembly (end-to-end fixture)
# --------------------------------------------------------------------------- #

REAL_STEP = find_mechanism_step()


@pytest.mark.skipif(
    REAL_STEP is None,
    reason=(
        "the real fixture Transmission/Assemblies/full_transmission.step is not on this host "
        "(36 MB, it lives in Origami-WEC/Mechanism); set CAD_TO_GAZEBO_STEP_FIXTURE to point "
        "at a checkout to run the end-to-end check"
    ),
)
def test_real_transmission_assembly_imports():
    ir = step_import.import_step(REAL_STEP)
    counts = ir["counts"]
    assert counts["assemblies"] > 0
    assert counts["parts"] > 0
    assert counts["solids"] > 0

    paths = [asm["path"] for asm in ir["assemblies"]] + [part["path"] for part in ir["parts"]]
    assert len(paths) == len(set(paths)), "names must be unique"
    assert all(name.strip() and name != "unnamed" for name in paths), "names must be non-empty"
    assert all(part["bbox"] is not None for part in ir["parts"])

    # The file is FreeCAD's AP214 export; the IR must say so, not claim AP242.
    assert ir["source"]["application_protocol"] in ("AP214", "AP242")
    assert ir["source"]["bytes"] > 1_000_000

    root = ir["assemblies"][0]
    assert root["parent"] is None
    assert root["depth"] == 0
    assert root["children"], "the root assembly must list its components"
