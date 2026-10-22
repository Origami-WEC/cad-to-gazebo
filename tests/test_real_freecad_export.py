"""Export reale da documenti FreeCAD (``.FCStd``) via ``freecadcmd``.

Il test lancia davvero ``freecad_exporter.py`` sotto FreeCAD su file del
progetto e verifica il manifest prodotto. Non si fida dell'exit code:
``freecadcmd`` esce con 0 anche quando lo script fallisce, quindi il criterio
e' che ``config/cad_manifest.json`` esista e superi il validatore IR.

Requisiti (altrimenti il test si salta con il motivo esplicito):
  * ``freecadcmd`` installato (default: AppImage/bundle macOS; override con
    ``FREECADCMD``);
  * i file ``.FCStd`` presenti;
  * ``FREECAD_EXTRA_SITE``: cartella con ``jsonschema`` per il Python incorporato
    di FreeCAD, che non lo include. Senza, l'export si ferma.

Il workspace e' una copia temporanea: ``maritime_ws`` non viene scritto.

Il file ``Dispositivo/test/assembly.FCStd`` ha solo giunti ``Fixed``: per lui si
verifica la pipeline, non il requisito del giunto non-fixed.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

import ir_schema

ROOT = Path(__file__).resolve().parent.parent
EXPORTER = ROOT / "freecad_exporter.py"
#: Il config/pipeline.json vive nel workspace maritime_ws, sopra src/cad.
WORKSPACE_PIPELINE = ROOT.parent.parent / "config" / "pipeline.json"

FREECADCMD = os.environ.get(
    "FREECADCMD", "/Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd")
EXTRA_SITE = os.environ.get("FREECAD_EXTRA_SITE", "")

CAD_DIR = ROOT.parent.parent / "cad"
CASE_ASSEMBLY_TEST = CAD_DIR / "assembly_test.FCStd"
CASE_DISPOSITIVO = Path(
    "/Users/riccardo/Documents/Cartella/Progetti/Origami/Dispositivo/test/assembly.FCStd")

#: Wrapper che aggiunge ``FREECAD_EXTRA_SITE`` a ``sys.path`` prima dell'export:
#: ``freecadcmd`` non rispetta ``PYTHONPATH``.
WRAPPER = """\
import os, runpy, sys
extra = os.environ.get("FREECAD_EXTRA_SITE", "")
if extra:
    sys.path.insert(0, extra)
runpy.run_path({exporter!r}, run_name="__main__")
"""


def _skip_reason(cad_file: Path) -> str | None:
    if not os.path.exists(FREECADCMD):
        return "freecadcmd assente: %s" % FREECADCMD
    if not cad_file.is_file():
        return "file FCStd assente: %s" % cad_file
    if not WORKSPACE_PIPELINE.is_file():
        return "config/pipeline.json del workspace assente: %s" % WORKSPACE_PIPELINE
    if not EXTRA_SITE:
        return "FREECAD_EXTRA_SITE non impostato (jsonschema per il Python di FreeCAD)"
    return None


def _run_export(cad_file: Path, tmp_path: Path) -> Path:
    """Lancia l'exporter su ``cad_file`` e ritorna la cartella workspace."""
    ws = tmp_path / "ws"
    (ws / "worlds").mkdir(parents=True)
    (ws / "config").mkdir()
    shutil.copy(WORKSPACE_PIPELINE, ws / "config" / "pipeline.json")
    wrapper = tmp_path / "run_exporter.py"
    wrapper.write_text(WRAPPER.format(exporter=str(EXPORTER)), encoding="utf-8")
    env = dict(os.environ, MARITIME_WS=str(ws), MARITIME_CAD=str(cad_file),
               FREECAD_EXTRA_SITE=EXTRA_SITE)
    subprocess.run([FREECADCMD, str(wrapper)], env=env, capture_output=True,
                   text=True, timeout=900)
    return ws


@pytest.fixture(scope="module")
def assembly_test_manifest(tmp_path_factory):
    reason = _skip_reason(CASE_ASSEMBLY_TEST)
    if reason:
        pytest.skip(reason)
    ws = _run_export(CASE_ASSEMBLY_TEST, tmp_path_factory.mktemp("fc_assembly_test"))
    manifest = ws / "config" / "cad_manifest.json"
    assert manifest.is_file(), "export non ha prodotto cad_manifest.json"
    return json.loads(manifest.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def dispositivo_manifest(tmp_path_factory):
    reason = _skip_reason(CASE_DISPOSITIVO)
    if reason:
        pytest.skip(reason)
    ws = _run_export(CASE_DISPOSITIVO, tmp_path_factory.mktemp("fc_dispositivo"))
    manifest = ws / "config" / "cad_manifest.json"
    assert manifest.is_file(), "export non ha prodotto cad_manifest.json"
    return json.loads(manifest.read_text(encoding="utf-8"))


def test_assembly_test_manifest_is_valid(assembly_test_manifest):
    ir_schema.validate(assembly_test_manifest)


def test_assembly_test_has_a_non_fixed_joint(assembly_test_manifest):
    bodies = {b["name"] for b in assembly_test_manifest["bodies"]}
    non_fixed = [j for j in assembly_test_manifest["joints"] if j["fc_type"] != "Fixed"]
    assert non_fixed, "nessun giunto non-fixed nel manifest"
    for j in non_fixed:
        assert j["parent"] in bodies and j["child"] in bodies


def test_dispositivo_manifest_is_valid(dispositivo_manifest):
    ir_schema.validate(dispositivo_manifest)
