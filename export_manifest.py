#!/usr/bin/env python3
"""export_manifest.py -- Rigenera i file di configurazione dal CAD.

Catena:
  cad/*.FCStd  --freecadcmd-->  config/cad_manifest.json + meshes/visual
  (nessun CAD) --demo------->  config/cad_manifest.json + meshes/visual
  cad_manifest.json --sync-->  config/pipeline.json
                               config/hydrodynamics_params.json
                               config/mooring.json

L'export FreeCAD e' cached sull'hash SHA-256 del file .FCStd: se il manifest
e' gia' all'ultimo hash non si riesporta. Il fallback demo parte solo se
manca del tutto il manifest.

Uso
---
  python3 src/cad/export_manifest.py [--force] [--write-mooring-derived]
  make config
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
from typing import List, Optional

if __package__ in (None, ""):
    _src = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _src not in sys.path:
        sys.path.insert(0, _src)

from ws_root import workspace_root

CAD_CANDIDATES = ("wec_assembly.FCStd", "assembly.FCStd", "assembly_test.FCStd")
EXPORTER = os.path.join("src", "cad", "freecad_exporter.py")
DEMO = os.path.join("src", "cad", "create_demo_model.py")
MANIFEST_REL = os.path.join("config", "cad_manifest.json")

FREECADCMD_CANDIDATES = (
    "/Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd",
    "freecadcmd",
    "FreeCADCmd",
)


def _sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return "sha256:" + h.hexdigest()


def _log(msg: str) -> None:
    print("[export] {}".format(msg))


def find_cad_file(ws: str, explicit: Optional[str] = None) -> Optional[str]:
    if explicit and os.path.isfile(explicit):
        return os.path.abspath(explicit)
    env = os.environ.get("MARITIME_CAD")
    if env and os.path.isfile(env):
        return os.path.abspath(env)
    for name in CAD_CANDIDATES:
        path = os.path.join(ws, "cad", name)
        if os.path.isfile(path):
            return path
    return None


def find_freecadcmd() -> Optional[str]:
    env = os.environ.get("FREECADCMD")
    if env and os.path.isfile(env) and os.access(env, os.X_OK):
        return env
    for cand in FREECADCMD_CANDIDATES:
        path = shutil.which(cand) if os.sep not in cand else cand
        if path and os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    return None


def manifest_is_current(manifest_path: str, cad_hash: str) -> bool:
    if not os.path.isfile(manifest_path):
        return False
    try:
        with open(manifest_path) as fh:
            manifest = json.load(fh)
    except (OSError, ValueError):
        return False
    return manifest.get("meta", {}).get("source_hash") == cad_hash


def export_from_freecad(ws: str, freecadcmd: str, cad_file: str) -> bool:
    env = dict(os.environ, MARITIME_WS=ws, MARITIME_CAD=cad_file)
    res = subprocess.run([freecadcmd, os.path.join(ws, EXPORTER)], env=env)
    return res.returncode == 0


def export_demo(ws: str, force: bool) -> bool:
    cmd = [sys.executable, os.path.join(ws, DEMO)]
    if force:
        cmd.append("--force")
    return subprocess.run(cmd).returncode == 0


def sync_configs(ws: str, write_mooring_derived: bool) -> None:
    from cad import config_sync

    report = config_sync.sync_from_manifest(
        ws=ws, write=True, write_mooring_derived=write_mooring_derived)
    report.log()


def export_manifest(ws: Optional[str] = None,
                    cad_file: Optional[str] = None,
                    force: bool = False,
                    write_mooring_derived: bool = False) -> int:
    ws = os.path.abspath(ws or workspace_root())
    manifest_path = os.path.join(ws, MANIFEST_REL)
    cad = find_cad_file(ws, cad_file)

    if cad is not None:
        cad_hash = _sha256(cad)
        if not force and manifest_is_current(manifest_path, cad_hash):
            _log("manifest aggiornato ({}): skip export".format(
                cad_hash[:23] + "..."))
        else:
            freecadcmd = find_freecadcmd()
            if freecadcmd is None:
                _log("freecadcmd non trovato: uso manifest esistente (o demo)")
            else:
                _log("export da {} ({}...)".format(
                    os.path.relpath(cad, ws), cad_hash[7:23]))
                if not export_from_freecad(ws, freecadcmd, cad):
                    _log("export FreeCAD fallito")
                    return 1
    else:
        _log("nessun .FCStd in cad/: modello demo")

    if not os.path.isfile(manifest_path):
        _log("manifest assente: genero il modello demo")
        if not export_demo(ws, force=force):
            _log("creazione modello demo fallita")
            return 1

    sync_configs(ws, write_mooring_derived=write_mooring_derived)
    _log("configurazione allineata a {}".format(
        os.path.relpath(manifest_path, ws)))
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Rigenera i file di configurazione dal CAD (o demo)")
    parser.add_argument("--ws", default=None,
                        help="root del workspace (default: ws_root)")
    parser.add_argument("--cad", default=None,
                        help="file .FCStd esplicito (default: MARITIME_CAD o cad/)")
    parser.add_argument("--force", action="store_true",
                        help="riesporta anche se l'hash del CAD non e' cambiato")
    parser.add_argument("--write-mooring-derived", action="store_true",
                        help="riscrive mooring.break_tension_n dall'euristica")
    args = parser.parse_args(argv)
    return export_manifest(ws=args.ws, cad_file=args.cad, force=args.force,
                           write_mooring_derived=args.write_mooring_derived)


if __name__ == "__main__":
    sys.exit(main())
