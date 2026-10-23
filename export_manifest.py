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
import tempfile
from typing import List, Optional

if __package__ in (None, ""):
    _src = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _src not in sys.path:
        sys.path.insert(0, _src)

from ws_root import workspace_root

EXPORTER = os.path.join("src", "cad", "freecad_exporter.py")
DEMO = os.path.join("src", "cad", "create_demo_model.py")
MANIFEST_REL = os.path.join("config", "cad_manifest.json")

_FREECAD_APP = os.environ.get("FREECAD_APP", "/Applications/FreeCAD.app")

FREECADCMD_CANDIDATES = (
    os.path.join(_FREECAD_APP, "Contents", "Resources", "bin", "freecadcmd"),
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


def _mtime_ns(path: str) -> Optional[int]:
    try:
        return os.stat(path).st_mtime_ns
    except OSError:
        return None


FREECAD_SITE_DIRNAME = ".freecad_site"


def freecad_extra_site(ws: str) -> Optional[str]:
    """Cartella con pacchetti extra per il Python di FreeCAD (es. jsonschema).

    FREECAD_EXTRA_SITE ha la precedenza; altrimenti <ws>/.freecad_site se esiste.
    Creata da ``make freecad-deps``. Non tocca l'installazione di FreeCAD.
    """
    env = os.environ.get("FREECAD_EXTRA_SITE")
    if env:
        return os.path.abspath(env)
    local = os.path.join(ws, FREECAD_SITE_DIRNAME)
    return local if os.path.isdir(local) else None


def _freecad_bootstrap(script: str, extra_site: Optional[str]) -> str:
    """File temporaneo eseguito da freecadcmd: aggiunge extra_site a sys.path.

    freecadcmd ignora PYTHONPATH, quindi il path va inserito dentro il processo.
    """
    lines = ["import sys, runpy"]
    if extra_site:
        lines.append("sys.path.insert(0, {!r})".format(extra_site))
    lines.append("sys.argv = [{!r}]".format(script))
    lines.append("runpy.run_path({!r}, run_name='__main__')".format(script))
    fd, path = tempfile.mkstemp(prefix="fc_boot_", suffix=".py")
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    return path


def export_from_freecad(ws: str, freecadcmd: str, cad_file: str) -> bool:
    env = dict(os.environ, MARITIME_WS=ws, MARITIME_CAD=cad_file)
    boot = _freecad_bootstrap(os.path.join(ws, EXPORTER), freecad_extra_site(ws))
    try:
        res = subprocess.run([freecadcmd, boot], env=env)
    finally:
        try:
            os.remove(boot)
        except OSError:
            pass
    return res.returncode == 0


def export_demo(ws: str, force: bool) -> bool:
    cmd = [sys.executable, os.path.join(ws, DEMO)]
    if force:
        cmd.append("--force")
    return subprocess.run(cmd).returncode == 0


def validate_manifest(manifest_path: str) -> bool:
    """Gate the manifest through the IR v1 validator before the config sync.

    The producers (`freecad_exporter.py`, `create_demo_model.py`) already refuse
    to write a manifest that breaks the contract. This second gate catches the
    case where the manifest comes from somewhere else -- a checkout produced by
    an older exporter, a hand-edited file. A pre-v1 (legacy) manifest is
    accepted with an explicit warning instead of blocking the whole run; a
    manifest that *claims* to be v1 and is not is a hard failure.
    """
    from cad import ir_schema

    try:
        ir_schema.validate_file(manifest_path, allow_legacy=True)
    except ir_schema.IRSchemaError as exc:
        _log("manifest IR NON valido:")
        for msg in exc.errors:
            _log("  - " + msg)
        return False

    try:
        with open(manifest_path) as fh:
            manifest = json.load(fh)
    except (OSError, ValueError):  # pragma: no cover - validate_file caught it
        return False
    if ir_schema.looks_legacy(manifest):
        _log("ATTENZIONE: manifest pre-v1 (nessun tag '{}'): accettato in "
             "lettura, ma le nuove scritture devono usare lo schema v1 "
             "(vedi docs/ir-v1.md).".format(ir_schema.IR_SCHEMA))
    return True


def sync_configs(ws: str, write_mooring_derived: bool) -> bool:
    """Allinea pipeline/hydro/mooring al manifest. False = errore, gia' loggato.

    Un'eccezione qui (es. modulo mancante) va riportata come errore leggibile,
    non come traceback: il chiamante decide l'exit code.
    """
    try:
        from cad import config_sync

        report = config_sync.sync_from_manifest(
            ws=ws, write=True, write_mooring_derived=write_mooring_derived)
        report.log()
        return True
    except Exception as exc:  # noqa: BLE001 - riportato all'utente, non nascosto
        _log("ERRORE config_sync: {}: {}".format(type(exc).__name__, exc))
        return False


def export_manifest(ws: Optional[str] = None,
                    cad_file: Optional[str] = None,
                    force: bool = False,
                    write_mooring_derived: bool = False) -> int:
    """Esporta il manifest dal FCStd (o demo se non c'e' alcun CAD).

    Regole (niente fallback silenziosi):
      * scelta del CAD: `cad_input.discover` (argomento > MARITIME_CAD > unico
        file in cad/); CAD esplicito inesistente o ambiguo -> errore;
      * CAD presente ma FreeCAD assente -> errore, a meno che il manifest sia
        gia' allineato all'hash del CAD (in tal caso nulla da fare);
      * export riuscito solo se il manifest riporta l'hash del CAD: il exit
        code di freecadcmd non basta (Python eccezioni -> exit 0).
    """
    ws = os.path.abspath(ws or workspace_root())
    manifest_path = os.path.join(ws, MANIFEST_REL)
    # Un'unica regola di scelta del CAD, condivisa con l'ingresso unico.
    from cad import cad_input
    try:
        cad = cad_input.discover(ws, cad_file)
        if cad is not None and cad_input.sniff_kind(cad) != cad_input.FREECAD:
            raise cad_input.CadInputError(
                "{} non e' un .FCStd: per STEP usa `python -m cad.cad_input`"
                .format(os.path.relpath(cad, ws)))
    except cad_input.CadInputError as exc:
        _log("ERRORE: {}".format(exc))
        return 1

    if cad is not None:
        cad_hash = _sha256(cad)
        if not force and manifest_is_current(manifest_path, cad_hash):
            _log("manifest aggiornato ({}): skip export".format(
                cad_hash[:23] + "..."))
        else:
            freecadcmd = find_freecadcmd()
            if freecadcmd is None:
                _log("ERRORE: {} richiede FreeCAD (freecadcmd non trovato; "
                     "imposta FREECADCMD)".format(os.path.relpath(cad, ws)))
                return 1
            _log("export da {} ({}...)".format(
                os.path.relpath(cad, ws), cad_hash[7:23]))
            mtime_before = _mtime_ns(manifest_path)
            export_from_freecad(ws, freecadcmd, cad)
            # Il manifest precedente puo' gia' avere questo hash: serve anche che
            # il run lo abbia riscritto (exit code e hash da soli non bastano).
            rewritten = _mtime_ns(manifest_path) not in (None, mtime_before)
            if not (rewritten and manifest_is_current(manifest_path, cad_hash)):
                _log("ERRORE: export FreeCAD non ha prodotto un manifest per "
                     "questo CAD (vedi log sopra)")
                _log("  se il log cita 'jsonschema': make freecad-deps")
                return 1
    else:
        _log("nessun .FCStd in cad/: modello demo")

    if not os.path.isfile(manifest_path):
        _log("manifest assente: genero il modello demo")
        if not export_demo(ws, force=force):
            _log("creazione modello demo fallita")
            return 1

    if not validate_manifest(manifest_path):
        _log("config non allineati: il manifest non supera il validatore IR")
        return 1

    if not sync_configs(ws, write_mooring_derived=write_mooring_derived):
        return 1
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
