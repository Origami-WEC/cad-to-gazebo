#!/usr/bin/env python3
"""cad_input.py -- Ingresso unico del CAD: FreeCAD (.FCStd) o STEP (.step/.stp).

Il chiamante non deve sapere quale formato sta usando. Questo modulo:

  1. riconosce il tipo di file dal contenuto (magic number), non solo dal nome;
  2. porta il file al manifest IR v1 (``config/cad_manifest.json``):
       FCStd -> export_manifest (freecadcmd, cache su hash SHA-256);
       STEP  -> step_to_ir (headless, OCCT), con contratto opzionale;
  3. valida il manifest e allinea la configurazione (config_sync).

Discovery del file se non indicato (in ordine):
  argomento --cad  >  env MARITIME_CAD  >  unico CAD in cad/  >  demo (nessun CAD).
Se in cad/ ci sono piu' CAD e nessuno e' indicato, l'errore elenca i candidati:
la scelta non viene fatta in silenzio.

Uso
---
  python -m cad.cad_input                      # auto (cad/ o demo)
  python -m cad.cad_input --cad cad/x.FCStd
  python -m cad.cad_input --cad part.step --contract contract.json
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from typing import List, Optional

if __package__ in (None, ""):
    _src = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _src not in sys.path:
        sys.path.insert(0, _src)

from cad import export_manifest as em
from ws_root import workspace_root

FREECAD = "freecad"
STEP = "step"
FREECAD_MAGIC = b"PK\x03\x04"           # .FCStd e' uno zip
STEP_MAGIC = b"ISO-10303-21"           # intestazione ISO 10303-21 (Part 21)
CONTRACT_ENV = "MARITIME_CONTRACT"


class CadInputError(RuntimeError):
    """Input CAD non riconoscibile, ambiguo o non convertibile."""


def sniff_kind(path: str) -> str:
    """Ritorna FREECAD o STEP in base al contenuto; errore se non e' nessuno dei due."""
    if not os.path.isfile(path):
        raise CadInputError("file CAD non esistente: {}".format(path))
    with open(path, "rb") as fh:
        head = fh.read(512)
    if head.startswith(FREECAD_MAGIC):
        return FREECAD
    if STEP_MAGIC in head[:256]:
        return STEP
    raise CadInputError(
        "formato non riconosciuto: {} (attesi FreeCAD .FCStd o STEP .step/.stp)"
        .format(path))


def discover(ws: str, explicit: Optional[str] = None) -> Optional[str]:
    """Trova il CAD da usare. None = nessun CAD (si usa il modello demo).

    Unica regola, valida per ogni punto d'ingresso: argomento esplicito >
    `MARITIME_CAD` > l'unico file CAD in `cad/`. Piu' file e nessuna scelta
    esplicita e' un errore, mai una preferenza implicita.
    """
    chosen = explicit or os.environ.get("MARITIME_CAD") or None
    if chosen:
        path = chosen if os.path.isabs(chosen) else os.path.join(ws, chosen)
        if not os.path.isfile(path):
            raise CadInputError("CAD indicato non esiste: {}".format(chosen))
        return os.path.abspath(path)

    cad_dir = os.path.join(ws, "cad")
    found: List[str] = []
    for pattern in ("*.FCStd", "*.fcstd", "*.step", "*.stp", "*.STEP", "*.STP"):
        found.extend(glob.glob(os.path.join(cad_dir, pattern)))
    found = sorted(set(found))
    if not found:
        return None
    if len(found) == 1:
        return found[0]
    names = ", ".join(os.path.relpath(p, ws) for p in found)
    raise CadInputError(
        "piu' CAD in cad/ ({}): indicarne uno con CAD=<file>".format(names))


def _sha256(path: str) -> str:
    return em._sha256(path)


def _step_manifest_path(ws: str) -> str:
    return os.path.join(ws, em.MANIFEST_REL)


def _choose_root(document: dict) -> Optional[str]:
    """Body radice per un STEP senza contratto: quello di massa maggiore.

    E' una scelta euristica, dichiarata nel log: il contratto e' il modo
    corretto di fissare la radice del dispositivo.
    """
    best, best_mass = None, -1.0
    for body in document.get("bodies", []):
        mass = sum(float(s.get("mass_kg") or 0.0) for s in body.get("solids", []))
        if mass > best_mass:
            best, best_mass = body.get("name"), mass
    return best


def _convert_step(ws: str, cad: str, contract: Optional[str], force: bool) -> int:
    from cad import ir_schema, step_to_ir

    digest = _sha256(cad)
    manifest_path = _step_manifest_path(ws)
    if not force and em.manifest_is_current(manifest_path, digest):
        em._log("manifest aggiornato ({}): skip conversione STEP".format(digest[:23] + "..."))
        return 0

    rel = os.path.relpath(cad, ws)
    em._log("conversione STEP {} (headless, puo' richiedere minuti)".format(rel))
    try:
        document = step_to_ir.convert_step(cad, mesh_root=ws, contract=contract)
    except (step_to_ir.StepToIrError, ir_schema.IRSchemaError) as exc:
        em._log("ERRORE conversione STEP: {}".format(exc))
        return 1

    meta = document.setdefault("meta", {})
    if not meta.get("root"):
        root = _choose_root(document)
        if root is not None:
            em._log("ATTENZIONE: nessuna radice nel contratto: uso il body di massa "
                    "maggiore '{}' (fissare meta.root nel contratto)".format(root))
            meta["root"] = root
            ir_schema.validate(document, source=rel)
    meta["source_file"] = rel
    meta["source_hash"] = digest

    os.makedirs(os.path.dirname(manifest_path), exist_ok=True)
    with open(manifest_path, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(document, fh, indent=2, ensure_ascii=False)
        fh.write("\n")
    em._log("IR scritto: {} ({} body, {} joint)".format(
        os.path.relpath(manifest_path, ws), len(document.get("bodies", [])),
        len(document.get("joints", []))))
    return 0


def ingest(cad: Optional[str], ws: Optional[str] = None,
           contract: Optional[str] = None, force: bool = False,
           write_mooring_derived: bool = False) -> int:
    """Porta il CAD (o il demo) al manifest v1 e allinea la configurazione."""
    ws = os.path.abspath(ws or workspace_root())
    try:
        cad = discover(ws, cad)
    except CadInputError as exc:
        em._log("ERRORE: {}".format(exc))
        return 1

    if cad is None:
        em._log("nessun CAD in cad/: modello demo")
        return em.export_manifest(ws=ws, cad_file=None, force=force,
                                  write_mooring_derived=write_mooring_derived)

    try:
        kind = sniff_kind(cad)
    except CadInputError as exc:
        em._log("ERRORE: {}".format(exc))
        return 1
    em._log("CAD: {} -> {}".format(os.path.relpath(cad, ws), kind))

    if kind == FREECAD:
        return em.export_manifest(ws=ws, cad_file=cad, force=force,
                                  write_mooring_derived=write_mooring_derived)

    contract = contract or os.environ.get(CONTRACT_ENV) or None
    if contract is not None and not os.path.isfile(contract):
        em._log("ERRORE: contratto non trovato: {}".format(contract))
        return 1
    rc = _convert_step(ws, cad, contract, force)
    if rc != 0:
        return rc
    if not em.validate_manifest(_step_manifest_path(ws)):
        em._log("config non allineati: il manifest non supera il validatore IR")
        return 1
    if not em.sync_configs(ws, write_mooring_derived=write_mooring_derived):
        return 1
    em._log("configurazione allineata a {}".format(em.MANIFEST_REL))
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Ingresso unico CAD: FreeCAD .FCStd o STEP .step/.stp")
    parser.add_argument("--ws", default=None, help="root del workspace")
    parser.add_argument("--cad", default=None,
                        help="file CAD (default: MARITIME_CAD, poi unico file in cad/)")
    parser.add_argument("--contract", default=None,
                        help="contratto JSON per STEP (default: MARITIME_CONTRACT)")
    parser.add_argument("--force", action="store_true",
                        help="ignora la cache su hash e riconverte")
    parser.add_argument("--write-mooring-derived", action="store_true",
                        help="riscrive mooring.break_tension_n dall'euristica")
    args = parser.parse_args(argv)
    return ingest(args.cad, ws=args.ws, contract=args.contract, force=args.force,
                  write_mooring_derived=args.write_mooring_derived)


if __name__ == "__main__":
    sys.exit(main())
