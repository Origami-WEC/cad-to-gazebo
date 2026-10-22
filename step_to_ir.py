#!/usr/bin/env python3
"""step_to_ir.py  --  Ponte ``step-ir/1`` -> ``cad-ir/1`` (headless, senza FreeCAD).

Perche' esiste
--------------
``cad.step_import`` legge un file STEP e descrive la *geometria*: alberatura di
assemblaggi, placement, volume, bbox.  Non descrive la *fisica* del modello WEC:
topologia dei giunti, materiali, densita'.  Il contratto IR v1
(``cad-to-gazebo/cad-ir/1``) invece pretende entrambe, ed e' quello che mangiano
``cad.sdf_artifacts``, ``cad.config_sync`` e il build di ``maritime_ws``.

Questo modulo e' il ponte fra i due, con una divisione di responsabilita'
esplicita:

  * la **geometria** viene dal CAD (STEP): nomi, placement, volume, CoM,
    tensore d'inerzia calcolati dal kernel sul B-rep;
  * la **topologia dei giunti** e i **materiali/densita'** NON vengono
    dedotti dal CAD: sono *dichiarati* in un contratto JSON (``--contract``),
    perche' un giunto non e' un fatto geometrico e una densita' non e' scritta
    in nessun STEP.  Senza contratto il ponte emette ``joints: []`` e lo dice:
    non inventa topologia.

Lo stesso STEP viene letto una sola volta: ``step_import.import_step()`` riempie
un dizionario di forme OCCT (``shapes_out``) che qui vengono tessellate nei
mesh visuali del workspace (``meshes/visual/<body>__<solid>.stl``), cosi' la
catena STEP -> IR -> artefatti -> SDF gira anche su un host senza FreeCAD.

Contratto (``--contract``)
--------------------------
::

    {
      "device_type": "transmission",        // meta.device_type
      "root": "spare_transmission",         // body radice cinematico
      "tolerance_mm": 0.5,                  // tolleranza di tessellatura
      "density_kg_m3": 7850.0,              // default per le parti non elencate
      "material_card": "steel",             // carta materiale di default
      "skip": ["bolt_1", "seal_2"],         // parti non strutturali
      "parts": {
        "body001": {"density_kg_m3": 2700.0, "material_card": "aluminium"}
      },
      "joints": [
        {
          "name": "pto_slider", "fc_type": "Slider",
          "parent": "hull", "child": "pto_shaft",
          "origin": [0, 0, 0, 0, 0, 0, 1],
          "axis": [0, 0, 1],
          "limits": {"lower": -1.5, "upper": 1.5, "lower_enabled": true,
                     "upper_enabled": true, "unit": "m", "offset_m": 0.0},
          "activated": true
        }
      ]
    }

Le chiavi di ``parts`` e di ``skip`` sono i nomi CAD **oppure** i nomi IR
(sanitizzati): entrambi gli indici esistono, cosi' il contratto resta leggibile
anche quando il CAD usa spazi o maiuscole.

Uso
---
::

    python -m cad.step_to_ir --step Transmission/Assemblies/full_transmission.step \\
                             --ws . --contract contracts/transmission.json \\
                             --out config/cad_manifest.json

``--out`` assente = stdout.  Exit status 0 su successo, 1 su
:class:`StepToIrError` (motivo su stderr).  ``--no-meshes`` salta la
tessellatura e lascia i solidi senza ``stl_path``: l'IR resta valido, ma gli
artefatti SDF non avranno mesh da leggere.
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence

try:  # in-repo import: la radice del repository E' il package ``cad``
    from cad import ir_schema, material_props, naming, step_import
except ImportError:  # esecuzione standalone dalla radice del repository
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from cad import ir_schema, material_props, naming, step_import

__all__ = [
    "DEFAULT_DENSITY_KG_M3",
    "DEFAULT_DEVICE_TYPE",
    "DEFAULT_MATERIAL_CARD",
    "DEFAULT_TOLERANCE_MM",
    "StepToIrError",
    "convert",
    "convert_step",
    "load_contract",
    "plan",
    "main",
]

#: Densita' di default quando ne' il contratto ne' la CLI ne dichiarano una.
#: 7850 kg/m3 = acciaio: e' una scelta *dichiarata*, mai silenziosa -- finisce
#: in ``density_source`` di ogni solido che la usa.
DEFAULT_DENSITY_KG_M3 = 7850.0
DEFAULT_MATERIAL_CARD = "steel"
DEFAULT_DEVICE_TYPE = "generic"
DEFAULT_TOLERANCE_MM = 0.5

#: Unita' del grado di liberta' per tipo di ``Assembly::Joint`` di FreeCAD: e'
#: l'unica cosa del giunto che si *deduce* invece di dichiararla.  Un tipo non
#: elencato non e' un errore: pretende solo ``limits.unit`` scritto a mano.
_JOINT_UNITS = {
    "fixed": "m",
    "slider": "m",
    "cylindrical": "m",
    "distance": "m",
    "planar": "m",
    "screw": "m",
    "revolute": "rad",
    "ball": "rad",
    "continuous": "rad",
}

#: Chiavi accettate nel contratto.  Chiuse come il meta dell'IR: un refuso in un
#: contratto deve fallire subito, non essere ignorato.
CONTRACT_KEYS = frozenset({
    "device_type", "root", "tolerance_mm", "density_kg_m3", "material_card",
    "skip", "parts", "joints", "source_document",
})
PART_KEYS = frozenset({"density_kg_m3", "material_card", "skip"})
JOINT_KEYS = frozenset({
    "name", "label", "fc_type", "type", "parent", "child", "axis", "origin",
    "limits", "activated", "distance", "distance2",
})
LIMIT_KEYS = frozenset({
    "lower", "upper", "lower_enabled", "upper_enabled", "unit", "offset_m",
})

#: mm^3 -> m^3 e il fattore del tensore d'inerzia (mm^5 -> m^5), applicato
#: insieme alla densita' per ottenere kg*m^2.
_MM3_TO_M3 = 1e-9
_MM5_TO_M5 = 1e-15
_MM_TO_M = 1e-3

_ORTHONORMAL_TOL = 1e-6


class StepToIrError(RuntimeError):
    """Errore del ponte: STEP illeggibile, contratto incoerente, kernel assente."""


# --------------------------------------------------------------------------- #
# contratto
# --------------------------------------------------------------------------- #


def load_contract(path: str | Path) -> dict:
    """Legge e valida il contratto JSON.

    Valida la *forma* (chiavi note, tipi): il merito -- joint che puntano a
    body esistenti, limiti coerenti -- lo controlla ``cad.ir_schema`` sull'IR
    finale, che resta l'unico giudice.
    """
    contract_path = Path(path)
    if not contract_path.is_file():
        raise StepToIrError("contract not found: %s" % contract_path)
    try:
        with open(contract_path, encoding="utf-8") as handle:
            contract = json.load(handle)
    except ValueError as exc:
        raise StepToIrError("contract %s is not valid JSON: %s" % (contract_path, exc))
    if not isinstance(contract, dict):
        raise StepToIrError("contract %s must be a JSON object" % contract_path)
    _validate_contract(contract, str(contract_path))
    return contract


def _validate_contract(contract: dict, where: str) -> dict:
    """Chiavi chiuse, a ogni livello: un refuso si vede subito, non a valle.

    Vale sia per il file letto da disco sia per il dict passato a ``plan()``:
    le due strade non devono poter divergere.
    """
    unknown = sorted(set(contract) - CONTRACT_KEYS)
    if unknown:
        raise StepToIrError(
            "contract %s has unknown key(s): %s (known: %s)"
            % (where, ", ".join(unknown), ", ".join(sorted(CONTRACT_KEYS)))
        )
    parts = contract.get("parts", {})
    if not isinstance(parts, dict):
        raise StepToIrError("contract 'parts' must be an object keyed by part name")
    for name, entry in parts.items():
        if not isinstance(entry, dict):
            raise StepToIrError("contract parts[%r] must be an object" % name)
        extra = sorted(set(entry) - PART_KEYS)
        if extra:
            raise StepToIrError(
                "contract parts[%r] has unknown key(s): %s (known: %s)"
                % (name, ", ".join(extra), ", ".join(sorted(PART_KEYS)))
            )
    skip = contract.get("skip", [])
    if not isinstance(skip, list) or any(not isinstance(item, str) for item in skip):
        raise StepToIrError("contract 'skip' must be a list of part names")
    joints = contract.get("joints", [])
    if not isinstance(joints, list) or any(not isinstance(item, dict) for item in joints):
        raise StepToIrError("contract 'joints' must be a list of joint objects")
    for index, joint in enumerate(joints):
        extra = sorted(set(joint) - JOINT_KEYS)
        if extra:
            raise StepToIrError(
                "contract joints[%d] has unknown key(s): %s (known: %s)"
                % (index, ", ".join(extra), ", ".join(sorted(JOINT_KEYS)))
            )
        limits = joint.get("limits")
        if limits is not None:
            if not isinstance(limits, dict):
                raise StepToIrError("contract joints[%d].limits must be an object" % index)
            extra = sorted(set(limits) - LIMIT_KEYS)
            if extra:
                raise StepToIrError(
                    "contract joints[%d].limits has unknown key(s): %s (known: %s)"
                    % (index, ", ".join(extra), ", ".join(sorted(LIMIT_KEYS)))
                )
    return contract


def _number_of(value: Any, where: str) -> float:
    """Un numero dichiarato, altrimenti un errore che dice quale chiave."""
    try:
        return float(value)
    except (TypeError, ValueError):
        raise StepToIrError("%s: expected a number, got %r" % (where, value))


def _density_of(entry: dict, default: float, where: str) -> float:
    value = entry.get("density_kg_m3", default)
    density = _number_of(value, "%s density_kg_m3" % where)
    if not density > 0:
        raise StepToIrError("%s: density_kg_m3 must be > 0 (got %r)" % (where, density))
    return density


# --------------------------------------------------------------------------- #
# geometria: placement, massa, inerzia
# --------------------------------------------------------------------------- #


def _quaternion(rotation: List[List[float]], where: str) -> List[float]:
    """Quaternione unitario (x, y, z, w) da una matrice di rotazione 3x3."""
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quat = [
            (rotation[2][1] - rotation[1][2]) / scale,
            (rotation[0][2] - rotation[2][0]) / scale,
            (rotation[1][0] - rotation[0][1]) / scale,
            0.25 * scale,
        ]
    elif rotation[0][0] > rotation[1][1] and rotation[0][0] > rotation[2][2]:
        scale = math.sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]) * 2.0
        quat = [
            0.25 * scale,
            (rotation[0][1] + rotation[1][0]) / scale,
            (rotation[0][2] + rotation[2][0]) / scale,
            (rotation[2][1] - rotation[1][2]) / scale,
        ]
    elif rotation[1][1] > rotation[2][2]:
        scale = math.sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]) * 2.0
        quat = [
            (rotation[0][1] + rotation[1][0]) / scale,
            0.25 * scale,
            (rotation[1][2] + rotation[2][1]) / scale,
            (rotation[0][2] - rotation[2][0]) / scale,
        ]
    else:
        scale = math.sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]) * 2.0
        quat = [
            (rotation[0][2] + rotation[2][0]) / scale,
            (rotation[1][2] + rotation[2][1]) / scale,
            0.25 * scale,
            (rotation[1][0] - rotation[0][1]) / scale,
        ]
    norm = math.sqrt(sum(component * component for component in quat))
    if norm == 0.0:  # pragma: no cover - una matrice di rotazione non e' nulla
        raise StepToIrError("%s: rotation matrix has no quaternion" % where)
    return [component / norm for component in quat]


def _pose_from_transform(transform: Any, where: str) -> List[float]:
    """Pose SI ``[x, y, z, qx, qy, qz, qw]`` da una matrice row-major 4x4 in mm.

    La traslazione sta nella colonna 4 delle righe 1-3 (convenzione di
    ``cad.step_import``).  Una matrice non ortormale (scala o riflessione) non e'
    rappresentabile in SDF: il ponte si ferma invece di emettere un placement
    che nessun solver sa leggere.
    """
    if not isinstance(transform, (list, tuple)) or len(transform) != 16:
        raise StepToIrError("%s: transform must be 16 numbers, got %r"
                            % (where, transform))
    matrix = [float(value) for value in transform]
    rotation = [
        [matrix[0], matrix[1], matrix[2]],
        [matrix[4], matrix[5], matrix[6]],
        [matrix[8], matrix[9], matrix[10]],
    ]
    for row in range(3):
        for other in range(3):
            dot = sum(rotation[row][k] * rotation[other][k] for k in range(3))
            expected = 1.0 if row == other else 0.0
            if abs(dot - expected) > _ORTHONORMAL_TOL:
                raise StepToIrError(
                    "%s: placement is not a rigid transform (row %d . row %d = %.6f): "
                    "scaled or mirrored occurrences cannot be expressed in the IR"
                    % (where, row + 1, other + 1, dot)
                )
    det = (rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
           - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
           + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0]))
    # Un rispecchiamento (det = -1) ha righe ortonormali ma non e' una posa:
    # SDF non sa esprimerlo e accettarlo specchierebbe la geometria in silenzio.
    if abs(det - 1.0) > _ORTHONORMAL_TOL:
        raise StepToIrError(
            "%s: placement is not a right-handed rotation (det = %.6f): mirrored "
            "occurrences cannot be expressed in the IR" % (where, det)
        )
    translation = [matrix[3] * 1e-3, matrix[7] * 1e-3, matrix[11] * 1e-3]
    return translation + _quaternion(rotation, where)


def _mass_properties(shape: Any, density: float, where: str) -> Dict[str, Any]:
    """Volume, massa, CoM e tensore d'inerzia del B-rep, in SI, frame locale.

    Il kernel lavora in millimetri: volume in mm^3, matrice d'inerzia (gia'
    riferita al centro di massa, come documenta OCCT) in mm^5.  Le due costanti
    di conversione sono verificate da ``tests/test_step_to_ir.py`` su un cubo di
    lato noto, dove il risultato analitico e' ``m (a^2 + b^2) / 12``.
    """
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    props = GProp_GProps()
    try:
        BRepGProp.VolumeProperties_s(shape, props)
    except Exception as exc:  # pragma: no cover - dipende dal kernel
        raise StepToIrError("%s: the kernel failed on the shape: %s" % (where, exc))
    volume_mm3 = abs(float(props.Mass()))
    if volume_mm3 <= 0.0:
        raise StepToIrError("%s: solid has no volume: not a closed B-rep" % where)
    volume_m3 = volume_mm3 * _MM3_TO_M3
    mass_kg = volume_m3 * density

    centre = props.CentreOfMass()
    com = [float(centre.Coord(axis)) * 1e-3 for axis in (1, 2, 3)]
    matrix = props.MatrixOfInertia()
    inertia = [
        [float(matrix.Value(row, column)) * _MM5_TO_M5 * density for column in (1, 2, 3)]
        for row in (1, 2, 3)
    ]
    return {
        "volume_m3": volume_m3,
        "mass_kg": mass_kg,
        "com_m": com,
        "inertia_kg_m2": inertia,
    }


def _tessellate(shape: Any, tolerance_mm: float, where: str) -> None:
    """Tessella il B-rep (senza di che' l'STL esce vuoto)."""
    from OCP.BRepMesh import BRepMesh_IncrementalMesh

    BRepMesh_IncrementalMesh(shape, float(tolerance_mm))


def _stl_vertex_count(path: Path, where: str) -> int:
    """Vertici *distinti* del file STL appena scritto.

    Non si contano i nodi del B-rep (che OCCT duplica per faccia) ne' i vertici
    grezzi del file (3 per triangolo): si legge il file e si deduplicano le
    posizioni, cosi' il numero e' quello che vedra' chiunque apra la mesh — la
    stessa risposta che da' ``trimesh`` in ``sdf_artifacts``.
    """
    import struct

    blob = path.read_bytes()
    if len(blob) < 84:
        raise StepToIrError("%s: %s is not a binary STL (%d bytes)"
                            % (where, path, len(blob)))
    triangles = struct.unpack_from("<I", blob, 80)[0]
    expected = 84 + triangles * 50
    if triangles == 0 or len(blob) != expected:
        raise StepToIrError(
            "%s: %s holds %d triangle(s) but %d byte(s) (expected %d): the mesh "
            "is truncated" % (where, path, triangles, len(blob), expected)
        )
    seen = set()
    for index in range(triangles):
        offset = 84 + index * 50 + 12
        for corner in range(3):
            seen.add(struct.unpack_from("<3f", blob, offset + corner * 12))
    return len(seen)


def _in_metres(shape: Any, where: str) -> Any:
    """Copia della shape scalata da mm a m (origine fissa).

    Il kernel lavora in mm, ma l'IR e' SI ovunque (``docs/ir-v1.md``): le mesh su
    disco devono stare in metri come le pose.  Senza questa scala un corpo di
    100 mm arriva a SDF lungo 100 m, e ``sdf_artifacts`` ne calcola il volume
    etichettandolo ``m^3``.  La scala si applica a una copia, non alla shape
    usata per massa e inerzia.
    """
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Pnt, gp_Trsf

    transform = gp_Trsf()
    transform.SetScale(gp_Pnt(0.0, 0.0, 0.0), _MM_TO_M)
    builder = BRepBuilderAPI_Transform(shape, transform, True)
    if not builder.IsDone():  # pragma: no cover - una scala uniforme non fallisce
        raise StepToIrError("%s: cannot rescale the shape to metres" % where)
    return builder.Shape()


def _write_stl(shape: Any, path: Path, tolerance_mm: float, where: str) -> int:
    """Tessella il B-rep, ne scrive l'STL binario in **metri** e restituisce i vertici distinti.

    La shape va **gia'** meshata: in questa build di OCCT ``StlAPI_Writer`` non
    invoca la mesh da solo e scrive un file vuoto (verificato: senza
    ``BRepMesh_IncrementalMesh`` ``Write`` torna ``False`` e lascia 0 byte).
    Il file su disco e' comunque il verdetto finale: la firma di ``Write``
    cambia fra le build, e il conteggio dei vertici si legge da li'.

    La tolleranza di tessellatura resta in mm (come nel contratto) e viene
    convertita insieme alla geometria.
    """
    from OCP.StlAPI import StlAPI_Writer

    metres = _in_metres(shape, where)
    _tessellate(metres, tolerance_mm * _MM_TO_M, where)
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = StlAPI_Writer()
    writer.ASCIIMode = False  # binario: meta' spazio, stessa fedelta'
    writer.Write(metres, str(path))
    if not path.is_file() or path.stat().st_size == 0:
        raise StepToIrError("%s: cannot write the mesh to %s" % (where, path))
    return _stl_vertex_count(path, where)


# --------------------------------------------------------------------------- #
# conversione
# --------------------------------------------------------------------------- #


def _part_keys(part: dict, body: str) -> List[str]:
    """Chiavi con cui una parte puo' essere citata nel contratto."""
    keys = [body]
    for candidate in (part.get("name"), part.get("path"), part.get("label")):
        if isinstance(candidate, str) and candidate:
            keys.append(candidate)
            keys.append(naming.sanitize(candidate))
    return keys


def plan(step_ir: dict, *, contract: Optional[dict] = None,
         device_type: Optional[str] = None,
         density: Optional[float] = None,
         material_card: Optional[str] = None,
         tolerance_mm: Optional[float] = None,
         source_document: Optional[str] = None,
         now: Optional[str] = None) -> dict:
    """Fase dichiarativa del ponte: chi resta, con che nome, con che densita'.

    Non tocca la geometria, quindi gira anche **senza** il kernel OCCT ed e' la
    meta' verificabile senza OCP.  Restituisce ``meta`` (completo tranne cio'
    che dipende dalla geometria), i ``parts`` tenuti con il nome IR risolto,
    gli ``skipped``, i ``joints`` verificati contro i body sopravvissuti e
    ``joints_declared`` (falso = il contratto non dichiara topologia).
    """
    if step_ir.get("schema") != step_import.IR_SCHEMA:
        raise StepToIrError(
            "expected a %s document, got %r"
            % (step_import.IR_SCHEMA, step_ir.get("schema"))
        )
    contract = dict(contract or {})
    if contract:
        _validate_contract(contract, "passed to plan()")
    source = step_ir.get("source") or {}
    parts = step_ir.get("parts") or []
    if not parts:
        raise StepToIrError("the STEP IR carries no part: nothing to convert")

    device = device_type or contract.get("device_type") or DEFAULT_DEVICE_TYPE
    # Una chiave *presente* ma senza valore (null) e' un errore di chi scrive il
    # contratto, non un "usa il default": l'assenza e' assenza, null e' null.
    if density is not None:
        base_density = _density_of({"density_kg_m3": density}, DEFAULT_DENSITY_KG_M3, "cli")
    elif "density_kg_m3" in contract:
        base_density = _density_of(contract, DEFAULT_DENSITY_KG_M3, "contract")
    else:
        base_density = DEFAULT_DENSITY_KG_M3
    if "tolerance_mm" in contract:
        tolerance = _number_of(contract["tolerance_mm"], "contract tolerance_mm")
    else:
        tolerance = DEFAULT_TOLERANCE_MM
    if tolerance_mm is not None:
        tolerance = _number_of(tolerance_mm, "cli tolerance_mm")
    if not tolerance > 0:
        raise StepToIrError("tessellation tolerance must be > 0 (got %r)" % tolerance)

    declared_parts = contract.get("parts") or {}
    skipped = list(contract.get("skip") or [])

    kept: List[dict] = []
    taken: set = set()
    skipped_paths: List[str] = []

    for part in parts:
        raw_name = part.get("name") or "unnamed"
        body = naming.unique(naming.sanitize(raw_name), taken)
        keys = _part_keys(part, body)
        entry = next((declared_parts[key] for key in keys if key in declared_parts), {})
        if entry.get("skip") or any(key in skipped for key in keys):
            skipped_paths.append(part.get("path") or raw_name)
            continue

        # Scheda e densita' devono raccontare la stessa cosa: se una scheda e'
        # dichiarata, la sua densita' viene dalla tabella di material_props
        # (una sola fonte), e una densita' scritta a mano vince sulla tabella.
        declared_card = (entry.get("material_card") or material_card
                         or contract.get("material_card"))
        if "density_kg_m3" in entry:
            part_density = _density_of(entry, base_density, "contract parts[%r]" % raw_name)
            density_source = "contract"
        elif density is not None:
            part_density, density_source = base_density, "cli"
        elif "density_kg_m3" in contract:
            part_density, density_source = base_density, "contract:default"
        elif declared_card:
            from_card, matched = material_props.keyword_density(declared_card)
            if from_card is None:
                raise StepToIrError(
                    "card %r has no density in material_props.%s: declare "
                    "density_kg_m3 for part %r instead of guessing"
                    % (declared_card, matched or "DENSITY_KEYWORDS", raw_name)
                )
            part_density, density_source = from_card, "material_card"
        else:
            part_density, density_source = base_density, "step_to_ir:default_steel"

        if entry.get("material_card") or "material_card" in contract:
            card, material_source = declared_card, "contract"
        elif material_card:
            card, material_source = declared_card, "cli"
        elif density_source == "step_to_ir:default_steel":
            # Densita' e scheda vanno d'accordo: 7850 kg/m3 *e'* acciaio.
            card, material_source = DEFAULT_MATERIAL_CARD, "step_to_ir"
        else:
            # Una densita' dichiarata senza scheda non diventa "acciaio" per
            # comodita': la scheda resta dichiaratamente ignota ('card' e'
            # obbligatorio nello schema, 'unspecified' e' la verita').
            card, material_source = "unspecified", "unspecified"
        kept.append({
            "part": part,
            "path": part.get("path") or raw_name,
            "body": body,
            "label": raw_name,
            "density_kg_m3": part_density,
            "density_source": density_source,
            "material_card": card,
            "material_source": material_source,
        })

    if not kept:
        raise StepToIrError(
            "every part was excluded by the contract (%s): the IR would have no body"
            % ", ".join(skipped_paths or ["?"])
        )

    names = [entry["body"] for entry in kept]
    joints = _convert_joints(contract.get("joints") or [], names)

    root = contract.get("root")
    if root is not None:
        root = naming.sanitize(root)
        if root not in names:
            raise StepToIrError(
                "contract root %r is not one of the bodies (%s)"
                % (root, ", ".join(sorted(names)))
            )

    meta = {
        "generator": "cad.step_to_ir",
        "emitter_version": ir_schema.emitter_version(),
        "device_type": device,
        "source_hash": "sha256:" + str(source.get("sha256") or ""),
        "units": "SI (m, kg, rad)",
        "created": now or datetime.datetime.now().replace(microsecond=0).isoformat(),
        "source_document": source_document or source.get("file_description")
        or source.get("path") or "",
        "source_file": source.get("path") or "",
        "root": root,
        "tessellation_tolerance_mm": tolerance,
    }
    return {
        "meta": meta,
        "device_type": device,
        "tolerance_mm": tolerance,
        "root": root,
        "parts": kept,
        "skipped": skipped_paths,
        "joints": joints,
        "joints_declared": bool(contract.get("joints")),
    }


def _sub_solids(shape: Any, where: str) -> List[Any]:
    """I solidi OCCT che compongono la forma di una parte.

    Una parte STEP puo' essere un ``TopoDS_Compound`` con N solidi (es. una
    guida lineare modellata come corpo + pattino nella stessa parte): il ponte
    deve emetterne uno per solido, con massa, inerzia, CoM e mesh propri, non
    schiacciarli in un unico solido aggregato.  Per una parte a un solo solido
    (il caso normale) la lista ha un elemento, cioe' la forma stessa.
    """
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer

    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    found = []
    while explorer.More():
        found.append(explorer.Current())
        explorer.Next()
    if not found:
        # Nessun TopAbs_SOLID sotto la forma (es. shell/compound vuoto): non e'
        # un B-rep chiuso e _mass_properties lo rifiutera' con un messaggio
        # esplicito, invece di emettere silenziosamente zero solidi.
        raise StepToIrError(
            "%s: the shape contains no OCCT solid (not a closed B-rep)" % where)
    return found


def _solid_of(entry: dict, shape: Any, mesh_root: Optional[str],
              tolerance: float, *, solid_name: Optional[str] = None,
              label: Optional[str] = None) -> dict:
    """Solido IR di una parte: massa/inerzia dal B-rep, mesh se ``mesh_root``.

    ``solid_name`` distingue i solidi di una parte multi-solido (il primo
    mantiene il nome del body, gli altri ricevono ``<body>_2``, ...); ``label``
    e' il nome CAD della parte, ripetuto su ogni solido per tracciabilita'.
    """
    where = "part %r" % entry["path"]
    name = solid_name or entry["body"]
    solid: Dict[str, Any] = {
        "name": name,
        "label": entry["label"] if label is None else label,
        "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "density_kg_m3": entry["density_kg_m3"],
        "density_source": entry["density_source"],
        "material": {"card": entry["material_card"], "source": entry["material_source"]},
    }
    props = _mass_properties(shape, entry["density_kg_m3"], where)
    solid["reference_volume_m3"] = round(props["volume_m3"], 12)
    # ``reference_mass_kg`` e' obbligatorio nello schema v1: la massa si
    # calcola, non si presume -- ed e' il motivo per cui il ponte vuole la
    # forma, non solo il conteggio dei solidi.
    solid["reference_mass_kg"] = round(props["mass_kg"], 9)
    solid["reference_com_m"] = [round(value, 9) for value in props["com_m"]]
    solid["reference_inertia_kg_m2"] = [
        [round(value, 12) for value in row] for row in props["inertia_kg_m2"]
    ]
    if mesh_root is not None:
        relative = naming.visual_mesh_name(entry["body"], name)
        solid["num_vertices"] = _write_stl(shape, Path(mesh_root) / relative,
                                           tolerance, where)
        solid["visual_mesh"] = "model://" + relative
        solid["stl_path"] = relative
    return solid


def convert(step_ir: dict, *, shapes: dict,
            contract: Optional[dict] = None,
            device_type: Optional[str] = None,
            density: Optional[float] = None,
            material_card: Optional[str] = None,
            tolerance_mm: Optional[float] = None,
            mesh_root: Optional[str] = None,
            source_document: Optional[str] = None,
            now: Optional[str] = None,
            warn: bool = True) -> dict:
    """Trasforma un IR ``step-ir/1`` in un IR ``cad-ir/1`` (non valida: lo fa il chiamante).

    ``shapes`` sono le forme OCCT per path (da ``step_import.import_step``,
    riempite da ``shapes_out``): servono per massa, inerzia e mesh.  Una parte
    tenuta senza forma e' un errore, non un IR mutilato: senza il B-rep non si
    puo' riempire ``reference_mass_kg``, che lo schema v1 pretende.
    ``mesh_root`` (opzionale) e' la directory sotto cui scrivere
    ``meshes/visual/<body>__<solid>.stl``.
    """
    blueprint = plan(step_ir, contract=contract, device_type=device_type,
                     density=density, material_card=material_card,
                     tolerance_mm=tolerance_mm, source_document=source_document,
                     now=now)

    missing = [entry["path"] for entry in blueprint["parts"]
               if (shapes or {}).get(entry["path"]) is None]
    if missing:
        raise StepToIrError(
            "no OCCT shape for %s: massa e inerzia non si possono inventare "
            "(passa le forme da step_import.import_step(shapes_out=...))"
            % ", ".join(sorted(missing))
        )

    bodies: List[dict] = []
    for entry in blueprint["parts"]:
        part = entry["part"]
        where = "part %r" % entry["path"]
        # Una parte puo' essere un compound di N solidi reali: si emette un
        # solido IR per ciascuno, con nome unico nel body (il primo tiene il
        # nome del body, gli altri <body>_2, ...).  ``num_solids`` e' il numero
        # di solidi *emessi*, non il conteggio di step-ir/1: e' proprio la
        # riconciliazione che il validator fail-fast pretende
        # (``num_solids == len(solids)``).
        pieces = _sub_solids(shapes[entry["path"]], where)
        taken_solids: set = set()
        solids = [
            _solid_of(entry, piece, mesh_root, blueprint["tolerance_mm"],
                      solid_name=naming.unique(entry["body"], taken_solids))
            for piece in pieces
        ]
        bodies.append({
            "name": entry["body"],
            "label": entry["label"],
            "pose": _pose_from_transform(part.get("transform"), where),
            "solids": solids,
            "num_solids": len(solids),
            "cad_path": entry["path"],
            "bbox": part.get("bbox"),
        })

    document: Dict[str, Any] = {
        "schema": ir_schema.IR_SCHEMA,
        "meta": blueprint["meta"],
        "bodies": bodies,
        "joints": blueprint["joints"],
        "skipped_joints": [],
        "frames": [],
        "anchors": [],
        "sensors": [],
        "links": [],
    }

    if warn:
        defaulted = [body["name"] for body in bodies
                     if body["solids"][0]["density_source"] == "step_to_ir:default_steel"]
        if defaulted:
            sys.stderr.write(
                "[step_to_ir] attenzione: %d parte/i (%s) senza densita' dichiarata "
                "usano %.1f kg/m3 (acciaio) -- vedi 'density_source' nei solidi\n"
                % (len(defaulted), ", ".join(defaulted), DEFAULT_DENSITY_KG_M3))
        if not blueprint["joints_declared"]:
            sys.stderr.write(
                "[step_to_ir] attenzione: il contratto non dichiara 'joints': "
                "l'IR esce senza topologia. Un giunto non e' un fatto geometrico "
                "e non si deduce dal CAD (--) dichiaralo nel contratto.\n")
        if blueprint["skipped"]:
            sys.stderr.write(
                "[step_to_ir] %d parte/i escluse dal contratto: %s\n"
                % (len(blueprint["skipped"]), ", ".join(blueprint["skipped"])))
    return document


def _limits_of(joint: dict, fc_type: str, where: str) -> dict:
    """Limiti nel dialetto v1: le unita' si deducono, i numeri si dichiarano."""
    declared = joint.get("limits")
    if not isinstance(declared, dict):
        raise StepToIrError(
            "%s: 'limits' is required (lower/upper, in SI): the bridge does not "
            "invent a stroke, it carries the numbers you declare" % where
        )
    for key in ("lower", "upper"):
        if key not in declared:
            raise StepToIrError("%s: limits.%s is required" % (where, key))
    unit = declared.get("unit") or _JOINT_UNITS.get(str(fc_type).lower())
    if unit is None:
        raise StepToIrError(
            "%s: limits.unit is required for fc_type %r (prismatic -> 'm', "
            "revolute -> 'rad')" % (where, fc_type)
        )
    if unit not in ("m", "rad"):
        raise StepToIrError("%s: limits.unit must be 'm' or 'rad' (got %r)"
                            % (where, unit))
    limits = {
        "lower": float(declared["lower"]),
        "upper": float(declared["upper"]),
        "lower_enabled": bool(declared.get("lower_enabled", True)),
        "upper_enabled": bool(declared.get("upper_enabled", True)),
        "unit": unit,
        "offset_m": float(declared.get("offset_m", 0.0)),
    }
    if limits["lower"] > limits["upper"]:
        raise StepToIrError("%s: limits.lower > limits.upper (%r > %r)"
                            % (where, limits["lower"], limits["upper"]))
    return limits


def _convert_joints(declared: List[dict], names: Iterable[str]) -> List[dict]:
    """Nomina, completa e verifica i giunti dichiarati contro i body sopravvissuti.

    Il contratto e' un dialetto *semplificato* del giunto v1: l'ingegnere
    dichiara ``fc_type``, ``parent``, ``child``, ``axis`` e i numeri del
    movimento, il ponte aggiunge cio' che si deduce (l'unita' dal tipo di
    giunto, ``activated``, ``offset_m``) e non aggiunge nulla di inventato.
    """
    names = set(names)
    taken: set = set()
    joints: List[dict] = []
    for index, joint in enumerate(declared):
        where = "contract joints[%d]" % index
        fc_type = joint.get("fc_type") or joint.get("type")
        if not fc_type:
            raise StepToIrError(
                "%s: 'fc_type' is required (Fixed, Revolute, Slider, "
                "Cylindrical, Ball, ...)" % where
            )
        parent = naming.sanitize(joint.get("parent") or "")
        child = naming.sanitize(joint.get("child") or "")
        for role, name in (("parent", parent), ("child", child)):
            if name not in names:
                raise StepToIrError(
                    "%s: %s %r is not a body of this model (%s)"
                    % (where, role, joint.get(role), ", ".join(sorted(names)))
                )
        axis = joint.get("axis")
        if axis is None and fc_type.lower() == "fixed":
            # Un giunto fisso non ha un asse: lo schema ne pretende uno
            # comunque, quindi si dichiara l'asse che il CAD non usa.
            axis = [1.0, 0.0, 0.0]
        if axis is None:
            raise StepToIrError(
                "%s: 'axis' is required for %s: the DOF direction is a design "
                "decision, not something the bridge can guess" % (where, fc_type)
            )
        axis = [float(value) for value in axis]
        if len(axis) != 3 or all(value == 0.0 for value in axis):
            raise StepToIrError("%s: axis must be a non-zero 3-vector (got %r)"
                                % (where, joint.get("axis")))

        record: Dict[str, Any] = {
            "name": naming.unique(naming.joint_name(joint.get("name") or child), taken),
            "label": joint.get("label") or joint.get("name") or child,
            "fc_type": fc_type,
            "parent": parent,
            "child": child,
            "axis": axis,
            "limits": _limits_of(joint, fc_type, where),
            "activated": bool(joint.get("activated", True)),
        }
        for extra in ("origin", "distance", "distance2"):
            if extra in joint:
                record[extra] = joint[extra]
        joints.append(record)
    return joints


def convert_step(step_path: str | Path, *, mesh_root: Optional[str] = None,
                 contract: Optional[dict] = None, **kwargs: Any) -> dict:
    """Legge il STEP (una volta sola) e restituisce l'IR ``cad-ir/1`` validato.

    ``contract`` puo' essere un dict gia' letto o il path di un file.
    """
    if isinstance(contract, (str, Path)):
        contract = load_contract(contract)
    # Le forme si chiedono comunque al lettore: senza di esse non c'e' massa.
    # Le mesh su disco invece si scrivono solo se e' stato indicato un root.
    shapes: dict = {}
    step_ir = step_import.import_step(step_path, shapes_out=shapes)
    document = convert(step_ir, contract=contract, shapes=shapes,
                       mesh_root=mesh_root, **kwargs)
    ir_schema.validate(document, source=str(step_path))
    return document


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m cad.step_to_ir",
        description="Ponte step-ir/1 -> cad-ir/1: geometria dal STEP, "
                    "giunti e materiali dal contratto.")
    parser.add_argument("--step", required=True, help="file STEP sorgente")
    parser.add_argument("--ws", default=None,
                        help="root dei mesh visuali (default: directory corrente)")
    parser.add_argument("--contract", default=None,
                        help="contratto JSON (giunti, materiali, parti da saltare)")
    parser.add_argument("--device-type", default=None,
                        help="classe di dispositivo WEC (default: contract o 'generic')")
    parser.add_argument("--density", type=float, default=None,
                        help="densita' kg/m3 per le parti non elencate nel contratto")
    parser.add_argument("--material-card", default=None,
                        help="carta materiale di default")
    parser.add_argument("--tolerance-mm", type=float, default=None,
                        help="tolleranza di tessellatura (default: contract o %.1f)"
                             % DEFAULT_TOLERANCE_MM)
    parser.add_argument("--no-meshes", action="store_true",
                        help="non tessellare: i solidi restano senza stl_path")
    parser.add_argument("--out", default=None,
                        help="file di uscita; '-' o assente = stdout")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    mesh_root = None if args.no_meshes else (args.ws or os.getcwd())
    try:
        document = convert_step(
            args.step,
            mesh_root=mesh_root,
            contract=args.contract,
            device_type=args.device_type,
            density=args.density,
            material_card=args.material_card,
            tolerance_mm=args.tolerance_mm,
        )
    except (StepToIrError, step_import.StepImportError, ir_schema.IRSchemaError) as exc:
        sys.stderr.write("step_to_ir: %s\n" % exc)
        return 1
    text = json.dumps(document, indent=2, ensure_ascii=False, sort_keys=False) + "\n"
    if args.out in (None, "-"):
        sys.stdout.write(text)
        return 0
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(text)
    print("IR scritto: %s (%d body, %d joint)"
          % (out, len(document["bodies"]), len(document["joints"])))
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
