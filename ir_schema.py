"""ir_schema.py -- fail-fast validator for the versioned CAD IR.

The IR is ``config/cad_manifest.json``. Its contract is the JSON Schema in
``schema/cad_ir_v1.schema.json``, tagged ``cad-to-gazebo/cad-ir/1``; the human
contract (sections, versioning, migration) is ``docs/ir-v1.md``.

Validation has two passes and **aborts the export before anything is written**:

1. **structural** -- the JSON Schema document, evaluated with ``jsonschema``.
   Types, required keys, name grammar, pose arity, unit enums.
2. **semantic** -- the invariants a JSON Schema cannot express:
   * names are unique inside their scope (bodies, joints, frames, anchors,
     sensors, and the solids of a body);
   * every ``joint.parent`` / ``joint.child`` names an existing body;
   * ``meta.root`` names an existing body;
   * every ``frame.body``, ``anchor.body`` / ``anchor.frame``,
     ``sensor.frame`` resolves;
   * every pose carries a unit quaternion;
   * every ``reference_inertia_kg_m2`` is symmetric with a non-negative
     diagonal;
   * every joint ``axis`` is non-zero;
   * ``num_solids`` agrees with ``len(solids)`` when present.

Public API::

    from cad import ir_schema
    ir_schema.validate(manifest)                 # raises IRSchemaError
    ir_schema.validate_file("config/cad_manifest.json")
    ir_schema.collect_errors(manifest)           # list[str], never raises
    ir_schema.migrate_legacy(old_manifest)       # pre-v1 -> v1 (documented)

CLI::

    python -m cad.ir_schema config/cad_manifest.json [more.json ...]
    python -m cad.ir_schema --allow-legacy old.json     # migrate then validate
    python -m cad.ir_schema --print-schema

Exit status is 0 when every document validates, 1 when one does not (the
reasons go to stderr).
"""

from __future__ import annotations

import argparse
import copy
import datetime
import json
import os
import sys
from typing import Any, Dict, List, Optional, Sequence

if __package__ in (None, ""):  # allow ``python ir_schema.py`` from the repo root
    _pkg_root = os.path.dirname(os.path.abspath(__file__))
    if _pkg_root not in sys.path:
        sys.path.insert(0, _pkg_root)

__all__ = [
    "IR_SCHEMA",
    "IR_VERSION",
    "IRSchemaError",
    "PACKAGE_VERSION",
    "SCHEMA_PATH",
    "collect_errors",
    "emitter_version",
    "load_schema",
    "looks_legacy",
    "main",
    "migrate_legacy",
    "validate",
    "validate_file",
]

#: Version tag of the contract this module enforces. It is the value of the
#: top-level ``schema`` key of a valid document.
IR_SCHEMA = "cad-to-gazebo/cad-ir/1"

#: Revision of the *contract* (documentation + schema), not of the producers.
IR_VERSION = "1.0.0"

#: Version of this package as an IR emitter. Kept in sync with pyproject.toml
#: by ``tests/test_ir_schema.py::test_package_version_matches_pyproject``.
PACKAGE_VERSION = "0.1.0"

#: Canonical unit string of the IR (SI, metres/kilograms/radians).
IR_UNITS = "SI (m, kg, rad)"

SCHEMA_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "schema", "cad_ir_v1.schema.json")

#: Tolerances for the semantic pass.
QUATERNION_TOL = 1.0e-6
INERTIA_REL_TOL = 1.0e-6
INERTIA_ABS_TOL = 1.0e-12
AXIS_TOL = 1.0e-12

#: Keys of ``meta`` that survive a legacy migration (the v1 ones are rebuilt).
_META_PASSTHROUGH = ("source_document", "source_file", "root",
                     "tessellation_tolerance_mm")

_IDENTITY_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
_DEFAULT_MATERIAL = {"card": "unspecified", "diffuse": [0.6, 0.6, 0.6, 1.0],
                     "ambient": [0.6, 0.6, 0.6, 1.0]}


class IRSchemaError(RuntimeError):
    """Raised when an IR document does not satisfy the v1 contract.

    ``errors`` is the full, ordered list of violations (JSON-pointer paths),
    so a failing export prints everything that is wrong in one shot instead of
    one problem per run.
    """

    def __init__(self, errors: Sequence[str], source: Optional[str] = None):
        self.errors = list(errors)
        self.source = source
        where = " ({})".format(source) if source else ""
        head = "IR non valido{}: {} violazione/i".format(where, len(self.errors))
        super().__init__("\n".join([head] + ["  - " + e for e in self.errors]))


def emitter_version() -> str:
    """``<package>/<version>`` of the emitter writing the IR."""
    return "cad-to-gazebo/{}".format(PACKAGE_VERSION)


# --------------------------------------------------------------------------- #
# schema loading (jsonschema is imported lazily: the message must be useful)
# --------------------------------------------------------------------------- #


def load_schema(path: Optional[str] = None) -> dict:
    """Return the JSON Schema document, raising :class:`IRSchemaError` if unusable."""
    schema_path = path or SCHEMA_PATH
    try:
        with open(schema_path, encoding="utf-8") as handle:
            return json.load(handle)
    except OSError as exc:
        raise IRSchemaError(["schema non leggibile: {}: {}".format(schema_path, exc)])
    except ValueError as exc:
        raise IRSchemaError(["schema non e' JSON valido: {}: {}".format(schema_path, exc)])


def _jsonschema_validator(schema: dict):
    try:
        from jsonschema import Draft202012Validator
    except ImportError as exc:  # pragma: no cover - depends on the host
        raise IRSchemaError([
            "il validatore IR richiede 'jsonschema' (dipendenza core del "
            "pacchetto): installala con `python -m pip install 'cad-to-gazebo'`"
        ]) from exc
    return Draft202012Validator(schema)


# --------------------------------------------------------------------------- #
# structural pass
# --------------------------------------------------------------------------- #


def _format_path(path: Sequence[Any]) -> str:
    """``('bodies', 0, 'pose')`` -> ``bodies[0].pose`` (same style as the semantic pass)."""
    text = ""
    for part in path:
        if isinstance(part, int):
            text += "[{}]".format(part)
        else:
            text += ("" if not text else ".") + str(part)
    return text or "<root>"


def _structural_errors(ir: Any, schema: dict) -> List[str]:
    validator = _jsonschema_validator(schema)
    errors = []
    for error in sorted(validator.iter_errors(ir), key=lambda e: list(e.absolute_path)):
        errors.append("{}: {}".format(_format_path(error.absolute_path), error.message))
    return errors


# --------------------------------------------------------------------------- #
# semantic pass
# --------------------------------------------------------------------------- #


def _dict_or_empty(value: Any) -> Dict[str, Any]:
    """Typed helper: a mapping, or an empty one when ``value`` is not a dict."""
    return value if isinstance(value, dict) else {}


def _names_of(entries: Any, key: str = "name") -> List[str]:
    out = []
    for entry in entries or []:
        if isinstance(entry, dict) and isinstance(entry.get(key), str):
            out.append(entry[key])
    return out


def _duplicates(names: Sequence[str]) -> List[str]:
    seen = set()
    dups = []
    for name in names:
        if name in seen and name not in dups:
            dups.append(name)
        seen.add(name)
    return dups


def _pose_errors(pose: Any, where: str) -> List[str]:
    if not isinstance(pose, list) or len(pose) != 7:
        return []
    try:
        x, y, z, qx, qy, qz, qw = (float(v) for v in pose)
    except (TypeError, ValueError):
        return ["{}.pose: non numerico".format(where)]
    norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    if abs(norm - 1.0) > QUATERNION_TOL:
        return ["{}.pose: quaternione non unitario (norma {:.9f})".format(where, norm)]
    return []


def _inertia_errors(inertia: Any, where: str) -> List[str]:
    if inertia is None:
        return []
    if (not isinstance(inertia, list) or len(inertia) != 3
            or any(not isinstance(row, list) or len(row) != 3 for row in inertia)):
        return ["{}.reference_inertia_kg_m2: attesa matrice 3x3".format(where)]
    flat = [[float(v) for v in row] for row in inertia]
    errors = []
    scale = max((abs(v) for row in flat for v in row), default=0.0)
    tol = max(INERTIA_ABS_TOL, INERTIA_REL_TOL * scale)
    for i in range(3):
        for j in range(i + 1, 3):
            if abs(flat[i][j] - flat[j][i]) > tol:
                errors.append(
                    "{0}.reference_inertia_kg_m2: non simmetrica "
                    "([{1}][{2}]={3:.6g} vs [{2}][{1}]={4:.6g})".format(
                        where, i, j, flat[i][j], flat[j][i]))
    for i in range(3):
        if flat[i][i] < -tol:
            errors.append(
                "{0}.reference_inertia_kg_m2: diagonale negativa "
                "[{1}][{1}]={2:.6g}".format(where, i, flat[i][i]))
    return errors


def _semantic_errors(ir: dict) -> List[str]:
    errors: List[str] = []
    meta = _dict_or_empty(ir.get("meta"))
    bodies = ir.get("bodies") or []
    links = ir.get("links") or []
    joints = ir.get("joints") or []
    frames = ir.get("frames") or []
    anchors = ir.get("anchors") or []
    sensors = ir.get("sensors") or []

    body_names = _names_of(bodies)
    legacy_names = _names_of(links)
    known = set(body_names) | set(legacy_names)

    for dup in _duplicates(body_names):
        errors.append("bodies: nome duplicato '{}'".format(dup))
    for dup in _duplicates(legacy_names):
        errors.append("links: nome duplicato '{}'".format(dup))
    for dup in _duplicates(_names_of(joints)):
        errors.append("joints: nome duplicato '{}'".format(dup))
    for dup in _duplicates(_names_of(frames)):
        errors.append("frames: nome duplicato '{}'".format(dup))
    for dup in _duplicates(_names_of(anchors)):
        errors.append("anchors: nome duplicato '{}'".format(dup))
    for dup in _duplicates(_names_of(sensors)):
        errors.append("sensors: nome duplicato '{}'".format(dup))

    for index, body in enumerate(bodies):
        if not isinstance(body, dict):
            continue
        where = "bodies[{}]".format(index)
        errors.extend(_pose_errors(body.get("pose"), where))
        solid_names = _names_of(body.get("solids"))
        for dup in _duplicates(solid_names):
            errors.append("{}.solids: nome duplicato '{}'".format(where, dup))
        declared = body.get("num_solids")
        if isinstance(declared, int) and declared != len(body.get("solids") or []):
            errors.append(
                "{}.num_solids={} ma {} solidi".format(
                    where, declared, len(body.get("solids") or [])))
        for sindex, solid in enumerate(body.get("solids") or []):
            if not isinstance(solid, dict):
                continue
            swhere = "{}.solids[{}]".format(where, sindex)
            errors.extend(_pose_errors(solid.get("pose"), swhere))
            errors.extend(_inertia_errors(solid.get("reference_inertia_kg_m2"), swhere))

    for index, joint in enumerate(joints):
        if not isinstance(joint, dict):
            continue
        where = "joints[{}]".format(index)
        name = joint.get("name", "?")
        parent = joint.get("parent")
        child = joint.get("child")
        if isinstance(parent, str) and parent not in known:
            errors.append(
                "{}.parent: '{}' non e' un body del documento".format(where, parent))
        if isinstance(child, str) and child not in known:
            errors.append(
                "{}.child: '{}' non e' un body del documento".format(where, child))
        if isinstance(parent, str) and parent == child:
            errors.append(
                "{}.parent/child: '{}.{}' referenziano lo stesso body".format(
                    where, name, parent))
        errors.extend(_pose_errors(joint.get("origin"), where))
        axis = joint.get("axis")
        if (isinstance(axis, list) and len(axis) == 3
                and all(isinstance(v, (int, float)) for v in axis)):
            if sum(float(v) * float(v) for v in axis) ** 0.5 <= AXIS_TOL:
                errors.append("{}.axis: asse nullo".format(where))
        limits = joint.get("limits")
        if isinstance(limits, dict):
            lower, upper = limits.get("lower"), limits.get("upper")
            if (isinstance(lower, (int, float)) and isinstance(upper, (int, float))
                    and float(lower) > float(upper)):
                errors.append(
                    "{}.limits: lower={} > upper={}".format(where, lower, upper))

    root = meta.get("root")
    if isinstance(root, str) and root and root not in known:
        errors.append("meta.root: '{}' non e' un body del documento".format(root))

    frame_names = set(_names_of(frames))
    for index, frame in enumerate(frames):
        if not isinstance(frame, dict):
            continue
        where = "frames[{}]".format(index)
        errors.extend(_pose_errors(frame.get("pose"), where))
        body = frame.get("body")
        if isinstance(body, str) and body not in known:
            errors.append("{}.body: '{}' non e' un body del documento".format(where, body))
    for index, anchor in enumerate(anchors):
        if not isinstance(anchor, dict):
            continue
        where = "anchors[{}]".format(index)
        errors.extend(_pose_errors(anchor.get("pose"), where))
        body = anchor.get("body")
        if isinstance(body, str) and body not in known:
            errors.append("{}.body: '{}' non e' un body del documento".format(where, body))
        frame = anchor.get("frame")
        if isinstance(frame, str) and frame not in frame_names and frame not in known:
            errors.append(
                "{}.frame: '{}' non e' un frame ne' un body".format(where, frame))
    for index, sensor in enumerate(sensors):
        if not isinstance(sensor, dict):
            continue
        where = "sensors[{}]".format(index)
        errors.extend(_pose_errors(sensor.get("pose"), where))
        frame = sensor.get("frame")
        if isinstance(frame, str) and frame not in frame_names and frame not in known:
            errors.append(
                "{}.frame: '{}' non e' un frame ne' un body".format(where, frame))

    return errors


# --------------------------------------------------------------------------- #
# public entry points
# --------------------------------------------------------------------------- #


def collect_errors(ir: Any, schema: Optional[dict] = None) -> List[str]:
    """Return every violation of ``ir``; never raises for a bad document."""
    if not isinstance(ir, dict):
        return ["<root>: il documento IR deve essere un oggetto JSON"]
    document = schema if schema is not None else load_schema()
    structural = _structural_errors(ir, document)
    if structural:
        return structural
    return _semantic_errors(ir)


def validate(ir: Any, schema: Optional[dict] = None,
             source: Optional[str] = None) -> None:
    """Raise :class:`IRSchemaError` when ``ir`` violates the v1 contract."""
    errors = collect_errors(ir, schema=schema)
    if errors:
        raise IRSchemaError(errors, source=source)


def validate_file(path: str, allow_legacy: bool = False) -> dict:
    """Load a manifest from disk and validate it; return the (migrated) dict.

    With ``allow_legacy`` a pre-v1 document is migrated in memory first (the
    file on disk is never touched): the export path uses the strict form so a
    producer cannot write a v1 tag over a legacy shape.
    """
    try:
        with open(path, encoding="utf-8") as handle:
            ir = json.load(handle)
    except OSError as exc:
        raise IRSchemaError(["manifest non leggibile: {}".format(exc)], source=path)
    except ValueError as exc:
        raise IRSchemaError(["manifest non e' JSON valido: {}".format(exc)], source=path)
    if allow_legacy and looks_legacy(ir):
        ir = migrate_legacy(ir)
    validate(ir, source=path)
    return ir


def looks_legacy(ir: Any) -> bool:
    """True when the document predates IR v1 (no ``schema`` tag)."""
    return isinstance(ir, dict) and ir.get("schema") != IR_SCHEMA


def migrate_legacy(ir: dict, *, now: Optional[str] = None) -> dict:
    """Upgrade a pre-v1 manifest to v1 **in memory** and return the copy.

    Documented in ``docs/ir-v1.md``; the steps are:

    * stamp ``schema = IR_SCHEMA`` and rebuild ``meta`` with the v1 fields
      (``emitter_version``, ``device_type``, ``source_hash``, ``units``,
      ``created``) keeping the legacy pass-through ones;
    * fold the flat ``links`` list into ``bodies``: a legacy link is both the
      link and its single geometry entry, so each one becomes a body with one
      solid carrying the link's own mass/material/mesh. A link that already
      carries ``solids`` keeps them and only gets a pose;
    * leave ``joints`` / ``skipped_joints`` untouched (their shape is unchanged
      between pre-v1 and v1) and drop the legacy ``links`` key.

    Raises :class:`IRSchemaError` if the result still does not validate.
    """
    if not isinstance(ir, dict):
        raise IRSchemaError(["<root>: il documento IR deve essere un oggetto JSON"])
    out = {k: copy.deepcopy(v) for k, v in ir.items()
           if k not in ("meta", "links", "schema")}
    meta_in = _dict_or_empty(ir.get("meta"))

    meta: Dict[str, Any] = {
        "generator": meta_in.get("generator") or "cad.ir_schema.migrate_legacy",
        "emitter_version": meta_in.get("emitter_version") or emitter_version(),
        "device_type": meta_in.get("device_type") or "generic",
        "source_hash": meta_in.get("source_hash") or "",
        "units": IR_UNITS,
        "created": meta_in.get("created") or (
            now or datetime.datetime.now().isoformat(timespec="seconds")),
    }
    for key in _META_PASSTHROUGH:
        if meta_in.get(key) is not None:
            meta[key] = copy.deepcopy(meta_in[key])

    bodies = [copy.deepcopy(b) for b in (ir.get("bodies") or [])]
    for link in ir.get("links") or []:
        if not isinstance(link, dict):
            continue
        incarnation = {k: copy.deepcopy(v) for k, v in link.items() if k != "solids"}
        solids = link.get("solids")
        if solids:
            incarnation["solids"] = [copy.deepcopy(s) for s in solids]
        else:
            incarnation["solids"] = [{
                "name": link.get("name"),
                "label": link.get("label", link.get("name")),
                "pose": copy.deepcopy(link.get("pose") or _IDENTITY_POSE),
                "density_kg_m3": link.get("density_kg_m3") or 1.0,
                "density_source": link.get("density_source", "legacy"),
                "reference_mass_kg": link.get("reference_mass_kg", 0.0),
                "material": copy.deepcopy(link.get("material") or _DEFAULT_MATERIAL),
                "stl_path": link.get("stl_path", ""),
                "visual_mesh": link.get("visual_mesh", ""),
                "collision_mesh": link.get("collision_mesh", ""),
            }]
        incarnation.setdefault("pose", copy.deepcopy(link.get("pose") or _IDENTITY_POSE))
        bodies.append(incarnation)

    out["schema"] = IR_SCHEMA
    out["meta"] = meta
    if bodies:
        out["bodies"] = bodies
    out.setdefault("joints", copy.deepcopy(ir.get("joints") or []))
    out.setdefault("skipped_joints", copy.deepcopy(ir.get("skipped_joints") or []))

    ids = _duplicates(_names_of(out.get("bodies")))
    if ids:
        raise IRSchemaError(
            ["migrazione legacy: nomi body duplicati {}".format(", ".join(ids))])
    validate(out)
    return out


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m cad.ir_schema",
        description="Valida i manifest del CAD IR v1 (fail-fast).")
    parser.add_argument("files", nargs="*", metavar="MANIFEST",
                        help="uno o piu' cad_manifest.json da validare")
    parser.add_argument("--allow-legacy", action="store_true",
                        help="migra i manifest pre-v1 in memoria prima di validare")
    parser.add_argument("--migrate", metavar="OUT",
                        help="scrive il manifest migrato (uno solo, richiede --allow-legacy)")
    parser.add_argument("--print-schema", action="store_true",
                        help="stampa lo schema JSON applicato")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    if args.print_schema:
        print(json.dumps(load_schema(), indent=2, ensure_ascii=False))
        return 0

    if not args.files:
        print("ir_schema: nessun manifest da validare", file=sys.stderr)
        return 1
    if args.migrate and len(args.files) > 1:
        print("ir_schema: --migrate accetta un solo manifest", file=sys.stderr)
        return 1

    failed = 0
    for path in args.files:
        try:
            if args.migrate:
                with open(path, encoding="utf-8") as handle:
                    ir = migrate_legacy(json.load(handle))
                with open(args.migrate, "w", encoding="utf-8") as handle:
                    json.dump(ir, handle, indent=2, ensure_ascii=False)
                    handle.write("\n")
            else:
                validate_file(path, allow_legacy=args.allow_legacy)
        except IRSchemaError as exc:
            print("ir_schema: {}".format(exc), file=sys.stderr)
            failed += 1
        else:
            print("ir_schema: OK {}".format(path))
    return 1 if failed else 0


if __name__ == "__main__":  # pragma: no cover - exercised through subprocesses
    raise SystemExit(main())
