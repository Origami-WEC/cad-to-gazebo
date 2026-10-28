#!/usr/bin/env python3
"""sdf_validate.py -- Gate SDF: parse and check a model against SDFormat.

Why this exists
---------------
`build_simulation.py` (maritime_ws) validated the emitted SDF with ``gz sdf -k``
*when the Gazebo CLI happened to be on PATH* and otherwise printed
``[ATTENZIONE] CLI 'gz' non trovato: salto la validazione SDF`` -- a build that
skips validation is a green build that proves nothing. C5 closes that hole:

* ``gz`` present -> ``gz sdf -p <file>``, the reference SDFormat parser (the
  same library ``gz sim`` uses). Its exit code is the verdict.
* ``gz`` absent  -> the embedded backend below: a real XML parse
  (``xml.etree.ElementTree``) plus SDFormat conformance checks, not a string
  comparison.

Two severity levels, deliberately
---------------------------------
* **errors**   -> what ``gz sdf`` rejects: malformed XML, missing required
  attributes/elements, unknown joint type, ``<parent>``/``<child>`` pointing at
  a link that does not exist, duplicate names, malformed ``<pose>``, inverted
  limits. These fail :func:`validate`.
* **warnings** -> physical hygiene that SDFormat silently defaults instead:
  missing ``<inertial>``, non-positive mass, non-normalised axis, a link with no
  ``<collision>``, a non-unit quaternion, a link disconnected from the main
  chain. ``gz`` accepts these and the model then behaves differently from the
  CAD it came from, which is exactly the failure this pipeline has to catch.

``--strict`` promotes warnings to failures for the E2E harness.

The embedded backend is a floor, not a replacement for the real parser: the
harness always prints which backend ran so nobody mistakes one for the other.

Usage
-----
  python -m cad.sdf_validate description/model.sdf
  python -m cad.sdf_validate --backend gz model.sdf   # fail if gz is missing
  python -m cad.sdf_validate --strict --quiet model.sdf
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import subprocess
import sys
from typing import Dict, List, Optional, Sequence, Tuple
from xml.etree import ElementTree as ET

#: Tolerance on the norm of a quaternion.
QUAT_TOL = 1.0e-6

#: Tolerance on the norm of a joint axis.
AXIS_TOL = 1.0e-4

#: Elements that must be numeric inside <inertia>.
INERTIA_ELEMENTS = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")

#: Joint types accepted by SDFormat 1.9 (sdformat.org/spec?elem=joint).
JOINT_TYPES = ("continuous", "revolute", "gearbox", "revolute2", "prismatic",
               "ball", "screw", "universal", "fixed")

#: Joint types whose <axis><xyz> is the DOF direction.
AXIS_TYPES = ("revolute", "revolute2", "prismatic", "screw", "universal")

BACKENDS = ("auto", "gz", "builtin")


class SdfError(RuntimeError):
    """Raised when a document violates the checks that were run.

    ``errors`` is the full ordered list of violations, so one run reports
    everything that is wrong instead of one problem per run.
    """

    def __init__(self, errors: Sequence[str], source: Optional[str] = None,
                 backend: Optional[str] = None,
                 warnings: Optional[Sequence[str]] = None):
        self.errors = list(errors)
        self.warnings = list(warnings or [])
        self.source = source
        self.backend = backend
        where = " ({})".format(source) if source else ""
        head = "SDF non valido{} [backend {}]: {} violazione/i".format(
            where, backend or "?", len(self.errors))
        lines = [head] + ["  - " + e for e in self.errors]
        if self.warnings:
            lines.append("  warning ({}):".format(len(self.warnings)))
            lines.extend("  * " + w for w in self.warnings)
        super().__init__("\n".join(lines))


def gz_available() -> bool:
    """True when the Gazebo/ignition CLI (`gz` or `ign`) is on PATH."""
    return gz_command() is not None


def gz_command() -> Optional[List[str]]:
    """The ``sdf -p`` command to run, or ``None`` when no CLI is installed."""
    for exe in ("gz", "ign"):
        path = shutil.which(exe)
        if path:
            return [path, "sdf", "-p"]
    return None


# --------------------------------------------------------------------------- #
# embedded backend
# --------------------------------------------------------------------------- #

def _text(element: Optional[ET.Element]) -> str:
    return (element.text or "").strip() if element is not None else ""


def _numbers(value: str, where: str, counts: Sequence[int],
             errors: List[str]) -> List[float]:
    parts = value.split()
    if len(parts) not in counts:
        errors.append("{}.0: attesi {} valori, trovati {}".format(
            where, " o ".join(str(c) for c in counts), len(parts)))
        return []
    try:
        return [float(p) for p in parts]
    except ValueError:
        errors.append("{}.0: valori non numerici in {!r}".format(where, value))
        return []


def _resolve(ref: str, known: set, nested: set) -> bool:
    """True when a joint/frame reference names something declared here.

    SDFormat allows ``world``, a link, a declared frame, or a scoped name
    (``nested_model::link``); the leading segment of a scoped name must be a
    nested model or a link of this scope.
    """
    if ref == "world":
        return True
    if "::" in ref:
        head = ref.split("::", 1)[0]
        return head in nested or head in known
    return ref in known


def _check_pose(element: ET.Element, where: str, errors: List[str],
                warnings: List[str]) -> None:
    for index, pose in enumerate(element.findall("pose")):
        pwhere = "{}.pose[{}]".format(where, index)
        values = _numbers(_text(pose), pwhere, (6, 7), errors)
        if len(values) == 7:
            norm = math.sqrt(sum(v * v for v in values[3:]))
            if abs(norm - 1.0) > QUAT_TOL:
                warnings.append("{}: quaternione non unitario (norma "
                                "{:.9f})".format(pwhere, norm))


def _check_link(link: ET.Element, where: str, errors: List[str],
                warnings: List[str], static: bool = False) -> None:
    _check_pose(link, where, errors, warnings)
    for tag in ("collision", "visual"):
        for index, shape in enumerate(link.findall(tag)):
            if not list(shape):
                errors.append("{}.{}[{}]: nessuna geometria".format(
                    where, tag, index))
    if static:
        # A <static>true</static> model is not integrated by the physics
        # engine: SDFormat ignores <inertial> and <collision> for it, so
        # their absence is the correct description of a visual-only body
        # (e.g. the gz-waves water surface), not a hygiene defect.
        return
    if not link.findall("collision"):
        warnings.append("{}: nessun <collision> (il link non collide)".format(
            where))

    inertial = link.find("inertial")
    if inertial is None:
        warnings.append("{}: <inertial> mancante (SDFormat applica massa=1 e "
                        "inerzia nulla: la fisica non e' quella del CAD)"
                        .format(where))
        return
    mass = _text(inertial.find("mass"))
    if not mass:
        warnings.append("{}.inertial.mass: mancante (default 1 kg)".format(
            where))
    else:
        try:
            if float(mass) <= 0.0:
                warnings.append("{}.inertial.mass={}: deve essere > 0".format(
                    where, mass))
        except ValueError:
            errors.append("{}.inertial.mass={!r}: non numerico".format(
                where, mass))
    inertia = inertial.find("inertia")
    if inertia is None:
        warnings.append("{}.inertial.inertia: mancante (default: matrice "
                        "nulla)".format(where))
        return
    for name in INERTIA_ELEMENTS:
        raw = _text(inertia.find(name))
        if not raw:
            warnings.append("{}.inertial.inertia.{}: mancante (default 0)"
                            .format(where, name))
            continue
        try:
            float(raw)
        except ValueError:
            errors.append("{}.inertial.inertia.{}={!r}: non numerico".format(
                where, name, raw))


def _check_joint(joint: ET.Element, where: str, known: set, nested: set,
                 errors: List[str], warnings: List[str]) -> Optional[Tuple]:
    jname = joint.get("name")
    if not jname:
        errors.append("{}.0: <joint> senza attributo name".format(where))
    jtype = joint.get("type")
    if not jtype:
        errors.append("{}.{}: <joint> senza attributo type".format(
            where, jname))
    elif jtype not in JOINT_TYPES:
        errors.append("{}.{}: type={!r} non e' un tipo SDFormat valido "
                      "({})".format(where, jname, jtype,
                                    ", ".join(JOINT_TYPES)))

    # <parent>/<child> carry the name as element TEXT (sdformat.org/spec?elem=joint)
    parent, child = joint.find("parent"), joint.find("child")
    pname, cname = _text(parent), _text(child)
    if parent is None or not pname:
        errors.append("{}.{}: <parent> mancante o vuoto".format(where, jname))
    if child is None or not cname:
        errors.append("{}.{}: <child> mancante o vuoto".format(where, jname))

    _check_pose(joint, where, errors, warnings)

    for role, ref in (("parent", pname), ("child", cname)):
        if ref and not _resolve(ref, known, nested):
            errors.append("{}.{}.{}: '{}' non e' un link/frame del model"
                          .format(where, jname, role, ref))
    if pname and pname == cname:
        warnings.append("{}.{}: parent e child sono lo stesso link '{}'"
                        .format(where, jname, pname))

    if jtype in AXIS_TYPES:
        axis = joint.find("axis")
        if axis is None:
            warnings.append("{}.{}: <axis> mancante (default 0 0 1)".format(
                where, jname))
        else:
            xyz = _numbers(_text(axis.find("xyz")),
                           "{}.{}.axis.xyz".format(where, jname), (3,), errors)
            if len(xyz) == 3:
                norm = math.sqrt(sum(v * v for v in xyz))
                if norm <= AXIS_TOL:
                    warnings.append("{}.{}.axis.xyz: asse nullo".format(
                        where, jname))
                elif abs(norm - 1.0) > AXIS_TOL:
                    warnings.append("{}.{}.axis.xyz: non normalizzato (norma "
                                    "{:.6f})".format(where, jname, norm))
        limit = joint.find("axis/limit")
        if limit is not None:
            lower, upper = _text(limit.find("lower")), _text(limit.find("upper"))
            if lower and upper:
                try:
                    if float(lower) > float(upper):
                        errors.append("{}.{}.axis.limit: lower={} > upper={}"
                                      .format(where, jname, lower, upper))
                except ValueError:
                    errors.append("{}.{}.axis.limit: lower/upper non numerici"
                                  .format(where, jname))
        elif jtype in ("revolute", "prismatic"):
            warnings.append("{}.{}: <axis><limit> mancante per un joint {}"
                            .format(where, jname, jtype))

    if pname in known and cname in known and pname != cname:
        return (pname, cname)
    return None


def _check_connectivity(known: set, edges: List[Tuple], where: str,
                        warnings: List[str]) -> None:
    """Every link should hang off the main chain (single connected tree)."""
    if len(known) < 2:
        return
    adjacency: Dict[str, set] = {name: set() for name in known}
    for parent, child in edges:
        adjacency[parent].add(child)
        adjacency[child].add(parent)
    seen = set()
    stack = [sorted(known)[0]]
    while stack:
        node = stack.pop()
        if node in seen:
            continue
        seen.add(node)
        stack.extend(adjacency[node] - seen)
    orphans = sorted(known - seen)
    if orphans:
        warnings.append("{}: link non collegati alla catena principale "
                        "(joint mancante?): {}".format(where,
                                                       ", ".join(orphans)))


def _include_names(model: ET.Element) -> set:
    """Names under which this model pulls in other models (`<include>`).

    A composed model has no local links of its own and joints may reference
    ``included_link::link``, so includes count as declared children.
    """
    names = set()
    for include in model.findall("include"):
        name = _text(include.find("name"))
        if not name:
            uri = _text(include.find("uri"))
            if uri:
                name = uri.rstrip("/").split("/")[-1].replace("model://", "")
        if name:
            names.add(name)
    return names


def _is_static(model: ET.Element) -> bool:
    """True when ``<static>`` is set on the model (SDFormat: no integration).

    ``static`` is inherited by nested models, so the caller passes the
    parent's verdict down.
    """
    raw = _text(model.find("static")).strip().lower()
    return raw in ("true", "1")


def _check_model(model: ET.Element, where: str, errors: List[str],
                 warnings: List[str], static_parent: bool = False) -> None:
    if not model.get("name"):
        errors.append("{}: <model> senza attributo name".format(where))
    static = static_parent or _is_static(model)
    links = model.findall("link")
    frames = model.findall("frame")
    submodels = model.findall("model")
    includes = model.findall("include")
    for index, include in enumerate(includes):
        uri = _text(include.find("uri"))
        if not uri:
            errors.append(
                "{}.include[{}]: <uri> mancante (SDFormat lo richiede)".format(
                    where, index))
        _check_pose(include, "{}.include[{}]".format(where, index), errors,
                    warnings)
    if not links and not submodels and not includes:
        errors.append("{}: nessun <link> (ne' <include>/<model> annidati)"
                      .format(where))

    link_names = [link.get("name") for link in links]
    for name in sorted({n for n in link_names if n}):
        if link_names.count(name) > 1:
            errors.append("{}.link '{}': nome duplicato".format(where, name))
    for index, frame in enumerate(frames):
        if not frame.get("name"):
            errors.append("{}.frame[{}]: senza attributo name".format(
                where, index))
        _check_pose(frame, "{}.frame[{}]".format(where, index), errors,
                    warnings)
    nested = {m.get("name") for m in submodels if m.get("name")}
    nested |= _include_names(model)
    known = {n for n in link_names if n} | {f.get("name") for f in frames
                                            if f.get("name")} | nested

    for index, link in enumerate(links):
        lwhere = "{}.link[{}]".format(where, index)
        if not link.get("name"):
            errors.append("{}.0: <link> senza attributo name".format(lwhere))
        _check_link(link, lwhere, errors, warnings, static=static)

    joints = model.findall("joint")
    joint_names = [joint.get("name") for joint in joints]
    for name in sorted({n for n in joint_names if n}):
        if joint_names.count(name) > 1:
            errors.append("{}.joint '{}': nome duplicato".format(where, name))

    edges = []
    for index, joint in enumerate(joints):
        edge = _check_joint(joint, "{}.joint[{}]".format(where, index), known,
                            nested, errors, warnings)
        if edge is not None:
            edges.append(edge)
    _check_connectivity({n for n in link_names if n}, edges, where, warnings)

    for index, plugin in enumerate(model.findall("plugin")):
        if not plugin.get("filename") or not plugin.get("name"):
            errors.append("{}.plugin[{}]: servono gli attributi filename e "
                          "name".format(where, index))


def _check_root(root: ET.Element, errors: List[str],
                warnings: List[str]) -> None:
    if root.tag != "sdf":
        errors.append("<root>: atteso <sdf>, trovato <{}>".format(root.tag))
        return
    if not root.get("version"):
        errors.append("<sdf.version>: mancante (SDFormat lo richiede)")
    models = root.findall("model")
    worlds = root.findall("world")
    if not models and not worlds:
        errors.append("<sdf>: nessun <model> ne' <world>")
    for index, model in enumerate(models):
        name = model.get("name")
        _check_model(model, "model[{}]{}".format(
            index, " '{}'".format(name) if name else ""), errors, warnings,
            static_parent=False)
    for index, world in enumerate(worlds):
        if not world.get("name"):
            errors.append("world[{}]: senza attributo name".format(index))
        for mindex, model in enumerate(world.findall("model")):
            name = model.get("name")
            _check_model(model, "world[{}].model[{}]{}".format(
                index, mindex, " '{}'".format(name) if name else ""),
                errors, warnings)


def collect_findings(path: str) -> Tuple[List[str], List[str]]:
    """Parse ``path`` with the embedded backend -> ``(errors, warnings)``."""
    if not os.path.exists(path):
        return ["file non trovato: {}".format(path)], []
    try:
        tree = ET.parse(path)
    except ET.ParseError as exc:
        return ["XML non ben formato: {}".format(exc)], []
    errors: List[str] = []
    warnings: List[str] = []
    _check_root(tree.getroot(), errors, warnings)
    return errors, warnings


def collect_errors(path: str) -> List[str]:
    """SDFormat conformance errors only (the embedded backend)."""
    return collect_findings(path)[0]


def collect_warnings(path: str) -> List[str]:
    """Physical-hygiene warnings only (the embedded backend)."""
    return collect_findings(path)[1]


# --------------------------------------------------------------------------- #
# public entry points
# --------------------------------------------------------------------------- #

def run_gz(path: str) -> List[str]:
    """Run ``gz sdf -p``; return the list of errors (empty on success)."""
    command = gz_command()
    if command is None:
        return ["CLI 'gz'/'ign' non trovata su PATH"]
    proc = subprocess.run(list(command) + [path], capture_output=True,
                          text=True)
    if proc.returncode == 0:
        return []
    detail = (proc.stderr or proc.stdout or "").strip()
    return ["gz sdf -p ha rifiutato {} (exit {}): {}".format(
        path, proc.returncode, detail.replace("\n", " | ")[:800])]


def resolve_backend(backend: str) -> str:
    """Pick the backend: ``auto`` -> gz when installed, else embedded."""
    if backend not in BACKENDS:
        raise ValueError("backend sconosciuto: {!r}".format(backend))
    if backend == "auto":
        return "gz" if gz_available() else "builtin"
    return backend


def check(path: str, backend: str = "auto") -> Tuple[List[str], List[str], str]:
    """Return ``(errors, warnings, backend_that_ran)`` for one SDF file."""
    chosen = resolve_backend(backend)
    if chosen == "gz":
        errors = run_gz(path)
        return errors, [], "gz"
    errors, warnings = collect_findings(path)
    return errors, warnings, "builtin"


def validate(path: str, backend: str = "auto", strict: bool = False) -> str:
    """Validate one SDF file; raise :class:`SdfError` on any finding.

    Returns the backend that actually ran (``"gz"`` or ``"builtin"``).
    """
    errors, warnings, ran = check(path, backend=backend)
    if ran == "gz" and errors and not gz_available():
        raise SdfError(["backend gz richiesto ma la CLI non e' su PATH"],
                       source=path, backend="gz:missing")
    failed = errors or (strict and warnings)
    if failed:
        raise SdfError(errors or warnings, source=path, backend=ran,
                       warnings=warnings if errors else [])
    return ran


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Valida un file SDF con `gz sdf -p` (se installato) "
                    "oppure con il parser SDFormat incluso.")
    parser.add_argument("files", nargs="+", help="file SDF da validare")
    parser.add_argument("--backend", default="auto", choices=list(BACKENDS),
                        help="auto (default): gz se presente, altrimenti "
                             "builtin")
    parser.add_argument("--strict", action="store_true",
                        help="tratta anche i warning come errori")
    parser.add_argument("--quiet", action="store_true",
                        help="stampa solo backend ed esito")
    args = parser.parse_args(argv)

    failures = 0
    for path in args.files:
        errors, warnings, ran = check(path, backend=args.backend)
        if not args.quiet:
            for warning in warnings:
                print("sdf-validate: warning: {}".format(warning))
        if errors or (args.strict and warnings):
            failures += 1
            print(SdfError(errors or warnings, source=path, backend=ran,
                           warnings=warnings if errors else []),
                  file=sys.stderr)
            continue
        if args.quiet:
            print("{}: ok ({})".format(path, ran))
        else:
            print("sdf-validate: OK {} [backend {}]".format(path, ran))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
