"""urdf_skeleton.py -- the kinematic URDF skeleton derived from the CAD IR.

This is block **C6** of the CAD -> simulation chain: it turns ``cad_manifest.json``
(the IR, ``schema: cad-to-gazebo/cad-ir/1``) into the **minimal URDF** ROS 2
needs for the kinematic tree (``robot_state_publisher`` / TF / RViz).

It is deliberately **not** a second robot description and **not** a twin of the
SDF emitter. The rich model that carries geometry is the SDF; the official
SDF->URDF bridge (``sdformat_urdf``) *rejects* a model that uses non-URDF joint
types, nested models, a model pose, sensors or lights inside a link -- and ours
does. So the ROS-side artefact is emitted from the IR, not converted from the
SDF, and it carries the **skeleton and nothing else**:

* one ``<link>`` per IR body, with **no** ``<inertial>``, ``<visual>``,
  ``<collision>`` or ``<material>`` child;
* one ``<joint>`` per IR joint, with its type (restricted to
  ``fixed`` / ``revolute`` / ``continuous`` / ``prismatic``), its ``parent`` /
  ``child`` links, its ``<origin>`` and its ``<axis>``;
* nothing else: no ``<gazebo>``, no plugin, no ``<transmission>``.

Materials, PBR, collision geometry and inertia belong to the SDF emitter of
``maritime_ws`` (``build_simulation``); physical control of the PTO joint is
``gz_ros2_control`` reading the SDF plus ``ros_gz_bridge`` -- not this file.
``xacro`` is out too: the IR already parameterises, and a template layer would
be a third representation of the same robot.

The document is a *closed* contract: every mapping decision (joint-type
collapse, bound rule, axis normalisation, quaternion -> RPY, the effort/velocity
placeholder) is pinned in ``docs/urdf-skeleton.md`` and implemented once, here.
Nothing in the emitted file is a TODO or a question: an open point belongs in
the PR / issue, not in a file the ROS stack parses.

Design decisions worth knowing before reading the code
-----------------------------------------------------

* **The tree is never invented.** A joint that is not in the URDF vocabulary is
  *mapped* by the documented table :data:`FC_TYPE_TO_URDF` (or by an explicit
  ``urdf_type`` on the joint), and a joint type outside both is a hard error: no
  DOF is guessed, because a bolt and a slider have the same faces. A body the IR
  did not connect to the root stays a second tree and is reported as a warning
  -- the emitter does not bolt it to the root on its own.
* **A deactivated joint becomes ``fixed``.** ``activated: false`` means "the CAD
  joint carries no DOF"; dropping it would leave the child body floating, so it
  is emitted rigid. The name and the parent/child edge are preserved.
* **Rotation is converted deterministically.** IR poses are
  ``[x, y, z, qx, qy, qz, qw]``; URDF wants ``<origin xyz=... rpy=...>`` with
  the fixed-axis XYZ convention. The quaternion is normalised, the gimbal-lock
  case is canonicalised (``roll = 0``), and all floats are rounded to
  :data:`_ROUND` decimals so two runs give identical bytes.
* **Axes are normalised.** URDF honours an ``<axis>`` direction, not its length;
  the emitted axis is the unit vector of the IR axis.
* **``effort`` / ``velocity`` are placeholders.** URDF *requires* them on a
  bounded joint and the IR carries no actuator data (it is a CAD contract, not a
  drive-train one). They come from the joint when the model author declares them
  (``effort`` / ``velocity``) and otherwise from the documented constants
  :data:`DEFAULT_EFFORT` / :data:`DEFAULT_VELOCITY`; the run records it as a
  warning so the placeholder is never silent.
* **The output is byte-idempotent.** No timestamp, no absolute path, no
  dict-order dependence, all floats rounded, ``\\n`` newline: regenerating and
  diffing is the gate (see the CI workflow).

CLI
---

::

    python -m cad.urdf_skeleton --manifest tests/fixtures/cad_manifest_urdf_skeleton.json \\
        --out tests/fixtures/urdf_skeleton_sample.urdf --strict
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from typing import Any, Dict, List, Optional, Sequence, Tuple

if __package__ in (None, ""):  # direct execution: make ``cad`` importable
    _ROOT = os.path.dirname(os.path.abspath(__file__))
    _SRC = os.path.dirname(_ROOT)
    if _SRC not in sys.path:
        sys.path.insert(0, _SRC)

__all__ = [
    "SKELETON_SCHEMA",
    "EMITTER_VERSION",
    "URDF_JOINT_TYPES",
    "FC_TYPE_TO_URDF",
    "DEFAULT_EFFORT",
    "DEFAULT_VELOCITY",
    "LEAN_TAGS",
    "UrdfSkeletonError",
    "urdf_joint_type",
    "quat_to_rpy",
    "normalize_quat",
    "normalize_axis",
    "skeleton_links",
    "skeleton_joints",
    "generate_urdf",
    "collect_errors",
    "validate_urdf",
    "parse_skeleton",
    "build_parser",
    "main",
]

#: Version tag of the kinematic skeleton contract (bumped per breaking change).
SKELETON_SCHEMA = "cad-to-gazebo/urdf-skeleton/1"

#: ``<package>/<version>`` of this emitter, mirrored into the header comment.
EMITTER_VERSION = "cad-to-gazebo/0.1.0"

#: The only joint types a URDF may carry, in the order of the specification.
URDF_JOINT_TYPES: Tuple[str, ...] = ("fixed", "revolute", "continuous", "prismatic")

#: FreeCAD ``Assembly::Joint`` type (lower-cased) -> URDF joint type.
#:
#: This table is the contract for everything URDF cannot express natively. Each
#: row keeps the *primary* degree of freedom of the CAD joint and drops only what
#: a single URDF joint has no way to carry; the dropped component is documented
#: per row in ``docs/urdf-skeleton.md`` (section 3). A joint whose type is not in
#: this table must declare ``urdf_type`` explicitly, otherwise the run fails.
FC_TYPE_TO_URDF: Tuple[Tuple[str, str], ...] = (
    ("fixed", "fixed"),
    ("revolute", "revolute"),
    ("continuous", "continuous"),
    ("slider", "prismatic"),
    ("prismatic", "prismatic"),
    # cylindrical = prismatic + co-axial revolute: URDF has no such joint, and
    # the linear DOF is the one an actuator drives on a piston.
    ("cylindrical", "prismatic"),
    # ball / spherical = three rotations: URDF keeps one.
    ("ball", "continuous"),
    ("spherical", "continuous"),
    # distance = a constraint, not a free coordinate: no DOF is added.
    ("distance", "fixed"),
    ("distance2", "fixed"),
    # screw = prismatic + coupled rotation: the lead is the primary motion.
    ("screw", "prismatic"),
    # rack & pinion / gear / belt: a rotation drives a translation; the joint
    # itself is a revolute in the URDF tree.
    ("rackpinion", "revolute"),
    ("gear", "revolute"),
    ("belt", "revolute"),
)

#: ``effort`` placeholder for a bounded joint when the IR declares none (N / N·m).
DEFAULT_EFFORT = 1000.0

#: ``velocity`` placeholder for a bounded joint when the IR declares none (m/s / rad/s).
DEFAULT_VELOCITY = 100.0

#: Elements that make a URDF more than a kinematic skeleton. Their absence is an
#: acceptance criterion of C6, asserted by :func:`collect_errors` and in tests.
LEAN_TAGS = ("inertial", "visual", "collision", "material", "gazebo", "plugin",
             "transmission", "sensor")

#: Identity pose [x, y, z, qx, qy, qz, qw].
_IDENTITY7 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

#: Decimals kept in the emitted floats. 9 is ~nanometre / ~nanoradian resolution:
#: far below the ROS-side tolerance and enough to make the text stable.
_ROUND = 9


class UrdfSkeletonError(RuntimeError):
    """Raised when the IR cannot be turned into a valid kinematic skeleton."""


# --------------------------------------------------------------------------- #
# small numeric helpers
# --------------------------------------------------------------------------- #


def _num(value: Any) -> str:
    """Canonical decimal text of a scalar (deterministic serialisation)."""
    number = round(float(value), _ROUND)
    if number == 0.0:  # normalise -0.0
        return "0"
    text = "{:.9f}".format(number).rstrip("0").rstrip(".")
    return text or "0"


def _vec(values: Sequence[Any]) -> str:
    return " ".join(_num(v) for v in values)


def normalize_quat(quat: Sequence[Any]) -> Tuple[float, float, float, float]:
    """Return the unit quaternion ``(x, y, z, w)`` of ``quat``.

    Raises for a null quaternion: a zero-length rotation is not a rotation, and
    silently defaulting it to identity would move the joint.
    """
    values = [float(v) for v in quat]
    if len(values) != 4:
        raise UrdfSkeletonError(
            "quaternione deve avere 4 valori [qx,qy,qz,qw]: {!r}".format(quat))
    norm = math.sqrt(sum(v * v for v in values))
    if norm <= 1e-15:
        raise UrdfSkeletonError("quaternione nullo nella pose: {!r}".format(quat))
    return tuple(v / norm for v in values)  # type: ignore[return-value]


def quat_to_rpy(quat: Sequence[Any]) -> Tuple[float, float, float]:
    """Convert a quaternion to URDF roll/pitch/yaw (fixed-axis XYZ).

    The returned angles reconstruct the same rotation under
    ``R = Rz(yaw) Ry(pitch) Rx(roll)``, which is the URDF/tf convention. At the
    gimbal-lock singularity (``|pitch| = 90 deg``) the decomposition is not
    unique, so it is canonicalised with ``roll = 0`` and the remaining rotation
    folded into ``yaw``: the same quaternion always gives the same triple.
    """
    x, y, z, w = normalize_quat(quat)
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    if sin_pitch >= 1.0 - 1e-12:
        pitch = math.pi / 2.0
    elif sin_pitch <= -1.0 + 1e-12:
        pitch = -math.pi / 2.0
    else:
        pitch = math.asin(sin_pitch)

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)

    if abs(abs(pitch) - math.pi / 2.0) < 1e-12:
        # Gimbal lock: roll and yaw are only defined through their sum/difference,
        # so pin roll = 0 and read yaw off the matrix rows that survive the
        # singularity: R[0][1] = -sin(yaw), R[0][2] = ±cos(yaw).
        roll = 0.0
        sin_yaw_lock = -2.0 * (x * y - z * w)
        cos_yaw_lock = 2.0 * (x * z + y * w)
        if sin_pitch < 0.0:
            cos_yaw_lock = -cos_yaw_lock
        yaw = math.atan2(sin_yaw_lock, cos_yaw_lock)
    return roll, pitch, yaw


def normalize_axis(axis: Optional[Sequence[Any]]) -> List[float]:
    """Return the unit vector of an IR axis (identity ``x`` when absent)."""
    if axis is None:
        return [1.0, 0.0, 0.0]
    values = [float(v) for v in axis]
    if len(values) != 3:
        raise UrdfSkeletonError("asse deve avere 3 valori [x,y,z]: {!r}".format(axis))
    norm = math.sqrt(sum(v * v for v in values))
    if norm <= 1e-12:
        raise UrdfSkeletonError("asse nullo: {!r}".format(axis))
    out = [v / norm for v in values]
    return [0.0 if abs(v) < 1e-15 else v for v in out]


# --------------------------------------------------------------------------- #
# joint type: the one mapping
# --------------------------------------------------------------------------- #


def _bounds_enabled(joint: Dict[str, Any]) -> Tuple[bool, bool]:
    limits = joint.get("limits") or {}
    lower = bool(limits.get("lower_enabled"))
    upper = bool(limits.get("upper_enabled"))
    return lower, upper


def urdf_joint_type(joint: Dict[str, Any]) -> str:
    """Map one IR joint to its URDF type.

    Order of decision (see ``docs/urdf-skeleton.md`` §3):

    1. ``activated: false`` -> ``fixed``: the CAD joint carries no DOF, but the
       child body must stay in the tree, so the edge is emitted rigid;
    2. an explicit ``urdf_type`` on the joint wins, when it is one of the four
       URDF types -- the documented escape hatch for a CAD joint whose primary
       DOF the model author wants to choose;
    3. a ``revolute`` joint whose two limits are *disabled* -> ``continuous``:
       that is exactly "unbounded rotation", which is what the IR states;
    4. otherwise the ``fc_type`` row of :data:`FC_TYPE_TO_URDF`;
    5. nothing matched -> :class:`UrdfSkeletonError`. A DOF is never guessed.
    """
    if not joint.get("activated", True):
        return "fixed"
    declared = joint.get("urdf_type")
    if declared is not None:
        text = str(declared).strip().lower()
        if text not in URDF_JOINT_TYPES:
            raise UrdfSkeletonError(
                "joint '{}': urdf_type '{}' non e' un tipo URDF ({}).".format(
                    joint.get("name", "?"), declared, ", ".join(URDF_JOINT_TYPES)))
        return text
    fc_type = str(joint.get("fc_type") or "").strip().lower()
    mapped = dict(FC_TYPE_TO_URDF).get(fc_type)
    if mapped is None:
        raise UrdfSkeletonError(
            "joint '{}': fc_type '{}' non ha un'immagine URDF; dichiara "
            "'urdf_type' tra {}.".format(
                joint.get("name", "?"), joint.get("fc_type"), ", ".join(URDF_JOINT_TYPES)))
    if mapped == "revolute":
        lower, upper = _bounds_enabled(joint)
        if not lower and not upper:
            return "continuous"
    return mapped


# --------------------------------------------------------------------------- #
# the kinematic model: links + joints
# --------------------------------------------------------------------------- #


def skeleton_links(ir: Dict[str, Any]) -> List[str]:
    """The link names of the skeleton, in IR body order."""
    return [body.get("name", "") for body in ir.get("bodies") or []]


def _joint_origin(joint: Dict[str, Any]) -> Tuple[List[float], List[float]]:
    pose = joint.get("origin")
    if pose is None:
        pose = _IDENTITY7
    values = [float(v) for v in pose]
    if len(values) != 7:
        raise UrdfSkeletonError(
            "joint '{}': origin deve avere 7 valori [x,y,z,qx,qy,qz,qw]: {!r}".format(
                joint.get("name", "?"), pose))
    xyz = [0.0 if abs(v) < 1e-15 else v for v in values[:3]]
    rpy = [0.0 if abs(v) < 1e-15 else v for v in quat_to_rpy(values[3:])]
    return xyz, rpy


def _joint_limits(joint: Dict[str, Any], urdf_type: str,
                  warnings: List[str]) -> Optional[Dict[str, str]]:
    """Return the ``<limit>`` attributes of a movable joint, or ``None``.

    ``fixed`` joints carry no limit. ``continuous`` joints carry only
    ``effort`` / ``velocity`` (URDF rejects bounds on an unbounded joint). The
    bounded types carry the IR's numeric ``lower`` / ``upper`` verbatim, in the
    unit the IR already states (``m`` for prismatic, ``rad`` for revolute).
    """
    if urdf_type == "fixed":
        return None
    limits = joint.get("limits") or {}
    effort = joint.get("effort", joint.get("max_effort"))
    velocity = joint.get("velocity", joint.get("max_velocity"))
    if effort is None:
        effort = DEFAULT_EFFORT
        warnings.append(
            "joint '{}': effort assente nel IR, placeholder {}".format(
                joint.get("name", "?"), _num(DEFAULT_EFFORT)))
    if velocity is None:
        velocity = DEFAULT_VELOCITY
        warnings.append(
            "joint '{}': velocity assente nel IR, placeholder {}".format(
                joint.get("name", "?"), _num(DEFAULT_VELOCITY)))
    out = {"effort": _num(effort), "velocity": _num(velocity)}
    if urdf_type == "continuous":
        return out
    lower = limits.get("lower", 0.0)
    upper = limits.get("upper", 0.0)
    if float(lower) > float(upper):
        raise UrdfSkeletonError(
            "joint '{}': lower={} > upper={}".format(joint.get("name", "?"), lower, upper))
    out["lower"] = _num(lower)
    out["upper"] = _num(upper)
    return out


def skeleton_joints(ir: Dict[str, Any]) -> List[Dict[str, Any]]:
    """The joint records of the skeleton, in IR joint order.

    Each record is ``{name, type, parent, child, origin: [7], axis: [3]|None,
    limit: {…}|None}`` -- the values the emitter writes, with the IR pose kept
    as-is next to the URDF view so the equivalence test can compare like with
    like.
    """
    joints: List[Dict[str, Any]] = []
    for joint in ir.get("joints") or []:
        urdf_type = urdf_joint_type(joint)
        xyz, rpy = _joint_origin(joint)
        record: Dict[str, Any] = {
            "name": joint.get("name", ""),
            "type": urdf_type,
            "parent": joint.get("parent", ""),
            "child": joint.get("child", ""),
            "origin": xyz + rpy,
            "origin_pose": [float(v) for v in (joint.get("origin") or _IDENTITY7)],
            "axis": None if urdf_type == "fixed" else normalize_axis(joint.get("axis")),
        }
        joints.append(record)
    return joints


def _collect_warnings(ir: Dict[str, Any], joints: List[Dict[str, Any]]) -> List[str]:
    """Structural warnings: the tree is described, never repaired."""
    warnings: List[str] = []
    links = skeleton_links(ir)
    known = set(links)
    children = {joint["child"] for joint in joints}
    roots = [name for name in links if name not in children]
    for joint in joints:
        for side in ("parent", "child"):
            if joint[side] not in known:
                raise UrdfSkeletonError(
                    "joint '{}': {} '{}' non e' un body del IR".format(
                        joint["name"], side, joint[side]))
    if not roots:
        raise UrdfSkeletonError(
            "nessun link radice: ogni link e' figlio di un joint (ciclo)")
    declared_root = (ir.get("meta") or {}).get("root")
    if declared_root and declared_root in known and declared_root not in roots:
        warnings.append(
            "meta.root '{}' non e' una radice del grafo (radici: {})".format(
                declared_root, ", ".join(roots)))
    if len(roots) > 1:
        warnings.append(
            "il grafo ha {} radici ({}): il IR non collega quei body. "
            "robot_state_publisher richiede un albero connesso.".format(
                len(roots), ", ".join(roots)))
    seen: Dict[str, str] = {}
    for joint in joints:
        if joint["child"] in seen:
            warnings.append(
                "link '{}' e' figlio di piu' joint ('{}' e '{}'): non e' un albero".format(
                    joint["child"], seen[joint["child"]], joint["name"]))
        seen[joint["child"]] = joint["name"]
    return warnings


# --------------------------------------------------------------------------- #
# rendering: deterministic XML
# --------------------------------------------------------------------------- #


def _robot_name(ir: Dict[str, Any]) -> str:
    """``<robot name>``: the IR root, else the first body, else ``robot``."""
    meta = ir.get("meta") or {}
    root = meta.get("root")
    if isinstance(root, str) and root:
        return root
    links = skeleton_links(ir)
    return links[0] if links else "robot"


def _comment(ir: Dict[str, Any]) -> str:
    """One deterministic provenance comment (no timestamp, no absolute path)."""
    meta = ir.get("meta") or {}
    root = meta.get("root")
    return ("<!-- cad-to-gazebo kinematic skeleton emitter {} | ir={} | root={} -->"
            .format(EMITTER_VERSION, ir.get("schema", ""), root if root else ""))


def render(ir: Dict[str, Any], warnings: Optional[List[str]] = None) -> str:
    """Render the kinematic skeleton of ``ir`` as URDF text (no validation)."""
    warn = warnings if warnings is not None else []
    joints = skeleton_joints(ir)
    lines = ['<?xml version="1.0"?>', _comment(ir),
             '<robot name="{}">'.format(_robot_name(ir))]
    for name in skeleton_links(ir):
        lines.append('  <link name="{}"/>'.format(name))
    for joint in joints:
        limit = _joint_limits(dict(
            _joint_of(ir, joint["name"]), name=joint["name"]), joint["type"], warn)
        lines.append('  <joint name="{}" type="{}">'.format(joint["name"], joint["type"]))
        lines.append('    <parent link="{}"/>'.format(joint["parent"]))
        lines.append('    <child link="{}"/>'.format(joint["child"]))
        lines.append('    <origin xyz="{}" rpy="{}"/>'.format(
            _vec(joint["origin"][:3]), _vec(joint["origin"][3:])))
        if joint["axis"] is not None:
            lines.append('    <axis xyz="{}"/>'.format(_vec(joint["axis"])))
        if limit is not None:
            attrs = " ".join('{}="{}"'.format(k, limit[k])
                             for k in ("effort", "velocity", "lower", "upper") if k in limit)
            lines.append('    <limit {}/>'.format(attrs))
        lines.append("  </joint>")
    lines.append("</robot>")
    return "\n".join(lines) + "\n"


def _joint_of(ir: Dict[str, Any], name: str) -> Dict[str, Any]:
    for joint in ir.get("joints") or []:
        if joint.get("name") == name:
            return joint
    raise UrdfSkeletonError("joint '{}' non trovato nel IR".format(name))


def generate_urdf(ir: Dict[str, Any]) -> str:
    """Validate ``ir`` and return the URDF text of its kinematic skeleton.

    This is the public entry point. It refuses an IR that violates the v1
    contract (the same fail-fast gate as every other emitter) and refuses a
    URDF that would not load, so a caller never gets an artefact the ROS stack
    would reject later.
    """
    from cad import ir_schema

    errors = ir_schema.collect_errors(ir)
    if errors:
        raise UrdfSkeletonError("IR non valido:\n  - " + "\n  - ".join(errors))
    warnings: List[str] = []
    joints = skeleton_joints(ir)  # fail-fast: joint non mappabile, asse nullo
    warnings.extend(_collect_warnings(ir, joints))
    text = render(ir, warnings)
    problems = collect_errors(text)
    if problems:
        raise UrdfSkeletonError(
            "URDF generato non valido:\n  - " + "\n  - ".join(problems))
    return text


# --------------------------------------------------------------------------- #
# the gate: a dependency-free URDF validator
# --------------------------------------------------------------------------- #


def _tag_name(element: Any) -> str:
    return element.tag.rsplit("}", 1)[-1]


def parse_skeleton(text: str) -> Dict[str, Any]:
    """Parse URDF ``text`` into ``{name, links, joints}`` (for consumers/tests)."""
    root = ET.fromstring(text)
    if _tag_name(root) != "robot":
        raise UrdfSkeletonError("radice XML '{}', attesa 'robot'".format(_tag_name(root)))
    links = [_attr(child, "name") for child in root if _tag_name(child) == "link"]
    joints: List[Dict[str, Any]] = []
    for child in root:
        if _tag_name(child) != "joint":
            continue
        record: Dict[str, Any] = {
            "name": _attr(child, "name"),
            "type": _attr(child, "type"),
            "parent": None,
            "child": None,
            "origin": None,
            "axis": None,
            "limit": None,
        }
        for elem in child:
            tag = _tag_name(elem)
            if tag == "parent":
                record["parent"] = _attr_or_none(elem, "link")
            elif tag == "child":
                record["child"] = _attr_or_none(elem, "link")
            elif tag == "origin":
                record["origin"] = {k: _attr_or_none(elem, k) for k in ("xyz", "rpy")}
            elif tag == "axis":
                record["axis"] = _attr_or_none(elem, "xyz")
            elif tag == "limit":
                record["limit"] = dict(elem.attrib)
        joints.append(record)
    return {"name": _attr(root, "name"), "links": links, "joints": joints}


def _attr(element: Any, key: str) -> str:
    value = element.get(key)
    if value is None:
        raise UrdfSkeletonError("elemento '{}' senza attributo '{}'".format(
            _tag_name(element), key))
    return value


def _attr_or_none(element: Any, key: str) -> Optional[str]:
    return element.get(key)


def _numbers(value: Optional[str], count: int, where: str) -> List[float]:
    if value is None:
        raise UrdfSkeletonError("{}: valore mancante".format(where))
    parts = value.split()
    if len(parts) != count:
        raise UrdfSkeletonError(
            "{}: attesi {} numeri, trovati {!r}".format(where, count, value))
    out = []
    for part in parts:
        try:
            out.append(float(part))
        except ValueError:
            raise UrdfSkeletonError("{}: {!r} non e' un numero".format(where, part))
    return out


def collect_errors(text: str, *, require_single_root: bool = False) -> List[str]:
    """Return every reason ``text`` is not a loadable kinematic skeleton URDF.

    This is the headless half of the gate (``docs/urdf-skeleton.md`` §6). It is
    *not* a string comparison: the document is really parsed and its tree is
    really walked. It checks what ``check_urdf`` and ``robot_state_publisher``
    need -- one ``<robot name>``, unique link/joint names, a joint type from the
    URDF vocabulary, ``parent`` / ``child`` resolving to existing links, a
    ``<limit>`` on every bounded joint, an ``<axis>`` on every movable joint --
    and the C6 leanness criterion: **no** inertial / visual / collision /
    material / gazebo / plugin / transmission / sensor element anywhere.
    """
    errors: List[str] = []
    try:
        tree = ET.fromstring(text)
    except ET.ParseError as exc:
        return ["XML non valido: {}".format(exc)]
    if _tag_name(tree) != "robot":
        return ["radice XML '{}', attesa 'robot'".format(_tag_name(tree))]
    if not (tree.get("name") or "").strip():
        errors.append("robot: attributo 'name' mancante o vuoto")

    links = [child for child in tree if _tag_name(child) == "link"]
    link_names = [child.get("name") for child in links]
    for name in link_names:
        if not name:
            errors.append("link: attributo 'name' mancante")
    if len(set(link_names)) != len(link_names):
        errors.append("link: nome duplicato")
    if not links:
        errors.append("robot: nessun link")

    for element in tree.iter():
        if _tag_name(element) in LEAN_TAGS:
            errors.append(
                "elemento '{}' presente: il C6 emette il solo scheletro cinematico".format(
                    _tag_name(element)))

    known = set(link_names)
    joint_names: List[str] = []
    children = set()
    for element in tree:
        if _tag_name(element) != "joint":
            continue
        name = element.get("name")
        joint_names.append(name or "")
        where = "joint '{}'".format(name)
        if not name:
            errors.append("joint: attributo 'name' mancante")
        if name in joint_names[:-1]:
            errors.append("{}: nome duplicato".format(where))
        joint_type = element.get("type")
        if joint_type not in URDF_JOINT_TYPES:
            errors.append("{}: type '{}' non URDF".format(where, joint_type))
        parent = child = None
        for sub in element:
            tag = _tag_name(sub)
            if tag == "parent":
                parent = sub.get("link")
            elif tag == "child":
                child = sub.get("link")
        if parent not in known:
            errors.append("{}: parent '{}' non e' un link".format(where, parent))
        if child not in known:
            errors.append("{}: child '{}' non e' un link".format(where, child))
        if parent is not None and parent == child:
            errors.append("{}: parent e child coincidono".format(where))
        if child is not None:
            children.add(child)
        limit = next((sub for sub in element if _tag_name(sub) == "limit"), None)
        axis = next((sub for sub in element if _tag_name(sub) == "axis"), None)
        if joint_type in ("revolute", "prismatic"):
            if limit is None:
                errors.append("{}: type '{}' richiede <limit>".format(where, joint_type))
            else:
                for key in ("effort", "velocity"):
                    if limit.get(key) is None:
                        errors.append("{}: <limit> senza '{}'".format(where, key))
                low, high = limit.get("lower", "0"), limit.get("upper", "0")
                try:
                    if float(low) > float(high):
                        errors.append("{}: lower {} > upper {}".format(where, low, high))
                except ValueError:
                    errors.append("{}: limiti non numerici".format(where))
        if axis is not None:
            try:
                vector = _numbers(axis.get("xyz"), 3, "{} axis".format(where))
                if math.sqrt(sum(v * v for v in vector)) <= 1e-12:
                    errors.append("{}: axis nullo".format(where))
            except UrdfSkeletonError as exc:
                errors.append(str(exc))
        elif joint_type != "fixed":
            errors.append("{}: type '{}' richiede <axis>".format(where, joint_type))
        for sub in element:
            if _tag_name(sub) == "origin":
                for key in ("xyz", "rpy"):
                    try:
                        _numbers(sub.get(key), 3, "{} origin {}".format(where, key))
                    except UrdfSkeletonError as exc:
                        errors.append(str(exc))

    roots: List[str] = [n for n in link_names if n and n not in children]
    if not roots:
        errors.append("nessun link radice: grafo ciclico")
    if require_single_root and len(roots) > 1:
        errors.append("il grafo ha {} radici ({}): robot_state_publisher ne richiede una"
                      .format(len(roots), ", ".join(roots)))
    return errors


def validate_urdf(text: str, *, require_single_root: bool = False) -> None:
    """Raise :class:`UrdfSkeletonError` when ``text`` is not a valid skeleton."""
    errors = collect_errors(text, require_single_root=require_single_root)
    if errors:
        raise UrdfSkeletonError("URDF non valido:\n  - " + "\n  - ".join(errors))


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m cad.urdf_skeleton",
        description="Emette lo scheletro cinematico URDF (link + joint, niente "
                    "geometria) dal manifest IR.")
    parser.add_argument("--manifest", required=True, help="path del manifest IR (JSON)")
    parser.add_argument("--out", default=None,
                        help="file di uscita; '-' o assente = stdout")
    parser.add_argument("--strict", action="store_true",
                        help="l'albero deve avere una sola radice (robot_state_publisher)")
    return parser


def _warnings_for(ir: Dict[str, Any]) -> List[str]:
    warnings: List[str] = []
    joints = skeleton_joints(ir)
    warnings.extend(_collect_warnings(ir, joints))
    for joint in ir.get("joints") or []:
        joint_type = urdf_joint_type(joint)
        if joint_type != "fixed":
            _joint_limits(joint, joint_type, warnings)
    return warnings


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    with open(args.manifest, encoding="utf-8") as handle:
        ir = json.load(handle)
    try:
        text = generate_urdf(ir)
    except UrdfSkeletonError as exc:
        sys.stderr.write("{}\n".format(exc))
        return 1
    warnings = _warnings_for(ir)
    for warning in warnings:
        sys.stderr.write("[WARN] {}\n".format(warning))
    if args.strict:
        problems = collect_errors(text, require_single_root=True)
        if problems:
            sys.stderr.write("URDF non valido (--strict):\n  - {}\n".format(
                "\n  - ".join(problems)))
            return 1
    if args.out in (None, "-"):
        sys.stdout.write(text)
        return 0
    with open(args.out, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
