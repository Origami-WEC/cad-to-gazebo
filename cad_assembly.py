#!/usr/bin/env python3
"""
cad_assembly.py  --  Logica pura Assembly FreeCAD -> modello di simulazione

Nessuna dipendenza da FreeCAD: unit-testabile fuori dal CAD.

Responsabilita'
--------------
  1. mapping dei 13 JointType dell'Assembly FreeCAD 1.0 su joint SDF/Gazebo;
  2. merge degli incastri (Fixed): i corpi saldati diventano un unico link
     conservando densita' per solido e calcolando massa equivalente, CoM e
     tensore d'inerzia (teorema del baricentro + asse parallelo);
  3. matematica dei frame (pose link-local / globali, origin dei joint);
  4. albero cinematico: root dal GroundedJoint, parent/child dei joint.

Schema del manifest (contratto con freecad_exporter.py e build_simulation.py)
----------------------------------------------------------------------------
  meta.root                    nome del body grounded (radice dell'albero)
  bodies[]                     1 body = 1 App::Part dell'assembly
    .name .label .pose         pose globale del body [x,y,z,qx,qy,qz,qw]
    .solids[]                  solidi del body (1 mesh ciascuno, frame body)
      .name .density_kg_m3 .density_source
      .reference_mass_kg .reference_volume_m3 .reference_com_m
      .reference_inertia_kg_m2  (opzionale, attorno al CoM, frame body)
      .visual_mesh .collision_mesh .stl_path .stl_collision_path
      .material  {diffuse, ambient} RGBA
      .pose      frame del solido nel body [x,y,z,qx,qy,qz,qw]
  joints[]                     1 joint = 1 Assembly::JointObject
    .name .label .fc_type .sdf_type .mapped .merged
    .parent .child             nomi body (parent = Reference2, child = Reference1)
    .origin                    frame joint nel body child [x,y,z,qx,qy,qz,qw]
    .axis                      asse nel frame joint (asse Z del connector)
    .limits                    {lower, upper, lower_enabled, upper_enabled,
                                 unit, offset_m}
                             lower/upper = valori ASSOLUTI FreeCAD (distanza
                             JCS da coincidenza). offset_m = coordinata
                             corrente del joint (assieme montato). La
                             conversione in spostamenti SDF avviene con
                             freecad_limits_to_sdf().
    .activated
"""

from dataclasses import dataclass, field
import math

import numpy as np

MM_TO_M = 1.0e-3
DEG_TO_RAD = math.pi / 180.0

FC_JOINT_TYPES = (
    "Fixed", "Revolute", "Cylindrical", "Slider", "Ball",
    "Distance", "Parallel", "Perpendicular", "Angle",
    "RackPinion", "Screw", "Gears", "Belt",
)

JOINT_SLIDER = "Slider"
JOINT_FIXED = "Fixed"

LIMIT_UNIT_M = "m"
LIMIT_UNIT_RAD = "rad"
LIMIT_UNIT_M_PER_REV = "m_per_rev"

# SDF asse = +Z del JCS child; DOF FreeCAD = (JCS_parent - JCS_child) · asse.
# Spostare il child lungo +asse RIDUCE il DOF FreeCAD -> segno -1.
AXIS_SIGN_JCS_DISTANCE = -1.0


@dataclass
class JointMapping:
    sdf_type: str
    mapped: bool
    merge: bool
    limit_unit: str
    requires_dummy_link: bool = False
    note: str = ""


JOINT_MAP = {
    "Fixed": JointMapping("fixed", True, True, LIMIT_UNIT_M,
                          note="incastro: corpi fusi in un unico link"),
    "Revolute": JointMapping("revolute", True, False, LIMIT_UNIT_RAD),
    "Cylindrical": JointMapping("cylindrical", True, False, LIMIT_UNIT_M,
                                requires_dummy_link=True,
                                note="espansione revolute+prismatic su dummy link"),
    "Slider": JointMapping("prismatic", True, False, LIMIT_UNIT_M),
    "Ball": JointMapping("ball", True, False, LIMIT_UNIT_M),
    "Screw": JointMapping("screw", True, False, LIMIT_UNIT_M_PER_REV),
    "Gears": JointMapping("gearbox", True, False, LIMIT_UNIT_M),
    "Belt": JointMapping("gearbox", True, False, LIMIT_UNIT_M),
    "Distance": JointMapping(None, False, False, LIMIT_UNIT_M,
                             note="vincolo di distanza: nessun joint SDF nativo"),
    "Parallel": JointMapping(None, False, False, LIMIT_UNIT_M,
                             note="vincolo CAD, non e' un DOF fisico"),
    "Perpendicular": JointMapping(None, False, False, LIMIT_UNIT_M,
                                  note="vincolo CAD, non e' un DOF fisico"),
    "Angle": JointMapping(None, False, False, LIMIT_UNIT_RAD,
                          note="vincolo CAD, non e' un DOF fisico"),
    "RackPinion": JointMapping(None, False, False, LIMIT_UNIT_M,
                               note="accoppiamento rot-tras: nessun joint SDF nativo"),
}


def map_joint_type(fc_type):
    if fc_type not in JOINT_MAP:
        raise ValueError(
            "JointType FreeCAD sconosciuto: {!r} (attesi: {})".format(
                fc_type, ", ".join(FC_JOINT_TYPES)))
    return JOINT_MAP[fc_type]


@dataclass
class SolidSpec:
    name: str
    density_kg_m3: float
    reference_mass_kg: float
    reference_volume_m3: float
    reference_com_m: tuple
    stl_path: str = ""
    visual_mesh: str = ""
    collision_mesh: str = ""
    stl_collision_path: str = ""
    density_source: str = ""
    reference_inertia_kg_m2: object = None
    material: dict = field(default_factory=dict)
    pose: object = None

    def __post_init__(self):
        self.reference_com_m = tuple(float(c) for c in self.reference_com_m)
        if self.pose is None:
            self.pose = np.eye(4)
        elif not isinstance(self.pose, np.ndarray):
            self.pose = pose_to_matrix(self.pose)


@dataclass
class BodySpec:
    name: str
    label: str
    pose: object
    solids: list = field(default_factory=list)

    def __post_init__(self):
        if not isinstance(self.pose, np.ndarray):
            self.pose = pose_to_matrix(self.pose)


@dataclass
class JointLimits:
    lower: float = 0.0
    upper: float = 0.0
    lower_enabled: bool = False
    upper_enabled: bool = False
    unit: str = LIMIT_UNIT_M
    offset_m: float = 0.0

    def to_dict(self):
        """Snapshot fedele in SI, come in cad_manifest.json."""
        return {
            "lower": float(self.lower),
            "upper": float(self.upper),
            "lower_enabled": bool(self.lower_enabled),
            "upper_enabled": bool(self.upper_enabled),
            "unit": str(self.unit),
            "offset_m": float(self.offset_m),
        }

    @classmethod
    def from_dict(cls, raw):
        raw = raw or {}
        return cls(
            lower=float(raw.get("lower", 0.0)),
            upper=float(raw.get("upper", 0.0)),
            lower_enabled=bool(raw.get("lower_enabled", False)),
            upper_enabled=bool(raw.get("upper_enabled", False)),
            unit=str(raw.get("unit", LIMIT_UNIT_M)),
            offset_m=float(raw.get("offset_m", 0.0)),
        )

    @classmethod
    def from_entry(cls, entry):
        """Da voce di pipeline (limits) o dal formato legacy limit_lower/upper."""
        raw = entry.get("limits")
        if raw is not None:
            return cls.from_dict(raw)
        return cls(
            lower=float(entry.get("limit_lower", 0.0)),
            upper=float(entry.get("limit_upper", 0.0)),
            lower_enabled=entry.get("limit_lower") is not None,
            upper_enabled=entry.get("limit_upper") is not None,
            unit=LIMIT_UNIT_M,
        )


@dataclass
class JointSpec:
    name: str
    label: str
    fc_type: str
    parent: str
    child: str
    origin: object = None
    axis: tuple = (0.0, 0.0, 1.0)
    limits: JointLimits = field(default_factory=JointLimits)
    activated: bool = True
    sdf_type: str = None
    mapped: bool = True
    merged: bool = False
    requires_dummy_link: bool = False
    note: str = ""
    gear_ratio: float = None
    screw_thread_pitch_m: float = None

    def __post_init__(self):
        if self.origin is None:
            self.origin = np.eye(4)
        elif not isinstance(self.origin, np.ndarray):
            self.origin = pose_to_matrix(self.origin)
        self.axis = tuple(float(a) for a in self.axis)
        m = map_joint_type(self.fc_type)
        if self.sdf_type is None:
            self.sdf_type = m.sdf_type
        self.mapped = m.mapped
        self.merged = m.merge
        self.requires_dummy_link = m.requires_dummy_link
        if not self.note:
            self.note = m.note


@dataclass
class MassPoint:
    mass_kg: float
    com_m: tuple
    inertia_kg_m2: object
    R: object = None
    t: tuple = (0.0, 0.0, 0.0)

    def __post_init__(self):
        self.com_m = tuple(float(c) for c in self.com_m)
        self.inertia_kg_m2 = np.asarray(self.inertia_kg_m2, dtype=float).reshape(3, 3)
        if self.R is None:
            self.R = np.eye(3)
        else:
            self.R = np.asarray(self.R, dtype=float).reshape(3, 3)
        self.t = tuple(float(v) for v in self.t)


@dataclass
class RigidBody:
    name: str
    label: str
    pose: object
    members: list = field(default_factory=list)
    mass_kg: float = 0.0
    com_m: tuple = (0.0, 0.0, 0.0)
    inertia_kg_m2: object = None

    def __post_init__(self):
        if not isinstance(self.pose, np.ndarray):
            self.pose = pose_to_matrix(self.pose)
        if self.inertia_kg_m2 is None:
            self.inertia_kg_m2 = np.zeros((3, 3))


def quat_to_matrix(quat_xyzw, pos=(0.0, 0.0, 0.0)):
    x, y, z, w = (float(v) for v in quat_xyzw)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 1e-15:
        raise ValueError("quaternione nullo")
    x, y, z, w = x / n, y / n, z / n, w / n
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = [float(v) for v in pos]
    return m


def matrix_to_quat(m):
    r = np.asarray(m, dtype=float)[:3, :3]
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return (float(x), float(y), float(z), float(w))


def matrix_to_rpy(m):
    r = np.asarray(m, dtype=float)[:3, :3]
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    if abs(r[2, 0]) < 1.0 - 1e-12:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:
        roll = math.atan2(-r[1, 2], r[1, 1])
        yaw = 0.0
    return (roll, pitch, yaw)


def rpy_to_matrix(rpy, pos=(0.0, 0.0, 0.0)):
    roll, pitch, yaw = (float(v) for v in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rot = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = [float(v) for v in pos]
    return m


def pose_to_matrix(pose):
    """Pose [x,y,z,qx,qy,qz,qw] oppure [x,y,z,roll,pitch,yaw] -> 4x4."""
    vals = [float(v) for v in pose]
    if len(vals) == 7:
        return quat_to_matrix(vals[3:], vals[:3])
    if len(vals) == 6:
        return rpy_to_matrix(vals[3:], vals[:3])
    if isinstance(pose, np.ndarray) and pose.shape == (4, 4):
        return np.asarray(pose, dtype=float)
    raise ValueError("pose deve avere 6 (rpy) o 7 (quat) valori: {!r}".format(pose))


def matrix_to_pose(m):
    m = np.asarray(m, dtype=float)
    pos = tuple(float(v) for v in m[:3, 3])
    quat = matrix_to_quat(m)
    return (pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3])


def matrix_to_pose6(m):
    m = np.asarray(m, dtype=float)
    pos = tuple(float(v) for v in m[:3, 3])
    rpy = matrix_to_rpy(m)
    return (pos[0], pos[1], pos[2], rpy[0], rpy[1], rpy[2])


def transform_points(m, pts):
    m = np.asarray(m, dtype=float)
    pts = np.atleast_2d(np.asarray(pts, dtype=float))
    return (pts @ m[:3, :3].T) + m[:3, 3]


def rotate_inertia(inertia, R):
    inertia = np.asarray(inertia, dtype=float).reshape(3, 3)
    R = np.asarray(R, dtype=float).reshape(3, 3)
    return R @ inertia @ R.T


def parallel_axis(inertia, mass, offset):
    inertia = np.asarray(inertia, dtype=float).reshape(3, 3)
    d = np.asarray(offset, dtype=float).reshape(3)
    d2 = float(d @ d)
    return inertia + mass * (d2 * np.eye(3) - np.outer(d, d))


def aggregate_mass_properties(points):
    """Massa, CoM e inerzia attorno al CoM di un insieme di corpi.

    Ogni punto porta massa, CoM e tensore d'inerzia attorno al proprio CoM,
    oltre alla trasformazione (R, t) dal proprio frame al frame di aggregazione.
    """
    points = list(points)
    if not points:
        raise ValueError("aggregate_mass_properties: nessun corpo")
    total_mass = 0.0
    com_acc = np.zeros(3)
    world = []
    for p in points:
        p = p if isinstance(p, MassPoint) else MassPoint(*p)
        c_w = p.R @ np.asarray(p.com_m) + np.asarray(p.t)
        world.append((p, c_w))
        total_mass += p.mass_kg
        com_acc += p.mass_kg * c_w
    if total_mass <= 0.0:
        raise ValueError("aggregate_mass_properties: massa totale nulla")
    com = com_acc / total_mass
    inertia = np.zeros((3, 3))
    for p, c_w in world:
        i_loc = rotate_inertia(p.inertia_kg_m2, p.R)
        inertia += parallel_axis(i_loc, p.mass_kg, c_w - com)
    return total_mass, (float(com[0]), float(com[1]), float(com[2])), inertia


class _UnionFind:
    def __init__(self, names):
        self.parent = {n: n for n in names}

    def find(self, a):
        root = a
        while self.parent[root] != root:
            root = self.parent[root]
        while self.parent[a] != root:
            self.parent[a], a = root, self.parent[a]
        return root

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[rb] = ra


@dataclass
class MergeResult:
    """Esito di merge_fixed. Si scompone come la vecchia 3-tupla."""
    rigid_bodies: list
    movable_joints: list
    skipped_joints: list
    root_of: dict = field(default_factory=dict)

    def __iter__(self):
        return iter((self.rigid_bodies, self.movable_joints,
                     self.skipped_joints))

    def __len__(self):
        return 3


def merge_fixed(bodies, joints, merge_fixed_joints=True, preferred_root=None):
    """Fonde i corpi uniti da joint Fixed e riporta i joint mobili.

    Ritorna MergeResult (scomponibile come (rigid_bodies, movable_joints,
    skipped_joints)):
      - rigid_bodies: list[RigidBody] con massa/CoM/inerzia aggregati e
        membri (SolidSpec, pose nel frame del corpo rigido);
      - movable_joints: list[JointSpec] con parent/child aggiornati ai root
        e origin ri-espresso nel frame del child rigido;
      - skipped_joints: list[(JointSpec, motivo)] non esportabili;
      - root_of: dict nome body CAD -> nome link SDF (dopo il merge).

    preferred_root: se un corpo del gruppo fuso ha questo nome (tipicamente
    meta.root del manifest), il link risultante prende quel nome invece di
    quello del root union-find (che dipende dall'ordine parent/child del
    joint Fixed e puo' sparire dai config).
    """
    bodies = {b.name: b for b in bodies}
    joints = list(joints)
    uf = _UnionFind(bodies)
    welds = []
    skipped = []

    for j in joints:
        if not j.activated:
            skipped.append((j, "joint disattivato (Activated=false)"))
            continue
        if j.parent not in bodies or j.child not in bodies:
            skipped.append((j, "parent/child assente nel manifest"))
            continue
        if not j.mapped:
            skipped.append((j, j.note or "joint non mappabile su SDF"))
            continue
        if j.merged and merge_fixed_joints:
            uf.union(j.parent, j.child)
            welds.append(j)

    groups = {}
    for name in bodies:
        groups.setdefault(uf.find(name), []).append(name)

    group_name = {}
    for root, members in groups.items():
        if preferred_root is not None and preferred_root in members:
            group_name[root] = preferred_root
        else:
            group_name[root] = root

    rigid_bodies = []
    root_of = {}
    for root, members in groups.items():
        link_name = group_name[root]
        members = sorted(members, key=lambda n: (n != link_name, n))
        root_body = bodies[link_name]
        mass_points = []
        member_solids = []
        for name in members:
            body = bodies[name]
            t_rel = np.linalg.inv(root_body.pose) @ body.pose
            for solid in body.solids:
                t_solid = t_rel @ solid.pose
                com_ref = np.asarray(solid.reference_com_m, dtype=float)
                if solid.reference_inertia_kg_m2 is not None:
                    inertia = np.asarray(solid.reference_inertia_kg_m2,
                                         dtype=float).reshape(3, 3)
                else:
                    inertia = np.zeros((3, 3))
                mass_points.append(MassPoint(
                    mass_kg=solid.reference_mass_kg,
                    com_m=com_ref,
                    inertia_kg_m2=inertia,
                    R=t_solid[:3, :3],
                    t=t_solid[:3, 3],
                ))
                member_solids.append((solid, name, t_solid))
            root_of[name] = link_name
        mass, com, inertia = aggregate_mass_properties(mass_points)
        rb = RigidBody(
            name=link_name,
            label=root_body.label,
            pose=root_body.pose,
            members=member_solids,
            mass_kg=mass,
            com_m=com,
            inertia_kg_m2=inertia,
        )
        rigid_bodies.append(rb)

    rigid_bodies.sort(key=lambda rb: rb.name)
    movable = []
    for j in joints:
        if not j.activated or j.parent not in bodies or j.child not in bodies:
            continue
        if not j.mapped or (j.merged and merge_fixed_joints):
            continue
        parent_root = root_of[j.parent]
        child_root = root_of[j.child]
        if parent_root == child_root:
            skipped.append((j, "parent e child fusi nello stesso corpo rigido"))
            continue
        origin = j.origin.copy()
        if j.child != child_root:
            t_rel = np.linalg.inv(bodies[child_root].pose) @ bodies[j.child].pose
            origin = t_rel @ origin
        moved = JointSpec(
            name=j.name, label=j.label, fc_type=j.fc_type,
            parent=parent_root, child=child_root,
            origin=origin, axis=j.axis, limits=j.limits,
            activated=j.activated, sdf_type=j.sdf_type,
            note=j.note, gear_ratio=j.gear_ratio,
            screw_thread_pitch_m=j.screw_thread_pitch_m,
        )
        movable.append(moved)

    movable.sort(key=lambda jj: jj.name)
    return MergeResult(rigid_bodies, movable, skipped, root_of=root_of)


def joint_axis_world(joint, child_pose):
    """Asse del joint nel frame globale date la pose del child."""
    axis_local = np.asarray(joint.axis, dtype=float)
    axis_in_child = joint.origin[:3, :3] @ axis_local
    return child_pose[:3, :3] @ axis_in_child


def freecad_limits_to_sdf(limits, axis_sign=AXIS_SIGN_JCS_DISTANCE):
    """Limiti assoluti FreeCAD -> spostamenti SDF dal montato.

    FreeCAD LengthMin/Max (o AngleMin/Max) sono valori assoluti del DOF del
    joint (distanza/angolo dei JCS da coincidenza). In SDF q=0 e' la pose
    montata e +q segue l'asse del joint. La conversione sottrae `offset_m`
    (valore corrente del DOF) e applica `axis_sign` (verso dell'asse SDF
    rispetto al DOF FreeCAD).

    Ritorna (lower, upper) per l'SDF, None per il lato disabilitato.
    """
    a = ((limits.lower - limits.offset_m) * axis_sign, limits.lower_enabled)
    b = ((limits.upper - limits.offset_m) * axis_sign, limits.upper_enabled)
    if a[0] > b[0]:
        a, b = b, a
    return (a[0] if a[1] else None, b[0] if b[1] else None)


def limit_bounds(limits):
    """Limiti gia' in coordinate SDF (offset_m = 0) -> (lower, upper)."""
    return freecad_limits_to_sdf(limits, axis_sign=1.0)


def limits_from_slider_mm(lower_mm, upper_mm, lower_enabled, upper_enabled):
    return JointLimits(lower=lower_mm * MM_TO_M, upper=upper_mm * MM_TO_M,
                       lower_enabled=lower_enabled, upper_enabled=upper_enabled,
                       unit=LIMIT_UNIT_M)


def limits_from_revolute_deg(lower_deg, upper_deg, lower_enabled, upper_enabled):
    return JointLimits(lower=lower_deg * DEG_TO_RAD,
                       upper=upper_deg * DEG_TO_RAD,
                       lower_enabled=lower_enabled, upper_enabled=upper_enabled,
                       unit=LIMIT_UNIT_RAD)


def screw_pitch_from_distance(distance_mm, default_m=0.001):
    if distance_mm is None or abs(distance_mm) < 1e-12:
        return default_m
    return abs(distance_mm) * MM_TO_M


def gear_ratio_from_distances(distance, distance2, default=1.0):
    if distance and abs(distance) > 1e-12:
        return float(distance2) / float(distance)
    if distance2 is not None and abs(distance2) > 1e-12:
        return float(distance2)
    return default
