"""Tests for :mod:`cad.urdf_skeleton` -- the C6 kinematic URDF skeleton (issue #8).

The suite is built around one golden file, ``fixtures/urdf_skeleton_sample.urdf``,
regenerated from ``fixtures/cad_manifest_urdf_skeleton.json``. Reproducibility is
a *byte* comparison against the committed document, so a drifted float or a
reordered element fails here and in the CI regeneration step.

The remaining tests hold the contract around that golden file:

* the **kinematic tree is equivalent** to the IR (same links, same joints, same
  parent/child, same axes, the documented type mapping) -- this is the C6
  acceptance criterion;
* the tree is **one connected root**, so ``robot_state_publisher`` can build it;
* the artefact is **lean** -- no ``inertial`` / ``visual`` / ``collision`` /
  ``material`` / ``gazebo`` / ``plugin`` / ``transmission`` / ``sensor`` element
  anywhere (asserted explicitly);
* the validator **rejects** every defect it claims to catch (a validator that
  never fails is not a gate);
* ``check_urdf`` really accepts the file when the ROS CLI is installed.
"""

import copy
import json
import math
import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
from cad import urdf_skeleton as us

FIXTURES = Path(__file__).resolve().parent / "fixtures"
IR_PATH = FIXTURES / "cad_manifest_urdf_skeleton.json"
GOLDEN = FIXTURES / "urdf_skeleton_sample.urdf"
C4_IR_PATH = FIXTURES / "cad_manifest_sample_assembly.json"

#: The C6 mapping pinned in the tests (docs/urdf-skeleton.md §3), not re-derived
#: from the emitter: an expectation table that shared the emitter's code would
#: pass even if the emitter changed its mind.
EXPECTED_TYPES = {
    "pto_joint": "prismatic",     # Slider
    "keel_joint": "fixed",        # Fixed
    "hinge_joint": "revolute",    # Revolute, bounds enabled
    "spin_joint": "continuous",   # Revolute, both bounds disabled
    "piston_joint": "prismatic",  # Cylindrical -> primary (linear) DOF
}


def _load(path):
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


@pytest.fixture()
def ir():
    return _load(IR_PATH)


@pytest.fixture()
def text(ir):
    return us.generate_urdf(ir)


def _golden_bytes():
    return GOLDEN.read_bytes()


def _joint_of(skeleton, name):
    return next(j for j in skeleton["joints"] if j["name"] == name)


def _ir_joint(ir_doc, name):
    return next(j for j in ir_doc["joints"] if j["name"] == name)


def _vectors(value):
    return [float(part) for part in value.split()]


# --------------------------------------------------------------------------- #
# the golden artefact
# --------------------------------------------------------------------------- #


def test_golden_byte_identity(ir):
    """The committed artefact is exactly what the emitter produces today."""
    assert us.generate_urdf(ir).encode("utf-8") == _golden_bytes()


def test_generation_is_idempotent(ir):
    """Two runs give identical bytes: no timestamp, no dict-order dependence."""
    assert us.generate_urdf(ir) == us.generate_urdf(ir)


def test_golden_is_ascii_and_newline_terminated():
    data = _golden_bytes()
    assert data.endswith(b"\n")
    assert not data.endswith(b"\n\n")
    data.decode("ascii")  # raises when a non-ASCII byte sneaked in


def test_golden_has_no_absolute_path_or_timestamp():
    body = _golden_bytes().decode("ascii")
    assert "/opt/" not in body and "/home/" not in body
    # no ISO-ish date anywhere in the document
    assert not re.search(r"\d{4}-\d{2}-\d{2}", body)


def test_golden_has_no_open_point(ir):
    """A closed contract: no TODO/FIXME/'?' marker in an artefact the ROS stack parses."""
    body = us.generate_urdf(ir)
    for marker in ("TODO", "FIXME", "XXX", "???"):
        assert marker not in body


def test_robot_name_is_the_ir_root(ir, text):
    skeleton = us.parse_skeleton(text)
    assert skeleton["name"] == ir["meta"]["root"] == "hull"


# --------------------------------------------------------------------------- #
# C6 acceptance: kinematic tree equivalence with the IR
# --------------------------------------------------------------------------- #


def test_links_match_ir_bodies_in_order(ir, text):
    assert us.parse_skeleton(text)["links"] == [b["name"] for b in ir["bodies"]]


def test_joints_match_ir_joints_in_order(ir, text):
    skeleton = us.parse_skeleton(text)
    assert [j["name"] for j in skeleton["joints"]] == [j["name"] for j in ir["joints"]]


def test_types_follow_the_documented_mapping(text):
    skeleton = us.parse_skeleton(text)
    assert {j["name"]: j["type"] for j in skeleton["joints"]} == EXPECTED_TYPES


def test_topology_matches_ir(ir, text):
    """Same parent/child edge for every joint: the tree is transported, not rebuilt."""
    skeleton = us.parse_skeleton(text)
    for ir_joint in ir["joints"]:
        emitted = _joint_of(skeleton, ir_joint["name"])
        assert emitted["parent"] == ir_joint["parent"]
        assert emitted["child"] == ir_joint["child"]


def test_axes_match_the_normalised_ir_axis(ir, text):
    skeleton = us.parse_skeleton(text)
    for ir_joint in ir["joints"]:
        emitted = _joint_of(skeleton, ir_joint["name"])
        if emitted["type"] == "fixed":
            assert emitted["axis"] is None  # an axis on a rigid edge says nothing
            continue
        axis = _vectors(emitted["axis"])
        assert math.isclose(float(np.linalg.norm(axis)), 1.0, abs_tol=1e-9)
        expected = np.asarray(us.normalize_axis(ir_joint["axis"]))
        assert np.allclose(axis, expected, atol=1e-9)


def test_origins_reconstruct_the_ir_pose(ir, text):
    """URDF ``xyz``/``rpy`` must rebuild the IR joint pose as a rigid transform."""
    skeleton = us.parse_skeleton(text)
    for ir_joint in ir["joints"]:
        emitted = _joint_of(skeleton, ir_joint["name"])
        xyz = _vectors(emitted["origin"]["xyz"])
        rpy = _vectors(emitted["origin"]["rpy"])
        pose = ir_joint["origin"]
        assert np.allclose(xyz, pose[:3], atol=1e-9)
        assert np.allclose(us.quat_to_rpy(pose[3:]), rpy, atol=1e-9)


def test_single_root_connected_tree(text):
    """``robot_state_publisher`` needs exactly one root and no floating body."""
    assert us.collect_errors(text, require_single_root=True) == []
    skeleton = us.parse_skeleton(text)
    children = {j["child"] for j in skeleton["joints"]}
    roots = [name for name in skeleton["links"] if name not in children]
    assert roots == ["hull"]
    assert children == set(skeleton["links"]) - {"hull"}


def test_limits_are_legal(text):
    """Bounded joints carry a limit; the unbounded one carries no bounds."""
    skeleton = us.parse_skeleton(text)
    for joint in skeleton["joints"]:
        if joint["type"] == "fixed":
            assert joint["limit"] is None
        elif joint["type"] == "continuous":
            assert joint["limit"] is not None
            assert "lower" not in joint["limit"] and "upper" not in joint["limit"]
            assert float(joint["limit"]["effort"]) > 0
            assert float(joint["limit"]["velocity"]) > 0
        else:
            assert float(joint["limit"]["lower"]) <= float(joint["limit"]["upper"])
            assert float(joint["limit"]["effort"]) > 0
            assert float(joint["limit"]["velocity"]) > 0


def test_declared_effort_is_carried(ir, text):
    """``hinge_joint`` declares effort/velocity in the IR; neither is overridden."""
    limit = _joint_of(us.parse_skeleton(text), "hinge_joint")["limit"]
    assert (limit["effort"], limit["velocity"]) == ("120", "3.5")


# --------------------------------------------------------------------------- #
# C6 acceptance: the artefact is a skeleton, nothing else
# --------------------------------------------------------------------------- #


def test_no_geometry_material_or_plugin_element(text):
    """Explicit leanness assertion (the C6 'not a twin of the SDF' criterion)."""
    root = us.ET.fromstring(text)
    tags = {us._tag_name(element) for element in root.iter()}
    assert tags == {"robot", "link", "joint", "parent", "child", "origin", "axis", "limit"}
    for forbidden in us.LEAN_TAGS:
        assert forbidden not in tags


def test_links_have_no_children(text):
    root = us.ET.fromstring(text)
    for link in (e for e in root if us._tag_name(e) == "link"):
        assert list(link) == []


def test_module_emits_no_forbidden_tag():
    """Belt and braces: the emitter source never writes a forbidden element."""
    source = Path(us.__file__).read_text(encoding="utf-8")
    body = source.split("LEAN_TAGS = (", 1)[1].split(")", 1)[1]  # past the constant
    for forbidden in us.LEAN_TAGS:
        assert "<{}".format(forbidden) not in body


# --------------------------------------------------------------------------- #
# the validator is a gate: every defect it claims to catch must fail
# --------------------------------------------------------------------------- #


def _broken(text, old, new):
    assert old in text, old
    return text.replace(old, new, 1)


def test_validator_accepts_the_golden(text):
    assert us.collect_errors(text) == []
    us.validate_urdf(text, require_single_root=True)


def test_validator_rejects_malformed_xml():
    assert us.collect_errors("<robot name='a'><link name='b'>") != []


def test_validator_rejects_wrong_root():
    errors = us.collect_errors('<?xml version="1.0"?><model name="a"/>')
    assert errors and "robot" in errors[0]


def test_validator_rejects_missing_robot_name(text):
    errors = us.collect_errors(_broken(text, '<robot name="hull">', "<robot>"))
    assert any("'name'" in e for e in errors)


def test_validator_rejects_duplicate_link(text):
    errors = us.collect_errors(_broken(text, '<link name="keel"/>', '<link name="hull"/>'))
    assert any("duplicato" in e for e in errors)


def test_validator_rejects_duplicate_joint(text):
    errors = us.collect_errors(
        _broken(text, '<joint name="keel_joint" type="fixed">',
                '<joint name="pto_joint" type="fixed">'))
    assert any("duplicato" in e for e in errors)


def test_validator_rejects_unknown_joint_type(text):
    errors = us.collect_errors(
        _broken(text, '<joint name="hinge_joint" type="revolute">',
                '<joint name="hinge_joint" type="cylindrical">'))
    assert any("non URDF" in e for e in errors)


def test_validator_rejects_dangling_parent(text):
    errors = us.collect_errors(_broken(text, '<parent link="hull"/>\n    <child link="piston"/>',
                                       '<parent link="ghost"/>\n    <child link="piston"/>'))
    assert any("parent 'ghost'" in e for e in errors)


def test_validator_rejects_dangling_child(text):
    errors = us.collect_errors(
        _broken(text, '<child link="keel"/>', '<child link="ghost"/>'))
    assert any("child 'ghost'" in e for e in errors)


def test_validator_rejects_self_loop(text):
    errors = us.collect_errors(
        _broken(text, '<parent link="hull"/>\n    <child link="keel"/>',
                '<parent link="keel"/>\n    <child link="keel"/>'))
    assert any("coincidono" in e for e in errors)


def test_validator_requires_limit_on_bounded_joint(text):
    errors = us.collect_errors(
        _broken(text, '    <limit effort="120" velocity="3.5" lower="-0.3" upper="0.3"/>\n',
                ""))
    assert any("richiede <limit>" in e for e in errors)


def test_validator_requires_effort_and_velocity(text):
    errors = us.collect_errors(_broken(text, '<limit effort="120" velocity="3.5"',
                                       '<limit velocity="3.5"'))
    assert any("'effort'" in e for e in errors)


def test_validator_rejects_inverted_bounds(text):
    errors = us.collect_errors(
        _broken(text, 'lower="-0.6" upper="0.6"', 'lower="0.6" upper="-0.6"'))
    assert any("lower" in e and "upper" in e for e in errors)


def test_validator_requires_axis_on_movable_joint(text):
    errors = us.collect_errors(_broken(text, '    <axis xyz="0 0 1"/>\n', ""))
    assert any("richiede <axis>" in e for e in errors)


def test_validator_rejects_null_axis(text):
    errors = us.collect_errors(_broken(text, '<axis xyz="0 0 1"/>', '<axis xyz="0 0 0"/>'))
    assert any("axis nullo" in e for e in errors)


def test_validator_rejects_unparsable_vector(text):
    errors = us.collect_errors(_broken(text, 'xyz="0 0 0.4"', 'xyz="0 0"'))
    assert any("attesi 3 numeri" in e for e in errors)


def test_validator_rejects_forbidden_element(text):
    """The leanness rule is enforced by the validator, not only by the tests."""
    errors = us.collect_errors(
        _broken(text, '<link name="hull"/>', '<link name="hull"><inertial/></link>'))
    assert any("inertial" in e for e in errors)


def test_validator_rejects_cycle():
    """A joint graph with no root is refused (and reported as such)."""
    cyclic = (
        '<?xml version="1.0"?>\n'
        '<robot name="loop">\n'
        '  <link name="a"/>\n'
        '  <link name="b"/>\n'
        '  <joint name="j1" type="continuous">\n'
        '    <parent link="a"/>\n    <child link="b"/>\n'
        '    <origin xyz="0 0 0" rpy="0 0 0"/>\n    <axis xyz="0 0 1"/>\n'
        '    <limit effort="1" velocity="1"/>\n'
        "  </joint>\n"
        '  <joint name="j2" type="continuous">\n'
        '    <parent link="b"/>\n    <child link="a"/>\n'
        '    <origin xyz="0 0 0" rpy="0 0 0"/>\n    <axis xyz="0 0 1"/>\n'
        '    <limit effort="1" velocity="1"/>\n'
        "  </joint>\n"
        "</robot>\n"
    )
    errors = us.collect_errors(cyclic)
    assert any("radice" in e for e in errors)


def test_validator_reports_multiple_roots_only_in_strict_mode(text):
    """Drop a whole joint: its child becomes a second root, named as such."""
    orphaned = _broken(text, '  <joint name="piston_joint" type="prismatic">\n'
                             '    <parent link="hull"/>\n'
                             '    <child link="piston"/>\n'
                             '    <origin xyz="0 -0.6 0" rpy="0 0 0"/>\n'
                             '    <axis xyz="0 1 0"/>\n'
                             '    <limit effort="1000" velocity="100" lower="-0.25" upper="0.25"/>\n'
                             "  </joint>\n", "")
    assert us.collect_errors(orphaned) == []
    strict = us.collect_errors(orphaned, require_single_root=True)
    assert any("radici" in e and "piston" in e for e in strict)


# --------------------------------------------------------------------------- #
# joint-type resolution: the documented rules, and their failure modes
# --------------------------------------------------------------------------- #


def _joint(**kwargs):
    base = {
        "name": "j",
        "fc_type": "Slider",
        "parent": "a",
        "child": "b",
        "axis": [0.0, 0.0, 1.0],
        "activated": True,
        "limits": {"lower": -1.0, "upper": 1.0, "lower_enabled": True,
                   "upper_enabled": True, "unit": "m", "offset_m": 0.0},
    }
    base.update(kwargs)
    return base


@pytest.mark.parametrize("fc_type,expected", [
    ("Fixed", "fixed"),
    ("Slider", "prismatic"),
    ("Revolute", "revolute"),
    ("Continuous", "continuous"),
    ("Cylindrical", "prismatic"),
    ("Ball", "continuous"),
    ("Distance", "fixed"),
    ("Screw", "prismatic"),
    ("RackPinion", "revolute"),
])
def test_fc_type_table(fc_type, expected):
    assert us.urdf_joint_type(_joint(fc_type=fc_type)) == expected


def test_deactivated_joint_becomes_fixed():
    """No DOF, but the child body stays in the tree: the edge is rigid."""
    assert us.urdf_joint_type(_joint(activated=False)) == "fixed"


def test_revolute_without_bounds_becomes_continuous():
    unbounded = {"lower": 0.0, "upper": 0.0, "lower_enabled": False,
                 "upper_enabled": False, "unit": "rad", "offset_m": 0.0}
    assert us.urdf_joint_type(_joint(fc_type="Revolute", limits=unbounded)) == "continuous"


def test_revolute_with_one_bound_stays_revolute():
    half = {"lower": -1.0, "upper": 0.0, "lower_enabled": True,
            "upper_enabled": False, "unit": "rad", "offset_m": 0.0}
    assert us.urdf_joint_type(_joint(fc_type="Revolute", limits=half)) == "revolute"


def test_unmappable_fc_type_is_a_hard_error():
    """A DOF is never guessed: an unknown CAD joint stops the run."""
    with pytest.raises(us.UrdfSkeletonError) as excinfo:
        us.urdf_joint_type(_joint(fc_type="WeirdCoupling"))
    assert "WeirdCoupling" in str(excinfo.value)
    assert "urdf_type" in str(excinfo.value)


def test_explicit_urdf_type_wins():
    assert us.urdf_joint_type(_joint(fc_type="WeirdCoupling", urdf_type="fixed")) == "fixed"


def test_explicit_urdf_type_is_validated():
    with pytest.raises(us.UrdfSkeletonError):
        us.urdf_joint_type(_joint(urdf_type="planar"))


def test_null_axis_is_a_hard_error():
    with pytest.raises(us.UrdfSkeletonError):
        us.normalize_axis([0.0, 0.0, 0.0])


def test_null_quaternion_is_a_hard_error():
    with pytest.raises(us.UrdfSkeletonError):
        us.normalize_quat([0.0, 0.0, 0.0, 0.0])


def test_deactivated_joint_is_still_emitted_rigid(ir):
    """A body connected by a deactivated joint must not float away."""
    doc = copy.deepcopy(ir)
    _ir_joint(doc, "pto_joint")["activated"] = False
    text = us.generate_urdf(doc)
    joint = _joint_of(us.parse_skeleton(text), "pto_joint")
    assert joint["type"] == "fixed"
    assert us.collect_errors(text, require_single_root=True) == []


# --------------------------------------------------------------------------- #
# rotation and the IR contract
# --------------------------------------------------------------------------- #


def _rotation_of(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _quaternion_matrix(quat):
    x, y, z, w = us.normalize_quat(quat)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


@pytest.mark.parametrize("axis", [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
                                  (1.0, 2.0, -3.0)])
@pytest.mark.parametrize("degrees", [0.0, 12.5, 90.0, 179.0, -75.0, -90.0])
def test_rpy_reconstructs_the_quaternion(axis, degrees):
    half = math.radians(degrees) / 2.0
    norm = math.sqrt(sum(v * v for v in axis))
    quat = [v / norm * math.sin(half) for v in axis] + [math.cos(half)]
    rpy = us.quat_to_rpy(quat)
    assert np.allclose(_rotation_of(rpy), _quaternion_matrix(quat), atol=1e-9)


def test_gimbal_lock_is_canonicalised():
    """At |pitch| = 90 deg the triple is pinned (roll = 0) and stays reconstructing."""
    # q = qz(yaw) * qy(90 deg) * qx(roll) with roll = 30 deg, yaw = 45 deg:
    # the singularity absorbs the roll, which the canonical form pins to zero.
    roll, pitch, yaw = math.radians(30.0), math.radians(90.0), math.radians(45.0)
    quat = [
        math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2)
        - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2),
        math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2),
        math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2),
        math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2)
        + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2),
    ]
    rpy = us.quat_to_rpy(quat)
    assert rpy[0] == 0.0                      # roll is pinned
    assert math.isclose(rpy[1], math.pi / 2, abs_tol=1e-9)
    assert np.allclose(_rotation_of(rpy), _quaternion_matrix(quat), atol=1e-9)


def test_random_rotations_round_trip():
    rng = np.random.default_rng(7)
    for _ in range(500):
        quat = list(rng.normal(size=4))
        rpy = us.quat_to_rpy(quat)
        assert np.allclose(_rotation_of(rpy), _quaternion_matrix(quat), atol=1e-8)


def test_ir_is_validated_before_rendering():
    """The emitter refuses a document that is not a v1 IR."""
    with pytest.raises(us.UrdfSkeletonError):
        us.generate_urdf({"schema": "cad-to-gazebo/cad-ir/1", "bodies": [], "joints": []})


def test_ir_with_unknown_body_in_a_joint_is_rejected(ir):
    doc = copy.deepcopy(ir)
    _ir_joint(doc, "pto_joint")["child"] = "ghost"
    with pytest.raises(us.UrdfSkeletonError) as excinfo:
        us.skeleton_joints(doc) and us._collect_warnings(doc, us.skeleton_joints(doc))
    assert "ghost" in str(excinfo.value)


def test_inverted_ir_limits_are_rejected(ir):
    doc = copy.deepcopy(ir)
    _ir_joint(doc, "pto_joint")["limits"]["lower"] = 5.0
    with pytest.raises(us.UrdfSkeletonError):
        us.skeleton_joints(doc) and us._joint_limits(_ir_joint(doc, "pto_joint"),
                                                     "prismatic", [])


# --------------------------------------------------------------------------- #
# the committed C4 IR: the real fixture, warnings instead of invented topology
# --------------------------------------------------------------------------- #


def test_c4_fixture_keeps_the_unconnected_body_and_warns():
    """``float`` has no joint in the C4 IR: the emitter says so, it does not bolt it."""
    doc = _load(C4_IR_PATH)
    text = us.generate_urdf(doc)
    skeleton = us.parse_skeleton(text)
    assert "float" in skeleton["links"]
    assert all(j["child"] != "float" for j in skeleton["joints"])
    warnings = us._warnings_for(doc)
    assert any("radici" in w for w in warnings) and any("float" in w for w in warnings)
    errors = us.collect_errors(text, require_single_root=True)
    assert errors and "radici" in errors[0]


def test_c4_fixture_emits_the_two_real_joints():
    doc = _load(C4_IR_PATH)
    skeleton = us.parse_skeleton(us.generate_urdf(doc))
    assert {j["name"]: j["type"] for j in skeleton["joints"]} == {
        "pto_joint": "prismatic", "keel_joint": "fixed"}


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #


def _run_cli(argv):
    return subprocess.run([sys.executable, "-m", "cad.urdf_skeleton"] + argv,
                          cwd=str(Path(us.__file__).resolve().parent),
                          capture_output=True, text=True)


def test_cli_writes_the_committed_bytes(tmp_path):
    out = tmp_path / "skeleton.urdf"
    result = _run_cli(["--manifest", str(IR_PATH), "--out", str(out), "--strict"])
    assert result.returncode == 0, result.stderr
    assert out.read_bytes() == _golden_bytes()
    # the placeholder warning is reported, never silent
    assert "[WARN]" in result.stderr and "effort" in result.stderr


def test_cli_stdout_mode():
    result = _run_cli(["--manifest", str(IR_PATH)])
    assert result.returncode == 0
    assert result.stdout == _golden_bytes().decode("ascii")


def test_cli_strict_fails_on_multiple_roots():
    result = _run_cli(["--manifest", str(C4_IR_PATH), "--strict"])
    assert result.returncode == 1
    assert "radici" in result.stderr


def test_cli_fails_on_an_unmappable_joint(tmp_path, ir):
    doc = copy.deepcopy(ir)
    _ir_joint(doc, "pto_joint")["fc_type"] = "WeirdCoupling"
    manifest = tmp_path / "ir.json"
    manifest.write_text(json.dumps(doc), encoding="utf-8")
    result = _run_cli(["--manifest", str(manifest)])
    assert result.returncode == 1
    assert "WeirdCoupling" in result.stderr


# --------------------------------------------------------------------------- #
# the external gate, when the ROS CLI is available
# --------------------------------------------------------------------------- #


def test_check_urdf_accepts_the_golden():
    """``check_urdf`` (urdfdom) is the official parser; run it when it exists."""
    check_urdf = shutil.which("check_urdf")
    if check_urdf is None:
        pytest.skip("check_urdf (urdfdom) non installato: gate headless via collect_errors")
    result = subprocess.run([check_urdf, str(GOLDEN)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Successfully Parsed" in (result.stdout + result.stderr)
