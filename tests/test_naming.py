"""Contract tests for the IR naming rules (issue #4, C3, deliverable 3).

Two things are pinned here:

* the sanitisation / uniqueness rules of ``cad.naming``;
* the fact that they are the *only* implementation: no other module in the
  repository defines a sanitiser, and the schema's ``name`` pattern is exactly
  :data:`cad.naming.NAME_PATTERN`.
"""

import json
import re
from pathlib import Path

import pytest
from cad import ir_schema, naming

ROOT = Path(__file__).resolve().parent.parent
FIXTURES = Path(__file__).resolve().parent / "fixtures"

CORPUS = [
    "Hull Shell",
    "pto shaft",
    "  Spaced  ",
    "flange 3/4\"",
    "alluminio-6060",
    "311",
    "_leading_underscore",
    "trailing_",
    "double__underscore",
    "già_UTC",
    "A.B.C",
    "mooring line #2",
    "x" * 200,
    "",
    None,
]


# --------------------------------------------------------------------------- #
# sanitisation
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("raw,expected", [
    ("Hull Shell", "hull_shell"),
    ("pto shaft", "pto_shaft"),
    ("  Spaced  ", "spaced"),
    ("flange 3/4\"", "flange_34"),
    ("alluminio-6060", "alluminio6060"),
    ("double__underscore", "double__underscore"),
    ("_leading_underscore", "leading_underscore"),
    ("trailing_", "trailing"),
    ("311", "311"),
    ("", "link"),
    (None, "link"),
    ("!!!", "link"),
    ("già_UTC", "gi_utc"),
])
def test_sanitize_table(raw, expected):
    assert naming.sanitize(raw) == expected


def test_sanitize_is_idempotent_over_the_corpus():
    for raw in CORPUS:
        once = naming.sanitize(raw)
        assert naming.sanitize(once) == once, raw


def test_sanitize_output_is_always_a_valid_ir_name():
    for raw in CORPUS:
        assert naming.is_valid_name(naming.sanitize(raw)), raw


def test_sanitize_uses_the_custom_fallback():
    assert naming.sanitize("", fallback="j_joint") == "j_joint"
    assert naming.sanitize(None, fallback="j_joint") == "j_joint"


def test_sanitize_truncates_instead_of_rejecting():
    name = naming.sanitize("x" * 200)
    assert len(name) == naming.MAX_NAME_LEN
    assert naming.is_valid_name(name)


def test_is_valid_name_rejects_the_obvious_dangers():
    for bad in ("Hull", "hull shell", "hull-shell", "hull.shell", "_hull",
                "hull_", "", "link/solid", "hull#2", None, 3):
        assert not naming.is_valid_name(bad), bad


# --------------------------------------------------------------------------- #
# uniqueness
# --------------------------------------------------------------------------- #


def test_unique_reserves_the_name_and_mutates_the_scope():
    taken = set()
    assert naming.unique("hull", taken) == "hull"
    assert naming.unique("hull", taken) == "hull_2"
    assert naming.unique("hull", taken) == "hull_3"
    assert taken == {"hull", "hull_2", "hull_3"}


def test_unique_does_not_collide_with_a_literal_suffix():
    taken = {"hull", "hull_2"}
    assert naming.unique("hull", taken) == "hull_3"


def test_unique_keeps_the_result_a_valid_name():
    taken = set()
    for _ in range(12):
        assert naming.is_valid_name(naming.unique("hull__shell", taken))


def test_split_size_suffix():
    assert naming.split_size_suffix("hull_2") == ("hull", 2)
    assert naming.split_size_suffix("hull_311") == ("hull", 311)
    assert naming.split_size_suffix("hull") == ("hull", 1)


def test_valid_unique():
    assert naming.valid_unique(["hull", "pto_shaft"])
    assert not naming.valid_unique(["hull", "hull"])
    assert not naming.valid_unique(["hull", "Hull"])


# --------------------------------------------------------------------------- #
# names built by the contract helpers
# --------------------------------------------------------------------------- #


def test_ir_name_from_path():
    assert naming.ir_name_from_path(
        "full_transmission/spare_structure/profiles/311") == \
        "full_transmission_spare_structure_profiles_311"
    assert naming.ir_name_from_path("/float/#2") == "float"
    assert naming.ir_name_from_path(None) == "link"
    assert naming.is_valid_name(
        naming.ir_name_from_path("Bolts M8 (12 pz)/washer"))


def test_name_builders_match_the_shipped_fixtures():
    """The builders must produce exactly the names the producers wrote."""
    full = json.loads((FIXTURES / "cad_manifest_full.json").read_text())
    frame_names = {f["name"] for f in full["frames"]}
    assert naming.frame_name("hull", "pto_joint") in frame_names
    assert naming.frame_name("pto_shaft", "pto_joint") in frame_names
    assert naming.body_name("pto_shaft") == "pto_shaft"
    assert naming.joint_name("pto_joint") == "pto_joint"
    assert {a["name"] for a in full["anchors"]} == {
        naming.anchor_name("jet", "pto_shaft", "bottom"),
        naming.anchor_name("mooring", "hull", "base"),
    }
    assert {s["name"] for s in full["sensors"]} == {
        naming.sensor_name("imu", "hull"),
        naming.sensor_name("encoder", "pto_joint"),
    }
    assert full["bodies"][0]["solids"][0]["name"] == naming.body_name("hull") + \
        naming.MESH_SEPARATOR + naming.sanitize("shell")


def test_mesh_names():
    assert naming.visual_mesh_name("hull", "hull__shell") == \
        "meshes/visual/hull__hull__shell.stl"
    assert naming.collision_mesh_name("pto_shaft", "pto_shaft__mass") == \
        "meshes/collision/pto_shaft__pto_shaft__mass_convex.stl"


# --------------------------------------------------------------------------- #
# exactly one implementation, pinned by the schema
# --------------------------------------------------------------------------- #


def test_schema_name_pattern_is_the_naming_contract():
    schema = ir_schema.load_schema()
    name_def = schema["$defs"]["name"]
    assert name_def["pattern"] == naming.NAME_PATTERN
    assert name_def["maxLength"] == naming.MAX_NAME_LEN


def test_sanitized_names_satisfy_the_schema_pattern():
    pattern = re.compile(ir_schema.load_schema()["$defs"]["name"]["pattern"])
    for raw in CORPUS:
        assert pattern.match(naming.sanitize(raw)), raw


def test_no_second_sanitizer_in_the_repository():
    """Deliverable 3: ONE implementation, no second variant anywhere."""
    offenders = []
    for path in sorted(ROOT.glob("*.py")):
        if path.name == "naming.py":
            continue
        source = path.read_text(encoding="utf-8")
        if re.search(r"^def .*sanitiz", source, re.MULTILINE):
            offenders.append(path.name)
    assert offenders == [], offenders


def test_exporter_reuses_the_single_implementation():
    source = (ROOT / "freecad_exporter.py").read_text(encoding="utf-8")
    assert "_sanitize = naming.sanitize" in source
    assert "_unique_name = naming.unique" in source
    assert "def _sanitize" not in source
    assert "def _unique_name" not in source


def test_step_paths_are_not_ir_names():
    """The STEP front-end keeps '#N' locators; IR names go through the sanitiser."""
    source = (ROOT / "step_import.py").read_text(encoding="utf-8")
    assert "def sanitize" not in source
    assert naming.ir_name_from_path("hull/gear_20#2") == "hull_gear_20"
