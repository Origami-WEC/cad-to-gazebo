"""Unit tests for cad.sdf_validate -- the SDF gate.

The embedded backend must reject what a real SDFormat parser rejects and must
NOT reject valid files: the regression fixtures below include a composed model
(`<include>` + `scoped::link` references) and a model with nested `<model>`
children, the two shapes that broke the first version of the checker.
"""

from __future__ import annotations

import os

import pytest
from cad import sdf_validate as sv
from cad.sdf_validate import SdfError

#: A minimal model that must validate clean (errors and warnings alike).
VALID = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="wec">
    <link name="hull">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>10</mass>
        <inertia>
          <ixx>1</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1</iyy><iyz>0</iyz><izz>1</izz>
        </inertia>
      </inertial>
      <collision name="hull_col">
        <geometry><box><size>1 1 1</size></box></geometry>
      </collision>
      <visual name="hull_vis">
        <geometry><box><size>1 1 1</size></box></geometry>
      </visual>
    </link>
    <link name="shaft">
      <inertial>
        <mass>2</mass>
        <inertia>
          <ixx>1</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1</iyy><iyz>0</iyz><izz>1</izz>
        </inertia>
      </inertial>
      <collision name="shaft_col">
        <geometry><cylinder><radius>0.1</radius><length>1</length></cylinder></geometry>
      </collision>
    </link>
    <joint name="pto" type="prismatic">
      <parent>hull</parent>
      <child>shaft</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1.5</lower><upper>1.5</upper></limit>
      </axis>
    </joint>
  </model>
</sdf>
"""

#: Real Gazebo shape: no local links, joints referencing included models.
COMPOSED = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="barrage">
    <include>
      <uri>model://barrage_buoy</uri>
      <name>buoy0</name>
    </include>
    <include>
      <uri>model://barrage_buoy</uri>
      <name>buoy1</name>
    </include>
    <joint type="universal" name="b_joint">
      <parent>buoy0::base_link</parent>
      <child>buoy1::base_link</child>
      <axis><xyz>0 0 1</xyz></axis>
    </joint>
  </model>
</sdf>
"""

#: Real Gazebo shape: parent model with no links of its own, links in children.
NESTED = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="w">
    <model name="parent">
      <model name="ball">
        <link name="body">
          <inertial><mass>15</mass></inertial>
          <collision name="c">
            <geometry><sphere><radius>0.5</radius></sphere></geometry>
          </collision>
        </link>
      </model>
      <model name="box">
        <link name="body">
          <inertial><mass>10</mass></inertial>
          <collision name="c">
            <geometry><box><size>1 1 1</size></box></geometry>
          </collision>
        </link>
      </model>
    </model>
  </world>
</sdf>
"""


def _write(tmp_path, text, name="model.sdf"):
    path = os.path.join(str(tmp_path), name)
    with open(path, "w") as handle:
        handle.write(text)
    return path


# --- clean input -------------------------------------------------------------- #

def test_valid_model_passes(tmp_path):
    path = _write(tmp_path, VALID)
    assert sv.collect_errors(path) == []
    assert sv.collect_warnings(path) == []
    assert sv.validate(path, backend="builtin") == "builtin"


def test_composed_model_with_includes_passes(tmp_path):
    """<include> renames are declared children: scoped refs are not dangling."""
    assert sv.collect_errors(_write(tmp_path, COMPOSED)) == []


def test_model_with_nested_submodels_passes(tmp_path):
    """A parent model may hold no link of its own (links live in children)."""
    assert sv.collect_errors(_write(tmp_path, NESTED)) == []


def test_include_without_uri_is_error(tmp_path):
    broken = COMPOSED.replace("<uri>model://barrage_buoy</uri>", "")
    errors = sv.collect_errors(_write(tmp_path, broken))
    assert any("<uri> mancante" in e for e in errors)


# --- conformance errors ------------------------------------------------------- #

def test_missing_file(tmp_path):
    errors = sv.collect_errors(os.path.join(str(tmp_path), "nope.sdf"))
    assert len(errors) == 1 and "file non trovato" in errors[0]


def test_malformed_xml(tmp_path):
    path = _write(tmp_path, VALID.replace("</joint>", "</jointx>"))
    assert any("XML non ben formato" in e for e in sv.collect_errors(path))


def test_dangling_joint_parent(tmp_path):
    path = _write(tmp_path, VALID.replace("<parent>hull</parent>",
                                          "<parent>ghost</parent>"))
    errors = sv.collect_errors(path)
    assert any("ghost" in e and "non e' un link/frame" in e for e in errors)


def test_joint_to_world_is_allowed(tmp_path):
    path = _write(tmp_path, VALID.replace("<parent>hull</parent>",
                                          "<parent>world</parent>"))
    assert sv.collect_errors(path) == []


def test_duplicate_link_name(tmp_path):
    path = _write(tmp_path, VALID.replace('<link name="shaft">',
                                          '<link name="hull">'))
    assert any("nome duplicato" in e for e in sv.collect_errors(path))


def test_unknown_joint_type(tmp_path):
    path = _write(tmp_path, VALID.replace('type="prismatic"',
                                          'type="hinge"'))
    assert any("non e' un tipo SDFormat valido" in e
               for e in sv.collect_errors(path))


def test_inverted_limit(tmp_path):
    path = _write(tmp_path, VALID.replace("<lower>-1.5</lower>",
                                          "<lower>9</lower>"))
    assert any("lower=9 > upper=1.5" in e for e in sv.collect_errors(path))


def test_non_numeric_inertia(tmp_path):
    path = _write(tmp_path, VALID.replace("<ixx>1</ixx>", "<ixx>abc</ixx>"))
    assert any("non numerico" in e for e in sv.collect_errors(path))


def test_joint_without_child(tmp_path):
    path = _write(tmp_path, VALID.replace("<child>shaft</child>", ""))
    assert any("<child> mancante" in e for e in sv.collect_errors(path))


def test_pose_with_wrong_arity(tmp_path):
    path = _write(tmp_path, VALID.replace("<pose>0 0 0 0 0 0</pose>",
                                          "<pose>0 0 0</pose>"))
    assert any("attesi 6 o 7 valori" in e for e in sv.collect_errors(path))


def test_missing_geometry_in_collision(tmp_path):
    path = _write(tmp_path, VALID.replace(
        "<geometry><cylinder><radius>0.1</radius><length>1</length>"
        "</cylinder></geometry>", ""))
    assert any("nessuna geometria" in e for e in sv.collect_errors(path))


def test_plugin_without_name(tmp_path):
    path = _write(tmp_path, VALID.replace(
        "</model>", '<plugin filename="gz-sim-x"></plugin></model>'))
    errors = sv.collect_errors(path)
    assert any("servono gli attributi filename e name" in e for e in errors)


# --- warnings ---------------------------------------------------------------- #

def test_missing_inertial_is_warning_not_error(tmp_path):
    """SDFormat defaults mass=1 and accepts it; we report, we don't reject."""
    stripped = VALID.replace("""      <inertial>
        <mass>10</mass>
        <inertia>
          <ixx>1</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1</iyy><iyz>0</iyz><izz>1</izz>
        </inertia>
      </inertial>
""", "")
    path = _write(tmp_path, stripped)
    assert sv.collect_errors(path) == []
    assert any("<inertial> mancante" in w for w in sv.collect_warnings(path))
    sv.validate(path, backend="builtin")           # non-strict: passes
    with pytest.raises(SdfError):
        sv.validate(path, backend="builtin", strict=True)


def test_non_positive_mass_is_warning(tmp_path):
    path = _write(tmp_path, VALID.replace("<mass>10</mass>", "<mass>0</mass>"))
    assert sv.collect_errors(path) == []
    assert any("deve essere > 0" in w for w in sv.collect_warnings(path))


def test_unnormalised_axis_is_warning(tmp_path):
    path = _write(tmp_path, VALID.replace("<xyz>0 0 1</xyz>",
                                          "<xyz>0 0 2</xyz>"))
    assert any("non normalizzato" in w for w in sv.collect_warnings(path))


def test_orphan_link_is_warning(tmp_path):
    # parent == child leaves 'shaft' hanging off no joint at all.
    path = _write(tmp_path, VALID.replace("<child>shaft</child>",
                                          "<child>hull</child>"))
    warnings = sv.collect_warnings(path)
    assert any("shaft" in w and "non collegati" in w for w in warnings)
    assert any("parent e child sono lo stesso link" in w for w in warnings)


def test_non_unit_quaternion_is_warning(tmp_path):
    # 7 values after <pose> means quaternion: norm 2 here, not 1.
    path = _write(tmp_path, VALID.replace("<pose>0 0 0 0 0 0</pose>",
                                          "<pose>0 0 0 0 0 0 2</pose>"))
    assert sv.collect_errors(path) == []
    assert any("quaternione non unitario" in w
               for w in sv.collect_warnings(path))


def test_link_without_collision_is_warning(tmp_path):
    path = _write(tmp_path, VALID.replace(
        """      <collision name="shaft_col">
        <geometry><cylinder><radius>0.1</radius><length>1</length></cylinder></geometry>
      </collision>
""", ""))
    assert any("nessun <collision>" in w for w in sv.collect_warnings(path))



def test_static_model_needs_no_inertial_nor_collision(tmp_path):
    """A <static>true</static> model is not integrated: visual-only is valid.

    Regression: the gz-waves water surface is a static, visual-only model; the
    strict gate used to reject it (maritime_ws#90).
    """
    doc = VALID.replace("<model name=\"wec\">",
                        "<model name=\"wec\">\n    <static>true</static>")
    path = _write(tmp_path, doc)
    errors, warnings, _ = sv.check(path, backend="builtin")
    assert errors == []
    assert not any("nessun <collision>" in w for w in warnings)
    assert not any("<inertial> mancante" in w for w in warnings)
    sv.validate(path, backend="builtin", strict=True)


def test_dynamic_model_still_warns_without_collision(tmp_path):
    """The exemption is scoped to static models, not a general silencing."""
    doc = VALID.replace(
        """      <collision name="shaft_col">
        <geometry><cylinder><radius>0.1</radius><length>1</length></cylinder></geometry>
      </collision>
""", "")
    path = _write(tmp_path, doc)
    assert any("nessun <collision>" in w for w in sv.collect_warnings(path))
    with pytest.raises(SdfError):
        sv.validate(path, backend="builtin", strict=True)


# --- backend selection -------------------------------------------------------- #

def test_resolve_backend_auto_follows_availability():
    assert sv.resolve_backend("auto") == ("gz" if sv.gz_available()
                                          else "builtin")


def test_resolve_backend_rejects_unknown():
    with pytest.raises(ValueError):
        sv.resolve_backend("gazebo9")


def test_check_reports_which_backend_ran(tmp_path):
    errors, warnings, ran = sv.check(_write(tmp_path, VALID),
                                     backend="builtin")
    assert (errors, warnings, ran) == ([], [], "builtin")


def test_forced_gz_without_cli_fails_cleanly(tmp_path, monkeypatch):
    path = _write(tmp_path, VALID)
    monkeypatch.setattr(sv.shutil, "which", lambda _exe: None)
    with pytest.raises(SdfError) as excinfo:
        sv.validate(path, backend="gz")
    assert "non e' su PATH" in str(excinfo.value)


def test_gz_verdict_is_used_when_available(tmp_path, monkeypatch):
    path = _write(tmp_path, VALID)

    class _Proc:
        returncode = 1
        stdout = ""
        stderr = "Error: link not found"

    monkeypatch.setattr(sv.shutil, "which",
                        lambda exe: "/usr/bin/gz" if exe == "gz" else None)
    monkeypatch.setattr(sv.subprocess, "run", lambda *a, **k: _Proc())
    with pytest.raises(SdfError) as excinfo:
        sv.validate(path, backend="auto")
    assert "gz sdf -p ha rifiutato" in str(excinfo.value)


# --- CLI ---------------------------------------------------------------------- #

def test_cli_exit_codes(tmp_path, capsys):
    good, bad = _write(tmp_path, VALID, "good.sdf"), _write(
        tmp_path, VALID.replace("<child>shaft</child>", ""), "bad.sdf")
    assert sv.main(["--backend", "builtin", "--quiet", good]) == 0
    assert "good.sdf: ok (builtin)" in capsys.readouterr().out
    assert sv.main(["--backend", "builtin", bad]) == 1
    assert "SDF non valido" in capsys.readouterr().err


def test_cli_strict_fails_on_warning(tmp_path):
    path = _write(tmp_path, VALID.replace("<mass>10</mass>", "<mass>0</mass>"))
    assert sv.main(["--backend", "builtin", "--quiet", path]) == 0
    assert sv.main(["--backend", "builtin", "--strict", "--quiet", path]) == 1


# --- regression against real Gazebo files ------------------------------------- #

_REAL_SDF = "/opt/data/work/maritime_ws/src/gazebo-sim/gz-waves-models"


@pytest.mark.skipif(not os.path.isdir(_REAL_SDF),
                    reason="vendored gz-waves models not available")
def test_real_gazebo_models_have_no_false_positive():
    """Files shipped with Gazebo must never be reported as invalid."""
    checked = 0
    for root, _dirs, files in os.walk(_REAL_SDF):
        for name in files:
            if not name.endswith(".sdf"):
                continue
            checked += 1
            errors = sv.collect_errors(os.path.join(root, name))
            assert errors == [], "{}: {}".format(name, errors)
    assert checked > 0
