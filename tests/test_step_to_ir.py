"""Ponte ``step-ir/1`` -> ``cad-ir/1``: la meta' che non ha bisogno del kernel.

Il ponte e' diviso in due: ``plan()`` decide *cosa* emettere (nomi IR, densita',
materiali, parti escluse, giunti verificati) e non tocca la geometria, quindi
gira anche senza OCCT; ``convert()`` ci aggiunge massa, inerzia e mesh, e per
questo pretende le forme.  Qui si verifica ``plan()`` e tutto il resto che il
kernel non lo vede nemmeno: lettura del contratto, matematica delle pose,
rifiuto delle pose impossibili, argomenti della CLI.

La meta' con il kernel -- i numeri di massa/inerzia di un box noto -- sta in
``tests/test_step_to_ir_ocp.py``.
"""

import json
import math

import pytest

import ir_schema
import step_to_ir
from step_to_ir import (
    DEFAULT_DENSITY_KG_M3,
    StepToIrError,
    convert,
    load_contract,
    plan,
)

# --------------------------------------------------------------------------- #
# Aiutanti
# --------------------------------------------------------------------------- #


def _part(name, **extra):
    """Una parte come le scrive ``step_import`` (trasformazione identita')."""
    part = {
        "name": name,
        "path": name,
        "solids": 1,
        "volume": 1000.0,
        "bbox": {"min": [0.0, 0.0, 0.0], "max": [20.0, 10.0, 5.0]},
        "transform": _transform(),
    }
    part.update(extra)
    return part


def _transform(translation=(0.0, 0.0, 0.0), rotation=((1, 0, 0), (0, 1, 0), (0, 0, 1))):
    """Matrice 4x4 row-major come la esporta OCCT (ultima colonna = posizione)."""
    rows = [list(rotation[row]) + [translation[row]] for row in range(3)]
    return rows[0] + rows[1] + rows[2] + [0.0, 0.0, 0.0, 1.0]


def _step_ir(parts=None, **source):
    return {
        "schema": "cad-to-gazebo/step-ir/1",
        "generator": {"module": "cad.step_import", "format": "AP242"},
        "source": {"path": "device.step", "sha256": "deadbeef",
                   "file_description": "Origami oscillator", **source},
        "parts": parts if parts is not None else [_part("frame"), _part("shaft")],
    }


def _contract(**overrides):
    contract = {"device_type": "oscillating_surge_converter", "root": "frame"}
    contract.update(overrides)
    return contract


# --------------------------------------------------------------------------- #
# Contratto: si legge una volta, e a chiavi chiuse
# --------------------------------------------------------------------------- #


def test_a_minimal_contract_is_read(tmp_path):
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(_contract()), encoding="utf-8")
    assert load_contract(path)["device_type"] == "oscillating_surge_converter"


def test_an_unknown_contract_key_is_a_typo_not_a_feature(tmp_path):
    path = tmp_path / "contract.json"
    path.write_text(json.dumps({"joinths": []}), encoding="utf-8")
    with pytest.raises(StepToIrError, match="joinths"):
        load_contract(path)


def test_an_unknown_key_inside_a_part_entry_is_refused(tmp_path):
    path = tmp_path / "contract.json"
    path.write_text(json.dumps({"parts": {"shaft": {"dinesty": 1}}}), encoding="utf-8")
    with pytest.raises(StepToIrError, match="dinesty"):
        load_contract(path)


def test_skip_must_be_a_list(tmp_path):
    path = tmp_path / "contract.json"
    path.write_text(json.dumps({"skip": "shaft"}), encoding="utf-8")
    with pytest.raises(StepToIrError, match="skip"):
        load_contract(path)


def test_a_missing_contract_file_is_reported_by_name(tmp_path):
    with pytest.raises(StepToIrError, match="missing.json"):
        load_contract(tmp_path / "missing.json")


def test_invalid_json_names_the_file(tmp_path):
    path = tmp_path / "contract.json"
    path.write_text("{not json", encoding="utf-8")
    with pytest.raises(StepToIrError, match="contract.json"):
        load_contract(path)


# --------------------------------------------------------------------------- #
# Densita': un numero dichiarato, mai indovinato
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("value", [0, -1, "steel", None])
def test_a_nonsensical_density_is_refused(value):
    with pytest.raises(StepToIrError):
        plan(_step_ir(), contract={"density_kg_m3": value})


def test_without_a_declared_density_steel_is_used_and_flagged():
    # 7850 kg/m3 e' una scelta, non un fatto: chi la usa deve poterlo vedere.
    assert plan(_step_ir())["parts"][0]["density_kg_m3"] == DEFAULT_DENSITY_KG_M3
    assert plan(_step_ir())["parts"][0]["density_source"] == "step_to_ir:default_steel"


def test_the_contract_default_wins_over_steel():
    blueprint = plan(_step_ir(), contract=_contract(density_kg_m3=2700.0))
    assert blueprint["parts"][0]["density_kg_m3"] == 2700.0
    assert blueprint["parts"][0]["density_source"] == "contract:default"


def test_a_density_without_a_card_does_not_become_steel_by_habit():
    # 2700 kg/m3 con la scheda "acciaio" sarebbe una contraddizione: la scheda
    # resta dichiaratamente ignota ('card' e' obbligatorio nello schema).
    blueprint = plan(_step_ir(), contract=_contract(density_kg_m3=2700.0))
    assert blueprint["parts"][0]["material_card"] == "unspecified"
    assert blueprint["parts"][0]["material_source"] == "unspecified"


def test_a_declared_card_brings_its_own_density():
    # Una sola fonte per la coppia (densita', scheda): la tabella di
    # material_props.  Dichiarare la scheda *e'* dichiarare la densita'.
    blueprint = plan(_step_ir(), contract=_contract(material_card="aluminium"))
    assert blueprint["parts"][0]["material_card"] == "aluminium"
    assert blueprint["parts"][0]["density_kg_m3"] == 2700.0
    assert blueprint["parts"][0]["density_source"] == "material_card"
    assert blueprint["parts"][0]["material_source"] == "contract"

    blueprint = plan(_step_ir(), material_card="aluminium")
    assert blueprint["parts"][0]["density_kg_m3"] == 2700.0
    assert blueprint["parts"][0]["material_source"] == "cli"

    # Una densita' scritta a mano vince sulla tabella della scheda.
    blueprint = plan(_step_ir(), contract=_contract(material_card="aluminium"),
                     density=1234.0)
    assert blueprint["parts"][0]["density_kg_m3"] == 1234.0
    assert blueprint["parts"][0]["density_source"] == "cli"


def test_a_card_without_a_known_density_is_an_error_not_a_guess():
    with pytest.raises(StepToIrError, match="titanium"):
        plan(_step_ir(), contract=_contract(material_card="titanium"))
    blueprint = plan(_step_ir(), contract=_contract(material_card="titanium",
                                                    density_kg_m3=4500.0))
    assert blueprint["parts"][0]["density_kg_m3"] == 4500.0


def test_a_per_part_density_wins_over_the_default():
    blueprint = plan(_step_ir(), contract=_contract(
        density_kg_m3=2700.0,
        parts={"shaft": {"density_kg_m3": 7850.0, "material_card": "steel"}},
    ))
    by_name = {entry["body"]: entry for entry in blueprint["parts"]}
    assert by_name["frame"]["density_kg_m3"] == 2700.0
    assert by_name["shaft"]["density_kg_m3"] == 7850.0
    assert by_name["shaft"]["material_card"] == "steel"
    assert by_name["shaft"]["density_source"] == "contract"


def test_the_cli_density_beats_the_contract_default():
    blueprint = plan(_step_ir(), contract=_contract(density_kg_m3=2700.0),
                     density=1000.0)
    entry = blueprint["parts"][0]
    assert entry["density_kg_m3"] == 1000.0
    assert entry["density_source"] == "cli"


def test_a_per_part_contract_entry_beats_the_cli():
    blueprint = plan(_step_ir(), density=1000.0,
                     contract={"parts": {"frame": {"density_kg_m3": 7850.0},
                                         "shaft": {"density_kg_m3": 7850.0}}})
    assert all(entry["density_source"] == "contract" for entry in blueprint["parts"])


def test_a_nonsensical_tolerance_is_refused():
    with pytest.raises(StepToIrError, match="tolerance"):
        plan(_step_ir(), tolerance_mm=0)


# --------------------------------------------------------------------------- #
# Nomi: gli stessi del resto della catena
# --------------------------------------------------------------------------- #


def test_body_names_follow_the_shared_sanitiser():
    blueprint = plan(_step_ir(parts=[_part("Main Frame"), _part("Power Shaft")]))
    assert [entry["body"] for entry in blueprint["parts"]] == ["main_frame", "power_shaft"]


def test_two_parts_with_the_same_name_do_not_collide():
    blueprint = plan(_step_ir(parts=[_part("Shaft"), _part("shaft")]))
    names = [entry["body"] for entry in blueprint["parts"]]
    assert len(set(names)) == len(names), names


def test_the_label_keeps_the_cad_name_for_traceability():
    entry = plan(_step_ir(parts=[_part("Main Frame")]))["parts"][0]
    assert entry["label"] == "Main Frame"
    assert entry["body"] == "main_frame"


def test_part_keys_accept_the_raw_name_and_the_sanitised_one():
    part = _part("Main Frame")
    keys = step_to_ir._part_keys(part, step_to_ir.naming.sanitize("Main Frame"))
    assert "Main Frame" in keys and "main_frame" in keys


def test_a_part_entry_can_skip_itself():
    blueprint = plan(_step_ir(), contract={"parts": {"shaft": {"skip": True}}})
    assert [entry["body"] for entry in blueprint["parts"]] == ["frame"]
    assert blueprint["skipped"] == ["shaft"]


def test_skip_accepts_the_sanitised_name_too():
    blueprint = plan(_step_ir(parts=[_part("Main Frame"), _part("Shaft")]),
                     contract={"skip": ["main_frame"]})
    assert [entry["label"] for entry in blueprint["parts"]] == ["Shaft"]


def test_skipping_every_part_is_refused():
    with pytest.raises(StepToIrError, match="no body"):
        plan(_step_ir(), contract={"skip": ["frame", "shaft", "device.step"]})


# --------------------------------------------------------------------------- #
# Giunti: dal contratto, verificati contro i body sopravvissuti
# --------------------------------------------------------------------------- #


def _slider(**overrides):
    joint = {
        "name": "Slider Shaft", "fc_type": "Slider", "parent": "frame",
        "child": "shaft", "axis": [1, 0, 0], "limits": {"lower": -0.1, "upper": 0.4},
    }
    joint.update(overrides)
    return joint


def test_joints_come_from_the_contract_and_are_renamed():
    blueprint = plan(_step_ir(), contract=_contract(joints=[_slider()]))
    assert blueprint["joints_declared"] is True
    joint = blueprint["joints"][0]
    assert joint["name"] == "slider_shaft"
    assert joint["label"] == "Slider Shaft"
    assert (joint["parent"], joint["child"]) == ("frame", "shaft")
    assert joint["fc_type"] == "Slider"


def test_the_bridge_completes_a_joint_into_the_v1_dialect():
    # Un giunto v1 pretende activated, limiti completi e unita': lo Slider le
    # ha deducibili (metri), quindi il contratto non deve ripeterle.
    joint = plan(_step_ir(), contract=_contract(joints=[_slider()]))["joints"][0]
    assert joint["activated"] is True
    assert joint["limits"] == {
        "lower": -0.1, "upper": 0.4, "lower_enabled": True,
        "upper_enabled": True, "unit": "m", "offset_m": 0.0,
    }


def test_a_revolute_joint_gets_radians():
    joint = plan(_step_ir(), contract=_contract(
        joints=[_slider(fc_type="Revolute", limits={"lower": 0.0, "upper": 1.57})]
    ))["joints"][0]
    assert joint["limits"]["unit"] == "rad"


def test_an_unknown_joint_type_needs_its_unit_declared():
    with pytest.raises(StepToIrError, match="unit"):
        plan(_step_ir(), contract=_contract(joints=[_slider(fc_type="Worm")]))
    joint = plan(_step_ir(), contract=_contract(
        joints=[_slider(fc_type="Worm", limits={"lower": 0.0, "upper": 1.0, "unit": "rad"})]
    ))["joints"][0]
    assert joint["limits"]["unit"] == "rad"


def test_a_joint_without_limits_is_refused():
    # Il ponte non inventa una corsa: i numeri del movimento li dichiara chi
    # progetta, e senza di essi non c'e' un giunto v1 valido.
    with pytest.raises(StepToIrError, match="limits"):
        plan(_step_ir(), contract=_contract(joints=[_slider(limits=None)]))


def test_inverted_limits_are_refused():
    with pytest.raises(StepToIrError, match="lower"):
        plan(_step_ir(), contract=_contract(
            joints=[_slider(limits={"lower": 0.4, "upper": -0.1})]
        ))


def test_a_movable_joint_without_an_axis_is_refused():
    with pytest.raises(StepToIrError, match="axis"):
        plan(_step_ir(), contract=_contract(joints=[_slider(axis=None)]))


def test_a_zero_axis_is_refused():
    with pytest.raises(StepToIrError, match="axis"):
        plan(_step_ir(), contract=_contract(joints=[_slider(axis=[0, 0, 0])]))


def test_a_fixed_joint_needs_no_axis_and_is_carried_deactivated_on_request():
    joint = plan(_step_ir(), contract=_contract(
        joints=[_slider(fc_type="Fixed", axis=None,
                        limits={"lower": 0.0, "upper": 0.0}, activated=False)]
    ))["joints"][0]
    assert joint["axis"] == [1.0, 0.0, 0.0]
    assert joint["activated"] is False


def test_a_joint_without_a_type_is_refused():
    with pytest.raises(StepToIrError, match="fc_type"):
        plan(_step_ir(), contract=_contract(
            joints=[{"name": "slider", "parent": "frame", "child": "shaft"}]
        ))


def test_an_unknown_joint_key_is_a_typo_not_a_feature():
    with pytest.raises(StepToIrError, match="limtis"):
        plan(_step_ir(), contract=_contract(joints=[_slider(limtis={})]))


def test_a_joint_to_a_body_that_does_not_exist_is_refused():
    with pytest.raises(StepToIrError, match="piston"):
        plan(_step_ir(), contract=_contract(joints=[_slider(child="piston")]))


def test_a_joint_to_a_skipped_part_is_refused():
    # Saltare una parte e lasciare un giunto che la nomina e' un contratto
    # incoerente: deve fallire qui, non a valle.
    with pytest.raises(StepToIrError, match="shaft"):
        plan(_step_ir(), contract=_contract(skip=["shaft"], joints=[_slider()]))


def test_without_joints_in_the_contract_the_ir_has_no_topology():
    blueprint = plan(_step_ir(), contract=_contract())
    assert blueprint["joints"] == []
    assert blueprint["joints_declared"] is False


def test_root_is_sanitised_like_a_body_name():
    blueprint = plan(_step_ir(parts=[_part("Main Frame"), _part("Shaft")]),
                     contract={"root": "Main Frame"})
    assert blueprint["root"] == "main_frame"
    assert blueprint["meta"]["root"] == "main_frame"


def test_an_unknown_root_is_refused():
    with pytest.raises(StepToIrError, match="hull"):
        plan(_step_ir(), contract={"root": "hull"})


# --------------------------------------------------------------------------- #
# meta: chiuse come lo schema, cosi' non si inventano campi
# --------------------------------------------------------------------------- #


def test_meta_only_carries_documented_keys():
    meta = plan(_step_ir())["meta"]
    declared = set(ir_schema.load_schema()["$defs"]["meta"]["properties"])
    assert set(meta) <= declared, set(meta) - declared


def test_meta_points_back_at_the_document_it_came_from():
    meta = plan(_step_ir(), now="2026-11-05T09:00:00+00:00")["meta"]
    assert meta["generator"] == "cad.step_to_ir"
    assert meta["emitter_version"] == ir_schema.emitter_version()
    assert meta["source_hash"] == "sha256:deadbeef"
    assert meta["source_document"] == "Origami oscillator"
    assert meta["source_file"] == "device.step"
    assert meta["units"] == "SI (m, kg, rad)"
    assert meta["created"] == "2026-11-05T09:00:00+00:00"


def test_the_tessellation_tolerance_is_recorded():
    assert plan(_step_ir(), tolerance_mm=1.5)["meta"]["tessellation_tolerance_mm"] == 1.5


def test_a_document_that_is_not_a_step_ir_is_refused():
    with pytest.raises(StepToIrError, match="cad-ir/1"):
        plan({"schema": "cad-to-gazebo/cad-ir/1", "parts": [_part("frame")]})


def test_a_step_ir_without_parts_is_refused():
    with pytest.raises(StepToIrError, match="no part"):
        plan(_step_ir(parts=[]))


# --------------------------------------------------------------------------- #
# Pose: millimetri in ingresso, metri e quaternione in uscita
# --------------------------------------------------------------------------- #


def test_the_identity_placement_is_the_identity_pose():
    pose = step_to_ir._pose_from_transform(_transform(), "part 'frame'")
    assert pose == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]


def test_a_millimetre_offset_becomes_metres():
    pose = step_to_ir._pose_from_transform(_transform((50.0, 0.0, 25.0)), "part")
    assert pose[:3] == [0.05, 0.0, 0.025]


def test_a_quarter_turn_about_z_is_a_90_degree_quaternion():
    pose = step_to_ir._pose_from_transform(
        _transform(rotation=((0, -1, 0), (1, 0, 0), (0, 0, 1))), "part")
    x, y, z, w = pose[3:]
    assert (x, y) == pytest.approx((0.0, 0.0), abs=1e-9)
    assert z == pytest.approx(math.sqrt(0.5), abs=1e-9)
    assert w == pytest.approx(math.sqrt(0.5), abs=1e-9)


def test_the_quaternion_comes_back_normalised():
    pose = step_to_ir._pose_from_transform(
        _transform(rotation=((0, -1, 0), (1, 0, 0), (0, 0, 1))), "part")
    assert sum(value * value for value in pose[3:]) == pytest.approx(1.0, abs=1e-12)


def test_a_scaled_occurrence_is_refused():
    with pytest.raises(StepToIrError, match="rigid"):
        step_to_ir._pose_from_transform(
            _transform(rotation=((2, 0, 0), (0, 2, 0), (0, 0, 2))), "part")


def test_a_mirrored_occurrence_is_refused():
    # Righe ortonormali ma determinante -1: non e' una rotazione, e' uno specchio.
    with pytest.raises(StepToIrError, match="handed"):
        step_to_ir._pose_from_transform(
            _transform(rotation=((-1, 0, 0), (0, 1, 0), (0, 0, 1))), "part")


def test_a_transform_of_the_wrong_length_is_refused():
    with pytest.raises(StepToIrError, match="16"):
        step_to_ir._pose_from_transform([1.0, 0.0, 0.0], "part")


# --------------------------------------------------------------------------- #
# La geometria non si inventa: senza kernel non si emette un IR valido
# --------------------------------------------------------------------------- #


def test_convert_refuses_to_emit_a_solid_without_the_kernel():
    # ``reference_mass_kg`` e' obbligatorio nello schema v1: senza il B-rep la
    # massa non c'e', e un IR senza massa non e' un IR -- meglio un errore.
    with pytest.raises(StepToIrError, match="shape"):
        convert(_step_ir(), shapes={})


def test_convert_refuses_a_shape_that_is_missing_for_one_part_only():
    with pytest.raises(StepToIrError, match="shaft"):
        convert(_step_ir(), shapes={"frame": object()})


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #


def test_the_cli_wants_a_step_file():
    with pytest.raises(SystemExit):
        step_to_ir.build_parser().parse_args([])


def test_the_cli_can_turn_the_meshes_off():
    args = step_to_ir.build_parser().parse_args(["--step", "device.step", "--no-meshes"])
    assert args.no_meshes is True
    assert args.ws is None
    assert args.out is None


def test_plan_is_part_of_the_public_surface():
    assert "plan" in step_to_ir.__all__
