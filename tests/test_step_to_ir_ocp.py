"""Ponte ``step-ir/1`` -> ``cad-ir/1``, metà con il kernel (``cadquery-ocp``).

Qui il ponte legge davvero il B-rep: massa, centro di massa e tensore d'inerzia
devono uscire *dai numeri*, non da una formula scritta nel test.  Il fixture e'
l'assemblaggio AP242 generato da ``step_fixtures`` (una piastra 20x10x5 mm e un
perno 4x4x60 mm): le stesse costanti 1000 mm3 e 960 mm3 le verifica gia' la
suite del front-end STEP, quindi qui si puo' controllare il *valore fisico*
contro l'analitico del parallelepipedo ``m (a^2 + b^2) / 12``.

Il modulo si salta con un motivo esplicito quando OCCT non c'e': CI installa le
binding solo nel job ``step-headless``, e uno skip qui significa "non verificato
su questa macchina", mai "verde".  Nessuna dipendenza da ``trimesh``/``scipy``:
l'STL viene riletto come binario e il volume ricavato con il teorema della
divergenza, cosi' il test gira anche nell'extra ``step``.
"""

from __future__ import annotations

import json
import struct
from pathlib import Path

import pytest

pytest.importorskip(
    "OCP",
    reason="OCCT bindings absent: install with `python -m pip install 'cad-to-gazebo[step]'`",
)

from step_fixtures import find_mechanism_step, write_ap242_assembly, write_ap242_multi_solid  # noqa: E402

import ir_schema  # noqa: E402
import naming  # noqa: E402
import step_import  # noqa: E402
import step_to_ir  # noqa: E402
from step_to_ir import (  # noqa: E402
    StepToIrError,
    _mass_properties,
    convert_step,
    load_contract,
)

ROOT = Path(__file__).resolve().parent.parent

#: Geometria del fixture AP242: piastra e perno, in millimetri.
FRAME_MM3 = 20.0 * 10.0 * 5.0
SHAFT_MM3 = 4.0 * 4.0 * 60.0

#: Il contratto di esempio e' un file del repository, non un dict di comodo:
#: e' lo stesso che usa la catena end-to-end del harness, quindi se si rompe
#: si rompe qui e non in CI (o, peggio, in mano a chi lo copia).
CONTRACT_PATH = ROOT / "tests" / "fixtures" / "contract_oscillator.json"
CONTRACT = load_contract(CONTRACT_PATH)


@pytest.fixture(scope="module")
def ap242_step(tmp_path_factory) -> Path:
    return write_ap242_assembly(tmp_path_factory.mktemp("bridge") / "assembly_ap242.step")


@pytest.fixture(scope="module")
def manifest(ap242_step, tmp_path_factory) -> tuple[dict, Path]:
    ws = tmp_path_factory.mktemp("ws")
    document = convert_step(ap242_step, mesh_root=str(ws), contract=CONTRACT,
                            source_document="fixture AP242")
    return document, ws


# --------------------------------------------------------------------------- #
# front-end: le forme escono da import_step, il file si legge una volta sola
# --------------------------------------------------------------------------- #


def test_import_step_can_hand_out_its_shapes(ap242_step):
    shapes: dict = {}
    ir = step_import.import_step(ap242_step, shapes_out=shapes)
    assert len(shapes) == len(ir["parts"]) == 2
    assert all(shape is not None for shape in shapes.values())


def test_import_step_is_unchanged_for_callers_that_do_not_ask(ap242_step):
    assert step_import.import_step(ap242_step) == step_import.import_step(ap242_step)


# --------------------------------------------------------------------------- #
# IR emesso
# --------------------------------------------------------------------------- #


def test_convert_step_emits_a_valid_v1_manifest(manifest):
    document, _ = manifest
    ir_schema.validate(document, source="bridge")
    assert document["meta"]["generator"] == "cad.step_to_ir"
    assert document["meta"]["root"] == "frame"
    assert document["meta"]["source_hash"].startswith("sha256:")
    assert [body["name"] for body in document["bodies"]] == ["frame", "shaft"]
    assert [joint["name"] for joint in document["joints"]] == ["surge_slider"]


def test_placings_carry_the_assembly_offsets_in_metres(manifest):
    document, _ = manifest
    poses = {body["name"]: body["pose"] for body in document["bodies"]}
    assert poses["frame"] == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    assert poses["shaft"][:3] == pytest.approx([0.05, 0.0, 0.0])  # 50 mm


def test_volume_mass_and_inertia_come_from_the_b_rep(manifest):
    document, _ = manifest
    bodies = {body["name"]: body["solids"][0] for body in document["bodies"]}

    frame = bodies["frame"]
    mass = FRAME_MM3 * 1e-9 * 2700.0
    assert frame["reference_volume_m3"] == pytest.approx(FRAME_MM3 * 1e-9)
    assert frame["reference_mass_kg"] == pytest.approx(mass, rel=1e-9)
    # parallelepipedo 20x10x5 mm: com al centro, inerzia m (a^2 + b^2) / 12
    assert frame["reference_com_m"] == pytest.approx([0.01, 0.005, 0.0025], abs=1e-9)
    expected = [mass / 12.0 * (0.010**2 + 0.005**2),
                mass / 12.0 * (0.020**2 + 0.005**2),
                mass / 12.0 * (0.020**2 + 0.010**2)]
    for axis, value in enumerate(expected):
        assert frame["reference_inertia_kg_m2"][axis][axis] == pytest.approx(value, rel=1e-6)
    assert frame["density_source"] == "contract:default"
    assert frame["material"] == {"card": "aluminium", "source": "contract"}

    shaft = bodies["shaft"]
    assert shaft["reference_volume_m3"] == pytest.approx(SHAFT_MM3 * 1e-9)
    assert shaft["reference_mass_kg"] == pytest.approx(SHAFT_MM3 * 1e-9 * 7850.0, rel=1e-9)
    assert shaft["density_source"] == "contract"
    assert shaft["material"]["card"] == "steel"


def test_the_manufacturing_default_density_is_declared_not_hidden(ap242_step, tmp_path):
    document = convert_step(ap242_step, mesh_root=str(tmp_path), contract=None)
    for body in document["bodies"]:
        solid = body["solids"][0]
        assert solid["density_source"] == "step_to_ir:default_steel"
        assert solid["density_kg_m3"] == 7850.0


def test_tessellation_tolerance_is_recorded(ap242_step, tmp_path):
    document = convert_step(ap242_step, mesh_root=str(tmp_path), contract=None,
                            tolerance_mm=0.25)
    assert document["meta"]["tessellation_tolerance_mm"] == 0.25


# --------------------------------------------------------------------------- #
# massa/inerzia del kernel: un cubo di lato noto non mente
# --------------------------------------------------------------------------- #


def test_mass_properties_of_a_1000_mm_cube_match_the_analytics():
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox

    shape = BRepPrimAPI_MakeBox(1000.0, 1000.0, 1000.0).Shape()
    props = _mass_properties(shape, 7850.0, "cubo")
    assert props["volume_m3"] == pytest.approx(1.0, rel=1e-9)
    assert props["mass_kg"] == pytest.approx(7850.0, rel=1e-9)
    assert props["com_m"] == pytest.approx([0.5, 0.5, 0.5], abs=1e-9)
    # un cubo di lato 1 m: I = m / 12 * (1 + 1)
    expected = 7850.0 / 12.0 * 2.0
    for axis in range(3):
        assert props["inertia_kg_m2"][axis][axis] == pytest.approx(expected, rel=1e-9)
        for other in range(3):
            if other != axis:
                assert props["inertia_kg_m2"][axis][other] == pytest.approx(0.0, abs=1e-9)


def test_an_empty_shape_is_refused():
    from OCP.TopoDS import TopoDS_Compound

    with pytest.raises(StepToIrError) as excinfo:
        _mass_properties(TopoDS_Compound(), 7850.0, "vuoto")
    assert "no volume" in str(excinfo.value)


# --------------------------------------------------------------------------- #
# mesh su disco: STL binari, rileggibili, del volume giusto
# --------------------------------------------------------------------------- #


def _stl_triangles(path: Path) -> list:
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    assert len(data) == 84 + 50 * count, "non e' un STL binario ben formato"
    triangles = []
    for index in range(count):
        vertex = struct.unpack_from("<9f", data, 84 + 50 * index + 12)
        triangles.append((vertex[0:3], vertex[3:6], vertex[6:9]))
    return triangles


def _mesh_volume_m3(triangles: list) -> float:
    """Volume della mesh nelle unita' del file (il ponte scrive in metri)."""
    total = 0.0
    for a, b, c in triangles:
        total += (a[0] * (b[1] * c[2] - b[2] * c[1])
                  - a[1] * (b[0] * c[2] - b[2] * c[0])
                  + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
    return abs(total)


def _extent_m(triangles: list) -> list:
    points = [vertex for triangle in triangles for vertex in triangle]
    return [max(p[axis] for p in points) - min(p[axis] for p in points)
            for axis in range(3)]


def test_the_visual_meshes_are_written_where_the_ir_says(manifest):
    document, ws = manifest
    for body in document["bodies"]:
        relative = body["solids"][0]["stl_path"]
        assert relative == naming.visual_mesh_name(body["name"], body["name"])
        target = ws / relative
        assert target.is_file() and target.stat().st_size > 0
        assert body["solids"][0]["visual_mesh"] == "model://" + relative


def test_the_mesh_reproduces_the_solid_it_came_from(manifest):
    document, ws = manifest
    for body, volume_mm3 in zip(document["bodies"], (FRAME_MM3, SHAFT_MM3)):
        triangles = _stl_triangles(ws / body["solids"][0]["stl_path"])
        assert len(triangles) == 12  # un parallelepipedo: 2 triangoli x 6 facce
        # la mesh e' in metri, come il volume dell'IR
        assert _mesh_volume_m3(triangles) == pytest.approx(volume_mm3 * 1e-9, rel=1e-6)
        assert body["solids"][0]["num_vertices"] == 8


def test_the_mesh_is_in_metres_like_the_ir_it_belongs_to(manifest):
    # Regressione: le pose erano in m ma le STL in mm, quindi la geometria
    # usciva 1000 volte piu' grande dell'assemblaggio.  Il frame e' 20x10x5 mm
    # e il suo CoM e' a (10, 5, 2.5) mm, quindi il box va da 0 a 20 mm in x.
    document, ws = manifest
    frame = document["bodies"][0]
    triangles = _stl_triangles(ws / frame["solids"][0]["stl_path"])
    assert _extent_m(triangles) == pytest.approx([0.020, 0.010, 0.005], abs=1e-9)
    xs = [vertex[0] for triangle in triangles for vertex in triangle]
    assert min(xs) == pytest.approx(0.0, abs=1e-9)
    assert max(xs) == pytest.approx(0.020, abs=1e-9)


def test_a_1000_mm_cube_becomes_one_metre_on_disk(tmp_path):
    # Il caso piu' semplice da leggere a occhio: 1000 mm devono diventare 1 m.
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox

    shape = BRepPrimAPI_MakeBox(1000.0, 1000.0, 1000.0).Shape()
    path = tmp_path / "cube.stl"
    step_to_ir._write_stl(shape, path, 0.5, "cubo")
    triangles = _stl_triangles(path)
    assert _extent_m(triangles) == pytest.approx([1.0, 1.0, 1.0], abs=1e-9)
    assert _mesh_volume_m3(triangles) == pytest.approx(1.0, rel=1e-6)


def test_no_meshes_means_no_stl_path_but_a_valid_ir(ap242_step, tmp_path):
    document = convert_step(ap242_step, mesh_root=None, contract=None)
    ir_schema.validate(document)
    for body in document["bodies"]:
        assert "stl_path" not in body["solids"][0]
        assert "visual_mesh" not in body["solids"][0]
    assert not (tmp_path / "meshes").exists()


# --------------------------------------------------------------------------- #
# fail-fast sul modello reale
# --------------------------------------------------------------------------- #


def test_a_ghost_joint_stops_the_conversion_of_the_real_file(ap242_step, tmp_path):
    contract = dict(CONTRACT, joints=[{"name": "ghost", "fc_type": "Fixed",
                                       "parent": "frame",
                                       "child": "spindle"}])
    with pytest.raises(StepToIrError) as excinfo:
        convert_step(ap242_step, mesh_root=str(tmp_path), contract=contract)
    assert "spindle" in str(excinfo.value)


def test_the_ir_can_be_written_to_disk_and_read_back(manifest, tmp_path):
    document, _ = manifest
    path = tmp_path / "cad_manifest.json"
    path.write_text(json.dumps(document, indent=2), encoding="utf-8")
    ir_schema.validate_file(str(path))  # alza se il documento su disco non e' v1
    written = json.loads(path.read_text(encoding="utf-8"))
    assert [body["name"] for body in written["bodies"]] == ["frame", "shaft"]
    assert written["meta"]["generator"] == "cad.step_to_ir"


# --------------------------------------------------------------------------- #
# parti multi-solido: una parte = N solidi OCCT -> N solidi IR
# --------------------------------------------------------------------------- #

#: Volume di ciascuno dei due box del fixture ``write_ap242_multi_solid``.
MULTI_BOX_MM3 = 30.0 * 8.0 * 8.0


@pytest.fixture(scope="module")
def multi_step(tmp_path_factory) -> Path:
    return write_ap242_multi_solid(
        tmp_path_factory.mktemp("multi") / "assembly_multi.step")


@pytest.fixture(scope="module")
def multi_manifest(multi_step, tmp_path_factory) -> tuple[dict, Path]:
    ws = tmp_path_factory.mktemp("ws_multi")
    document = convert_step(multi_step, mesh_root=str(ws), contract=None,
                            source_document="fixture AP242 multi-solido")
    return document, ws


def test_step_ir_counts_two_solids_for_a_compound_part(multi_step):
    shapes: dict = {}
    ir = step_import.import_step(multi_step, shapes_out=shapes)
    by_path = {part["path"]: part for part in ir["parts"]}
    assert by_path["fixture_multi_solid/rail"]["solids"] == 2
    assert by_path["fixture_multi_solid/frame"]["solids"] == 1
    assert len(shapes) == 2


def test_the_bridge_emits_one_ir_solid_per_occt_solid(multi_manifest):
    # Il vecchio ponte emetteva UN solido per parte ma copiava num_solids da
    # step-ir/1: su una parte a 2 solidi la validazione fail-fast moriva con
    # "num_solids=2 ma 1 solidi".  Ora num_solids e' il numero di solidi emessi.
    document, _ = multi_manifest
    ir_schema.validate(document, source="bridge multi-solido")
    bodies = {body["name"]: body for body in document["bodies"]}
    rail = bodies["rail"]
    assert rail["num_solids"] == 2
    assert len(rail["solids"]) == 2
    assert [solid["name"] for solid in rail["solids"]] == ["rail", "rail_2"]
    # Ogni solido porta il proprio volume: non un aggregato schiacciato in uno.
    total = sum(solid["reference_volume_m3"] for solid in rail["solids"])
    assert total == pytest.approx(2 * MULTI_BOX_MM3 * 1e-9, rel=1e-9)


def test_multi_solid_meshes_are_distinct_and_reproduce_each_solid(multi_manifest):
    document, ws = multi_manifest
    rail = next(body for body in document["bodies"] if body["name"] == "rail")
    paths = [solid["stl_path"] for solid in rail["solids"]]
    assert len(set(paths)) == 2, "due solidi non possono condividere la mesh"
    for solid, volume_mm3 in zip(rail["solids"], (MULTI_BOX_MM3, MULTI_BOX_MM3)):
        target = ws / solid["stl_path"]
        assert target.is_file() and target.stat().st_size > 0
        assert solid["visual_mesh"] == "model://" + solid["stl_path"]
        assert _mesh_volume_m3(_stl_triangles(target)) == pytest.approx(
            volume_mm3 * 1e-9, rel=1e-6)


def test_single_solid_parts_keep_the_one_solid_output(multi_manifest):
    document, _ = multi_manifest
    frame = next(body for body in document["bodies"] if body["name"] == "frame")
    assert frame["num_solids"] == 1
    assert [solid["name"] for solid in frame["solids"]] == ["frame"]
    assert frame["solids"][0]["stl_path"] == naming.visual_mesh_name("frame", "frame")


# --------------------------------------------------------------------------- #
# il modello reale: la catena STEP -> IR gira invece di fermarsi al validator
# --------------------------------------------------------------------------- #

REAL_STEP = find_mechanism_step()


@pytest.mark.skipif(
    REAL_STEP is None,
    reason=(
        "il fixture reale full_transmission.step non e' su questa macchina; "
        "imposta CAD_TO_GAZEBO_STEP_FIXTURE per puntare a un checkout"
    ),
)
def test_the_real_transmission_converts_to_a_valid_ir(tmp_path):
    step_ir = step_import.import_step(REAL_STEP)
    document = convert_step(REAL_STEP, mesh_root=str(tmp_path), contract=None)
    ir_schema.validate(document, source="bridge reale")
    assert len(document["bodies"]) == len(step_ir["parts"])
    for body in document["bodies"]:
        assert body["num_solids"] == len(body["solids"]) >= 1
    # Il difetto originale: qua dentro c'e' almeno una parte multi-solido.
    assert any(body["num_solids"] > 1 for body in document["bodies"]), (
        "il fixture reale deve contenere una parte multi-solido, altrimenti il "
        "test non copre il caso C7")

