"""Ponte STEP -> IR -> URDF su file CAD reali, con almeno un giunto non-fixed.

Tre assiemi diversi del progetto (planetario, cassetta rack, server 2U). Per
ciascuno il contratto in ``tests/fixtures/step_contracts/`` dichiara un giunto
``Revolute``: il ponte non deduce topologia dalla geometria, quindi senza quel
giunto il test non avrebbe nulla da verificare sul lato cinematico.

I file STEP vivono fuori dal repository (cartella ``Dispositivo``). Se mancano
o se OCCT non e' installato il test si salta con il motivo esplicito: uno skip
significa "non verificato su questa macchina", mai "verde".

Il gate ``--strict`` dell'URDF NON e' testato come "passa": con un solo giunto
per assieme il grafo ha molte radici e il gate lo rifiuta, come previsto. Qui si
verifica che il giunto dichiarato arrivi all'URDF non-strict con tipo corretto.
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

pytest.importorskip(
    "OCP",
    reason="OCCT bindings absent: install with `python -m pip install -e '.[step]'`",
)

import ir_schema  # noqa: E402
import urdf_skeleton  # noqa: E402
from step_to_ir import convert_step, load_contract  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
CONTRACTS = ROOT / "tests" / "fixtures" / "step_contracts"

#: Radice dei file CAD reali. Sovrascrivibile con ``CAD_STEP_ROOT``.
STEP_ROOT = Path(os.environ.get("CAD_STEP_ROOT", "CAD_STEP_ROOT-non-impostato"))

CASES = [
    (
        STEP_ROOT / "Mechanism/Transmission/Assemblies/full_transmission.step",
        CONTRACTS / "full_transmission.json",
        "planet_pin_revolute", "sun_gear001", "planet_gear001",
    ),
    (
        STEP_ROOT / "Electronics/IT_Module/server/server/4u-19-rack-case-1/Schroff_20860-611.step",
        CONTRACTS / "schroff_20860_611.json",
        "front_panel_hinge", "side_panel_left", "front_panel",
    ),
    (
        STEP_ROOT / "Electronics/IT_Module/server/server/2U Server.STEP",
        CONTRACTS / "2u_server.json",
        "nauo2_hinge", "nauo1", "nauo2",
    ),
]


@pytest.fixture(scope="module", params=CASES, ids=[c[0].stem for c in CASES])
def case(request, tmp_path_factory):
    step_path, contract_path, joint, parent, child = request.param
    if not step_path.is_file():
        pytest.skip("file STEP assente (imposta CAD_STEP_ROOT): %s" % step_path)
    ws = tmp_path_factory.mktemp("ws")
    contract = load_contract(contract_path)
    manifest = convert_step(step_path, mesh_root=str(ws), contract=contract,
                            source_document=step_path.name)
    return {
        "manifest": manifest,
        "contract": contract,
        "joint": joint,
        "parent": parent,
        "child": child,
    }


def test_ir_is_valid_against_contract(case):
    ir_schema.validate(case["manifest"])


def test_declared_non_fixed_joint_is_in_ir(case):
    joints = {j["name"]: j for j in case["manifest"]["joints"]}
    assert case["joint"] in joints, "il giunto dichiarato nel contratto non esce nell'IR"
    j = joints[case["joint"]]
    assert j["fc_type"] != "Fixed"
    assert j["parent"] == case["parent"]
    assert j["child"] == case["child"]
    bodies = {b["name"] for b in case["manifest"]["bodies"]}
    assert j["parent"] in bodies and j["child"] in bodies


def test_non_fixed_joint_reaches_urdf_with_its_type(case):
    xml = urdf_skeleton.render(case["manifest"])
    tag = '<joint name="%s" type="revolute">' % case["joint"]
    assert tag in xml
    assert '<parent link="%s"/>' % case["parent"] in xml
    assert '<child link="%s"/>' % case["child"] in xml
