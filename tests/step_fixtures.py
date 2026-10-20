"""STEP fixtures for the headless import tests.

The fixtures are generated at test time through the OCCT writer instead of
being committed as blobs: a 30-line builder is reviewable, a 40 kB STEP file
is not, and the generated file is a *genuine* AP242 file (the writer switches
schema, it does not relabel an AP214 body).

The real end-to-end fixture -- the FreeCAD-authored transmission -- cannot
live here: it is 36 MB. ``find_mechanism_step`` locates it on hosts that have
the ``Mechanism`` repository checked out next to this one, and the tests skip
explicitly (never silently) when it is absent.
"""

from __future__ import annotations

import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

#: Canonical relative location of the real STEP export inside ``Mechanism``.
MECHANISM_STEP_RELPATH = "Transmission/Assemblies/full_transmission.step"

#: Environment override, useful in CI and in a ``maritime_ws`` checkout where
#: the sibling repository is not next to this one.
FIXTURE_ENV = "CAD_TO_GAZEBO_STEP_FIXTURE"


def find_mechanism_step() -> Path | None:
    """Return the real ``full_transmission.step`` if this host has it."""
    override = os.environ.get(FIXTURE_ENV)
    candidates = []
    if override:
        candidates.append(Path(override))
    candidates.append(ROOT / "tests" / "fixtures" / "full_transmission.step")
    for anchor in (ROOT, ROOT.parent, ROOT.parent.parent):
        candidates.append(anchor / "Mechanism" / MECHANISM_STEP_RELPATH)
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return None


def write_ap242_assembly(path: str | Path, name: str = "fixture_assembly") -> Path:
    """Write a small but real AP242 assembly and return its path.

    The assembly is ``name`` containing two named box parts at different
    placements, so the reader has something to name, locate and count. The
    schema is set on the writer *after* it is constructed: OCCT registers the
    ``write.step.schema`` resource parameter lazily, when the first writer
    object exists, and setting it earlier is silently refused.
    """
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Trsf, gp_Vec
    from OCP.IFSelect import IFSelect_ReturnStatus
    from OCP.Interface import Interface_Static
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFApp import XCAFApp_Application
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    target = Path(path)
    application = XCAFApp_Application.GetApplication_s()
    document = TDocStd_Document(TCollection_ExtendedString("XmlXCAF"))
    application.NewDocument(TCollection_ExtendedString("XmlXCAF"), document)
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())

    root = shapes.NewShape()
    TDataStd_Name.Set_s(root, TCollection_ExtendedString(name))

    def _add(part_name: str, dx: float, dy: float, dz: float, size) -> None:
        definition = shapes.AddShape(BRepPrimAPI_MakeBox(*size).Shape(), False)
        TDataStd_Name.Set_s(definition, TCollection_ExtendedString(part_name))
        transform = gp_Trsf()
        transform.SetTranslation(gp_Vec(dx, dy, dz))
        component = shapes.AddComponent(root, definition, TopLoc_Location(transform))
        TDataStd_Name.Set_s(component, TCollection_ExtendedString(part_name))

    _add("frame", 0.0, 0.0, 0.0, (20.0, 10.0, 5.0))
    _add("shaft", 50.0, 0.0, 0.0, (4.0, 4.0, 60.0))
    shapes.UpdateAssemblies()

    writer = STEPCAFControl_Writer()
    assert Interface_Static.SetCVal_s("write.step.schema", "AP242DIS"), (
        "this OCCT build does not accept the AP242 schema switch"
    )
    writer.SetNameMode(True)
    writer.SetColorMode(True)
    assert writer.Transfer(document)
    status = writer.Write(str(target))
    assert status == IFSelect_ReturnStatus.IFSelect_RetDone, status
    return target
