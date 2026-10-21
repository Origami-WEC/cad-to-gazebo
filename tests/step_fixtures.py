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


def write_ap242_assembly(
    path: str | Path,
    name: str = "fixture_assembly",
    root_part: str = "frame",
    moving_part: str = "shaft",
) -> Path:
    """Write a small but real AP242 assembly and return its path.

    The assembly is ``name`` containing two named box parts at different
    placements, so the reader has something to name, locate and count. The
    schema is set on the writer *after* it is constructed: OCCT registers the
    ``write.step.schema`` resource parameter lazily, when the first writer
    object exists, and setting it earlier is silently refused.

    ``root_part`` / ``moving_part`` name the two parts. The defaults keep the
    bridge unit tests stable; the end-to-end harness overrides them to
    ``hull`` / ``pto_shaft`` so the very same STEP chain lands on a device the
    ``maritime_ws`` build recognises as a float (its hydrodynamics config wets
    the ``hull`` link), instead of the generic ``frame``/``shaft`` rig.
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

    _add(root_part, 0.0, 0.0, 0.0, (20.0, 10.0, 5.0))
    _add(moving_part, 50.0, 0.0, 0.0, (4.0, 4.0, 60.0))
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


def write_ap242_multi_solid(
    path: str | Path,
    name: str = "fixture_multi_solid",
    body_part: str = "rail",
    single_part: str = "frame",
) -> Path:
    """Write an AP242 assembly with one part that holds **two** OCCT solids.

    It is the reduced, committable stand-in for the real ``full_transmission``
    case: a part whose shape is a ``TopoDS_Compound`` of two disjoint boxes
    (a body + a slider modelled inside the same CAD part).  ``step-ir/1``
    counts two solids for it, so the bridge must emit two ``solids`` entries
    and ``num_solids == 2`` -- the invariant ``num_solids == len(solids)`` that
    the old one-solid-per-part emission violated.

    The assembly also carries a normal one-solid part, so the test can assert
    that the multi-solid path does not change single-solid output.
    """
    from OCP.BRep import BRep_Builder
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Trsf, gp_Vec
    from OCP.IFSelect import IFSelect_ReturnStatus
    from OCP.Interface import Interface_Static
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS_Compound
    from OCP.XCAFApp import XCAFApp_Application
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    target = Path(path)
    application = XCAFApp_Application.GetApplication_s()
    document = TDocStd_Document(TCollection_ExtendedString("XmlXCAF"))
    application.NewDocument(TCollection_ExtendedString("XmlXCAF"), document)
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())

    root = shapes.NewShape()
    TDataStd_Name.Set_s(root, TCollection_ExtendedString(name))

    # Un solo oggetto CAD (una parte) ma due solidi distinti: e' il caso che
    # in full_transmission.step fa scattare "num_solids=2 ma 1 solidi".
    # I due box sono disgiunti in Y: due TopoDS_Solid reali dentro lo stesso
    # TopoDS_Compound, cosi' il conteggio solidi di step-ir/1 e' 2.
    from OCP.gp import gp_Pnt

    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    builder.Add(compound, BRepPrimAPI_MakeBox(30.0, 8.0, 8.0).Shape())
    builder.Add(compound, BRepPrimAPI_MakeBox(gp_Pnt(0.0, 20.0, 0.0),
                                              30.0, 8.0, 8.0).Shape())
    definition = shapes.AddShape(compound, False)
    TDataStd_Name.Set_s(definition, TCollection_ExtendedString(body_part))
    component = shapes.AddComponent(root, definition, TopLoc_Location())
    TDataStd_Name.Set_s(component, TCollection_ExtendedString(body_part))

    single = shapes.AddShape(BRepPrimAPI_MakeBox(20.0, 10.0, 5.0).Shape(), False)
    TDataStd_Name.Set_s(single, TCollection_ExtendedString(single_part))
    transform = gp_Trsf()
    transform.SetTranslation(gp_Vec(50.0, 0.0, 0.0))
    single_component = shapes.AddComponent(
        root, single, TopLoc_Location(transform))
    TDataStd_Name.Set_s(single_component, TCollection_ExtendedString(single_part))

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
