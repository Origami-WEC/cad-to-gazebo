"""Headless STEP (AP242 / AP214 / AP203) import -> intermediate representation.

This module is the **CAD front-end** of the pipeline: it reads a STEP exchange
file and produces the neutral IR that the rest of the repository consumes
(hull derivation, material resolution, SDF emission). It replaces the
``freecadcmd``-based ``freecad_exporter.py`` path for the STEP case, and it is
the only supported way to get a named, located part tree out of a STEP file
without a CAD GUI.

Design rules
------------

* **Headless.** Nothing here imports FreeCAD, FreeCADGui or any windowing
  toolkit. The geometry kernel is Open CASCADE (OCCT) through the ``OCP``
  bindings shipped by ``cadquery-ocp``; it needs no display and no X server.
* **Lazy OCP.** ``import cad.step_import`` (or ``import step_import``) never
  touches ``OCP``. The kernel is imported on first use, so the package stays
  importable, lint-able and unit-testable on hosts that do not ship the
  kernel. Missing kernel => :class:`StepImportError` with the install hint.
* **Names come from the file, not from positions.** Every assembly and every
  part gets a name derived from the STEP product/occurrence names, addressed
  by its path from the root (``root/shaft_0/gear_20``). Where a name repeats
  among the siblings of the same parent, a ``#2``, ``#3``, ... suffix is
  appended in document order. The result is deterministic and stable across
  runs and across kernel upgrades; it never depends on a traverse index.
* **Read-only.** The module never writes STEP or any CAD format; it only
  reads, and optionally dumps the IR as JSON.

Protocollo STEP supportato
---------------------------

AP242, AP214 and AP203 are all handled by the same reader; the IR records
which schema the file declares (``source.application_protocol``). Files that
declare AP214 (``AUTOMOTIVE_DESIGN``) are accepted and reported as AP214 --
the import does not silently relabel them as AP242.

Unita' di lunghezza
-------------------

OCCT reads into its "cascade" unit (millimetre) and converts the file's own
length unit on the fly, so every coordinate in the IR is in millimetres.
``units`` in the IR records that contract.

IR shape (``schema == "cad-to-gazebo/step-ir/1"``)
--------------------------------------------------

::

    {
      "schema": "cad-to-gazebo/step-ir/1",
      "generator": {"tool": ..., "version": ..., "kernel": ..., "headless": true},
      "source":    {"path", "sha256", "bytes", "file_schema",
                    "application_protocol", "format"},
      "units":     {"length": "MM", "source": ...},
      "counts":    {"assemblies", "parts", "solids", "faces"},
      "assemblies": [{"name", "path", "parent", "depth", "transform",
                      "children": [path, ...]}],
      "parts":      [{"name", "path", "parent", "depth", "transform",
                      "bbox": {"min": [x, y, z], "max": [x, y, z]},
                      "volume", "faces", "solids", "color"}]
    }

``transform`` is a row-major 4x4 matrix (16 floats) mapping the node's local
frame to the assembly frame; translation lives in columns 4 of rows 1-3.
``bbox`` is the world-space bounding box of the part, in millimetres.

CLI
---

::

    python -m cad.step_import --step <file.step> --out <ir.json>

``--out`` is optional: without it the IR is written to stdout. Exit status is
0 on success, 1 on a :class:`StepImportError` (with the reason on stderr).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

__all__ = [
    "IR_SCHEMA",
    "StepImportError",
    "detect_protocol",
    "import_step",
    "main",
    "sha256_file",
]

IR_SCHEMA = "cad-to-gazebo/step-ir/1"
IR_VERSION = "0.2.0"

#: Hard cap on assembly nesting. STEP files from real CAD trees are 5-8 deep;
#: anything past this is either a malicious or a corrupted tree, and the cap
#: keeps the recursion bounded.
MAX_DEPTH = 64

#: Modules that would drag a GUI (or the FreeCAD host) into the import path.
#: ``tests/test_step_import.py`` asserts none of them appear in this source.
_GUI_MODULES = (
    "FreeCAD",
    "FreeCADGui",
    "PySide",
    "PySide2",
    "PySide6",
    "PyQt5",
    "PyQt6",
    "shiboken",
)

_STEP_MAGIC = "ISO-10303-21"
_HEADER_SCAN_BYTES = 65536
_SCHEMA_RE = re.compile(r"FILE_SCHEMA\s*\(\s*\(\s*'([^']*)'", re.IGNORECASE)
_DESCRIPTION_RE = re.compile(r"FILE_DESCRIPTION\s*\(\s*\(\s*'([^']*)'", re.IGNORECASE)

_OCP_CACHE: dict | None = None


class StepImportError(RuntimeError):
    """Raised for every failure of the STEP import (bad file, bad kernel, ...)."""


# --------------------------------------------------------------------------- #
# header / protocol detection (pure Python, no kernel)
# --------------------------------------------------------------------------- #


def sha256_file(path: str | Path, chunk: int = 1 << 20) -> str:
    """Return the SHA-256 of ``path`` (the IR stamps the source for caching)."""
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        while True:
            block = handle.read(chunk)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def read_header(path: str | Path, limit: int = _HEADER_SCAN_BYTES) -> str:
    """Return the first ``limit`` bytes of ``path`` decoded as text.

    The STEP ``HEADER`` section is ASCII by standard, so a lenient decode is
    safe and avoids failing on stray bytes in the ``DATA`` section.
    """
    try:
        with open(path, "rb") as handle:
            raw = handle.read(limit)
    except FileNotFoundError as exc:
        raise StepImportError("STEP file not found: %s" % path) from exc
    except OSError as exc:
        raise StepImportError("cannot read STEP file %s: %s" % (path, exc)) from exc
    return raw.decode("utf-8", errors="replace")


def classify_schema(schema: str) -> str:
    """Map a ``FILE_SCHEMA`` string onto ``AP242`` / ``AP214`` / ``AP203``.

    ``FILE_SCHEMA`` carries the application protocol indirectly: the AP242
    long-form schema is named ``AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_*``,
    AP214 ships as ``AUTOMOTIVE_DESIGN``, AP203 as ``CONFIG_CONTROL_DESIGN``.
    An unrecognised schema is reported as ``AP_UNKNOWN`` rather than guessed.
    """
    normalised = (schema or "").strip().lower()
    if not normalised:
        return "AP_UNKNOWN"
    if "242" in normalised or "managed_model_based" in normalised:
        return "AP242"
    if "214" in normalised or "automotive_design" in normalised:
        return "AP214"
    if "203" in normalised or "config_control_design" in normalised:
        return "AP203"
    return "AP_UNKNOWN"


def detect_protocol(path: str | Path) -> dict:
    """Validate the STEP header and report the declared schema.

    Raises :class:`StepImportError` when the file is not a STEP exchange file
    at all, which gives a clearer message than the kernel's own parser dump.
    """
    text = read_header(path)
    if _STEP_MAGIC not in text[:1024]:
        raise StepImportError(
            "not a STEP exchange file (missing %r in the header): %s" % (_STEP_MAGIC, path)
        )
    match = _SCHEMA_RE.search(text)
    if match is None:
        raise StepImportError("STEP header has no readable FILE_SCHEMA entry: %s" % path)
    schema = match.group(1).strip()
    description = _DESCRIPTION_RE.search(text)
    return {
        "file_schema": schema,
        "application_protocol": classify_schema(schema),
        "file_description": description.group(1) if description else None,
    }


# --------------------------------------------------------------------------- #
# OCP kernel (lazy)
# --------------------------------------------------------------------------- #


def _ocp() -> dict:
    """Import the OCCT kernel bindings on first use and cache the symbols.

    Kept behind a function so that importing this module never requires the
    kernel (the package must stay importable on a lint-only host).
    """
    global _OCP_CACHE
    if _OCP_CACHE is not None:
        return _OCP_CACHE

    try:
        from OCP.Bnd import Bnd_Box
        from OCP.BRepBndLib import BRepBndLib
        from OCP.BRepGProp import BRepGProp
        from OCP.collections import Sequence_TDF_Label
        from OCP.GProp import GProp_GProps
        from OCP.IFSelect import IFSelect_ReturnStatus
        from OCP.Interface import Interface_Static
        from OCP.STEPCAFControl import STEPCAFControl_Reader
        from OCP.TCollection import TCollection_ExtendedString
        from OCP.TDataStd import TDataStd_Name
        from OCP.TDF import TDF_Label
        from OCP.TDocStd import TDocStd_Document
        from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopLoc import TopLoc_Location
        from OCP.XCAFApp import XCAFApp_Application
        from OCP.XCAFDoc import (
            XCAFDoc_ColorTool,
            XCAFDoc_ColorType,
            XCAFDoc_DocumentTool,
            XCAFDoc_ShapeTool,
        )
    except ImportError as exc:  # pragma: no cover - depends on the host
        raise StepImportError(
            "the STEP front-end needs the OCCT kernel bindings (OCP), which are "
            "not installed on this host: %s. Install them with "
            "`python -m pip install 'cad-to-gazebo[step]'` (or `pip install cadquery-ocp`). "
            "Nothing else in the cad package needs them." % exc
        ) from exc

    try:
        import OCP as _ocp_module

        version = getattr(_ocp_module, "__version__", "unknown")
    except Exception:  # pragma: no cover - version is cosmetic
        version = "unknown"

    _OCP_CACHE = {
        "Bnd_Box": Bnd_Box,
        "BRepBndLib": BRepBndLib,
        "BRepGProp": BRepGProp,
        "GProp_GProps": GProp_GProps,
        "IFSelect_ReturnStatus": IFSelect_ReturnStatus,
        "Interface_Static": Interface_Static,
        "STEPCAFControl_Reader": STEPCAFControl_Reader,
        "Sequence_TDF_Label": Sequence_TDF_Label,
        "TCollection_ExtendedString": TCollection_ExtendedString,
        "TDF_Label": TDF_Label,
        "TDataStd_Name": TDataStd_Name,
        "TDocStd_Document": TDocStd_Document,
        "TopAbs_FACE": TopAbs_FACE,
        "TopAbs_SOLID": TopAbs_SOLID,
        "TopExp_Explorer": TopExp_Explorer,
        "TopLoc_Location": TopLoc_Location,
        "XCAFApp_Application": XCAFApp_Application,
        "XCAFDoc_ColorTool": XCAFDoc_ColorTool,
        "XCAFDoc_ColorType": XCAFDoc_ColorType,
        "XCAFDoc_DocumentTool": XCAFDoc_DocumentTool,
        "XCAFDoc_ShapeTool": XCAFDoc_ShapeTool,
        "version": version,
    }
    return _OCP_CACHE


# --------------------------------------------------------------------------- #
# geometry helpers
# --------------------------------------------------------------------------- #


def _label_name(ocp: dict, label) -> str | None:
    attribute = ocp["TDataStd_Name"]()
    if label.FindAttribute(ocp["TDataStd_Name"].GetID_s(), attribute):
        name = attribute.Get().ToExtString()
        return name or None
    return None


def _count_sub_shapes(ocp: dict, shape, shape_type) -> int:
    explorer = ocp["TopExp_Explorer"](shape, shape_type)
    count = 0
    while explorer.More():
        count += 1
        explorer.Next()
    return count


def _bounding_box(ocp: dict, shape) -> list[float] | None:
    box = ocp["Bnd_Box"]()
    ocp["BRepBndLib"].Add_s(shape, box)
    if box.IsVoid():
        return None
    return [
        float(box.CornerMin().X()),
        float(box.CornerMin().Y()),
        float(box.CornerMin().Z()),
        float(box.CornerMax().X()),
        float(box.CornerMax().Y()),
        float(box.CornerMax().Z()),
    ]


def _volume(ocp: dict, shape, n_solids: int) -> float | None:
    if n_solids <= 0:
        return None
    props = ocp["GProp_GProps"]()
    try:
        ocp["BRepGProp"].VolumeProperties_s(shape, props)
    except Exception:
        return None
    return abs(float(props.Mass()))


def _transform(ocp: dict, location) -> list[float]:
    """Return the row-major 4x4 matrix of a ``TopLoc_Location``."""
    translation = location.Transformation()
    matrix = []
    for row in range(1, 4):
        for column in range(1, 4):
            matrix.append(float(translation.Value(row, column)))
        matrix.append(float(translation.TranslationPart().Coord(row)))
    matrix.extend([0.0, 0.0, 0.0, 1.0])
    return matrix


def _color_hex(ocp: dict, color_tool, label) -> str | None:
    color = None
    for color_type in (ocp["XCAFDoc_ColorType"].XCAFDoc_ColorGen,
                       ocp["XCAFDoc_ColorType"].XCAFDoc_ColorSurf):
        candidate = None
        try:
            from OCP.Quantity import Quantity_Color

            candidate = Quantity_Color()
        except Exception:  # pragma: no cover - kernel always ships Quantity
            return None
        if color_tool.GetColor_s(label, color_type, candidate):
            color = candidate
            break
    if color is None:
        return None
    red, green, blue = (int(round(max(0.0, min(1.0, color.Red())) * 255)),
                        int(round(max(0.0, min(1.0, color.Green())) * 255)),
                        int(round(max(0.0, min(1.0, color.Blue())) * 255)))
    return "#%02x%02x%02x" % (red, green, blue)


# --------------------------------------------------------------------------- #
# traversal
# --------------------------------------------------------------------------- #


class _Traversal:
    """Walks the XCAF shape tree once and fills the assembly/part lists.

    XCAF keeps *definitions* and *occurrences* apart: a component label is a
    reference to a shared assembly or part, plus its own placement. The walk
    therefore resolves a reference to its referent only to classify it and to
    fetch its children/geometry, while the occurrence label keeps the path and
    the placement that make the node unique.
    """

    def __init__(self, ocp: dict, shape_tool, color_tool, shapes_out: dict | None = None):
        self.ocp = ocp
        self.shape_tool = shape_tool
        self.color_tool = color_tool
        self.assemblies: list[dict] = []
        self.parts: list[dict] = []
        self._sibling_names: dict[tuple[str, str], int] = {}
        #: When the caller passes a dict, every part's unplaced (link-local)
        #: geometry is stored in it under the part path, so a downstream step
        #: (``step_to_ir`` tessellation) does not have to re-read the STEP file.
        self.shapes_out = shapes_out

    def walk(self, label, parent_path: str, parent_transform, depth: int) -> str:
        ocp = self.ocp
        shape_tool = self.shape_tool

        raw_name = _label_name(ocp, label) or "unnamed"
        key = (parent_path, raw_name)
        ordinal = self._sibling_names.get(key, 0) + 1
        self._sibling_names[key] = ordinal
        name = raw_name if ordinal == 1 else "%s#%d" % (raw_name, ordinal)
        path = "%s/%s" % (parent_path, name) if parent_path else name

        if depth > MAX_DEPTH:
            raise StepImportError(
                "assembly nesting deeper than %d at %r: the STEP tree is corrupt "
                "or self-referential" % (MAX_DEPTH, path)
            )

        location = shape_tool.GetLocation_s(label)
        transform = parent_transform * location

        referent = ocp["TDF_Label"]()
        target = label
        if shape_tool.IsReference_s(label) and shape_tool.GetReferredShape_s(label, referent):
            target = referent

        if shape_tool.IsSimpleShape_s(target):
            self._add_part(label, target, name, path, parent_path, transform, depth)
            return path

        if shape_tool.IsAssembly_s(target):
            children: list[str] = []
            record = {
                "name": name,
                "path": path,
                "parent": parent_path or None,
                "depth": depth,
                "transform": _transform(ocp, transform),
                "children": children,
            }
            self.assemblies.append(record)
            components = ocp["Sequence_TDF_Label"]()
            if shape_tool.GetComponents_s(target, components, False):
                for index in range(1, components.Length() + 1):
                    children.append(
                        self.walk(components.Value(index), path, transform, depth + 1)
                    )
            return path

        return path

    def _add_part(self, label, target, name, path, parent_path, transform, depth) -> None:
        ocp = self.ocp
        shape_tool = self.shape_tool
        # ``GetShape`` on the referent gives unplaced geometry; applying the
        # accumulated transform yields the shape in assembly coordinates.
        local_shape = shape_tool.GetShape_s(target)
        world_shape = local_shape.Moved(transform)
        if self.shapes_out is not None:
            self.shapes_out[path] = local_shape
        faces = _count_sub_shapes(ocp, world_shape, ocp["TopAbs_FACE"])
        solids = _count_sub_shapes(ocp, world_shape, ocp["TopAbs_SOLID"])
        bounds = _bounding_box(ocp, world_shape)
        self.parts.append({
            "name": name,
            "path": path,
            "parent": parent_path or None,
            "depth": depth,
            "transform": _transform(ocp, transform),
            "bbox": None if bounds is None else {"min": bounds[:3], "max": bounds[3:]},
            "volume": _volume(ocp, world_shape, solids),
            "faces": faces,
            "solids": solids,
            "color": _color_hex(ocp, self.color_tool, label),
        })


# --------------------------------------------------------------------------- #
# public entry point
# --------------------------------------------------------------------------- #


def import_step(step_path: str | Path, *, source_path: str | None = None,
                shapes_out: dict | None = None) -> dict:
    """Read ``step_path`` and return the IR dict documented in the module header.

    ``source_path`` overrides the ``source.path`` recorded in the IR (used when
    the file was fetched to a temporary location and the caller wants the IR to
    keep pointing at the canonical name).

    ``shapes_out`` is an optional dict the caller owns: on return it maps every
    part ``path`` to that part's unplaced (link-local) ``TopoDS_Shape``.
    :mod:`cad.step_to_ir` uses it to tessellate the parts into STL without
    reading the STEP file a second time. Shapes are OCCT objects: they are only
    valid while the kernel document is alive, so consume them in the same
    process.

    Raises :class:`StepImportError` for a missing/unreadable file, a file whose
    STEP header is absent or unreadable, a file the kernel refuses to parse,
    and a kernel that is not installed.
    """
    step_file = Path(step_path)
    if not step_file.is_file():
        raise StepImportError("STEP file not found: %s" % step_file)

    protocol = detect_protocol(step_file)
    ocp = _ocp()

    application = ocp["XCAFApp_Application"].GetApplication_s()
    document = ocp["TDocStd_Document"](ocp["TCollection_ExtendedString"]("XmlXCAF"))
    application.NewDocument(ocp["TCollection_ExtendedString"]("XmlXCAF"), document)
    shape_tool = ocp["XCAFDoc_DocumentTool"].ShapeTool_s(document.Main())
    color_tool = ocp["XCAFDoc_DocumentTool"].ColorTool_s(document.Main())

    reader = ocp["STEPCAFControl_Reader"]()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    reader.SetLayerMode(True)

    status = reader.ReadFile(str(step_file))
    if status != ocp["IFSelect_ReturnStatus"].IFSelect_RetDone:
        raise StepImportError(
            "the STEP kernel could not read %s (status %s): the file is truncated "
            "or its DATA section is malformed" % (step_file, status)
        )
    if not reader.Transfer(document):
        raise StepImportError(
            "the STEP kernel read %s but transferred no shape: the file carries no "
            "shape representation the kernel understands" % step_file
        )

    traversal = _Traversal(ocp, shape_tool, color_tool, shapes_out=shapes_out)
    roots = ocp["Sequence_TDF_Label"]()
    shape_tool.GetFreeShapes(roots)
    identity = ocp["TopLoc_Location"]()
    for index in range(1, roots.Length() + 1):
        traversal.walk(roots.Value(index), "", identity, 0)

    if not traversal.parts and not traversal.assemblies:
        raise StepImportError(
            "no shape found in %s: the file is a valid STEP container but contains "
            "no product/shape representation" % step_file
        )

    counts = {
        "assemblies": len(traversal.assemblies),
        "parts": len(traversal.parts),
        "solids": sum(part["solids"] for part in traversal.parts),
        "faces": sum(part["faces"] for part in traversal.parts),
    }

    return {
        "schema": IR_SCHEMA,
        "generator": {
            "tool": "cad.step_import",
            "version": IR_VERSION,
            "kernel": "OCP/OCCT %s" % ocp["version"],
            "headless": True,
        },
        "source": {
            "path": str(source_path if source_path is not None else step_file),
            "sha256": sha256_file(step_file),
            "bytes": step_file.stat().st_size,
            "format": "STEP",
            "file_schema": protocol["file_schema"],
            "application_protocol": protocol["application_protocol"],
            "file_description": protocol["file_description"],
        },
        "units": {"length": "MM", "source": "occt cascade unit"},
        "counts": counts,
        "assemblies": traversal.assemblies,
        "parts": traversal.parts,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m cad.step_import",
        description="Import a STEP file headless (no FreeCAD, no GUI) and emit the IR JSON.",
    )
    parser.add_argument("--step", "-s", required=True, help="path to the .step/.stp file")
    parser.add_argument(
        "--out", "-o", default=None,
        help="write the IR to this JSON file (default: stdout)",
    )
    parser.add_argument("--indent", type=int, default=2, help="JSON indent (default: 2)")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        ir = import_step(args.step)
    except StepImportError as exc:
        print("step_import: %s" % exc, file=sys.stderr)
        return 1

    payload = json.dumps(ir, indent=args.indent, sort_keys=False)
    if args.out:
        Path(args.out).write_text(payload + "\n", encoding="utf-8")
        print(
            "step_import: %s -> %s (%d assemblies, %d parts, %d solids)"
            % (args.step, args.out, ir["counts"]["assemblies"], ir["counts"]["parts"],
               ir["counts"]["solids"])
        )
    else:
        print(payload)
    return 0


if __name__ == "__main__":  # pragma: no cover - exercised through subprocesses
    raise SystemExit(main())
