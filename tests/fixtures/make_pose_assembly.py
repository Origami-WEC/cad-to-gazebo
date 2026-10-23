"""Genera, sotto ``freecadcmd``, un assieme sintetico con pose note.

Serve a ``test_exporter_poses.py``: due componenti dell'assieme con la
geometria in un documento esterno (``App::Link``) e come ``App::Part`` diretto,
ciascuno con posizione e rotazione non banali. Nessun file binario nel repo: il
``.FCStd`` si rigenera a ogni esecuzione dal JSON in ``GEN_SPEC``.

Output: ``<out_dir>/parts.FCStd`` (geometria) e ``<out_dir>/asm.FCStd`` (assieme).
"""

import json
import os

import FreeCAD as App


def _placement(spec):
    return App.Placement(App.Vector(*spec["pos_mm"]),
                         App.Rotation(App.Vector(*spec["axis"]), spec["angle_deg"]))


def _box(doc, name, spec):
    box = doc.addObject("Part::Box", name)
    box.Length, box.Width, box.Height = spec["size_mm"]
    box.Placement = _placement(spec["placement"])
    box.addProperty("App::PropertyFloat", "Density", "Material")
    box.Density = spec["density_kg_m3"]
    return box


def main():
    spec = json.load(open(os.environ["GEN_SPEC"]))
    out = os.environ["GEN_OUT"]

    parts = App.newDocument("parts")
    cell = parts.addObject("App::Part", "Cell")
    cell.addObject(_box(parts, "CellBox", spec["linked_box"]))
    parts.recompute()
    parts.saveAs(os.path.join(out, "parts.FCStd"))

    asm = App.newDocument("asm")
    asm.saveAs(os.path.join(out, "asm.FCStd"))  # un link esterno richiede il doc salvato
    link = asm.addObject("App::Link", "linked_body")
    link.setLink(cell)
    link.LinkTransform = False
    link.Placement = _placement(spec["link_placement"])
    direct = asm.addObject("App::Part", "direct_body")
    direct.Placement = _placement(spec["direct_placement"])
    direct.addObject(_box(asm, "DirectBox", spec["direct_box"]))
    try:
        group = asm.addObject("Assembly::AssemblyObject", "Assembly")
        group.addObject(link)
        group.addObject(direct)
    except Exception:  # Assembly WB assente: l'exporter ha il fallback top-level
        pass
    asm.recompute()
    asm.saveAs(os.path.join(out, "asm.FCStd"))


main()
