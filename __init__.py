"""cad-to-gazebo -- CAD -> IR -> collision hulls/materials -> SDF pipeline.

This repository is consumed by ``maritime_ws`` as the ``src/cad`` submodule, so
its modules are imported as ``from cad import ...``. Importing this package
never imports FreeCAD: the FreeCAD layer lives in :mod:`cad.freecad_exporter`
and is only loaded by the ``freecadcmd`` export entry point when FreeCAD is
present. The pure-Python units (:mod:`cad.material_props`,
:mod:`cad.cad_assembly`, :mod:`cad.config_sync`, :mod:`cad.export_manifest`)
run and are tested without it.
"""

__all__ = [
    "material_props",
    "cad_assembly",
    "config_sync",
    "export_manifest",
]
