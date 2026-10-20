"""cad-to-gazebo -- CAD -> IR -> collision hulls/materials -> SDF pipeline.

This repository is consumed by ``maritime_ws`` as the ``src/cad`` submodule, so
its modules are imported as ``from cad import ...``. Importing this package
never imports FreeCAD: the FreeCAD layer lives in :mod:`cad.freecad_exporter`
and is only loaded by the ``freecadcmd`` export entry point when FreeCAD is
present. The pure-Python units (:mod:`cad.material_props`,
:mod:`cad.cad_assembly`, :mod:`cad.config_sync`, :mod:`cad.export_manifest`)
run and are tested without it.

The IR contract lives in :mod:`cad.ir_schema` (versioned JSON Schema +
fail-fast validator) and :mod:`cad.naming` (the single implementation of the
link/joint/frame naming rules). See ``docs/ir-v1.md`` and
``docs/naming-contract.md``.
"""

__all__ = [
    "material_props",
    "cad_assembly",
    "config_sync",
    "export_manifest",
    "ir_schema",
    "naming",
]
