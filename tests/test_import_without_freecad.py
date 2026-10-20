"""Prove the ``cad`` package imports with FreeCAD absent.

Runs a child interpreter whose import hook raises on any ``FreeCAD`` import,
then imports the pure package and its pure submodules. The FreeCAD layer
(``cad.freecad_exporter``) is intentionally not imported here: it is only
loaded by the ``freecadcmd`` entry point, when FreeCAD is present.
"""

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

_BOOTSTRAP = """
import sys, types, builtins
ROOT = r"{root}"
pkg = types.ModuleType("cad"); pkg.__path__ = [ROOT]; pkg.__file__ = ROOT + "/__init__.py"
sys.modules["cad"] = pkg
ws = types.ModuleType("ws_root"); ws.workspace_root = lambda: ROOT
sys.modules["ws_root"] = ws

_real = builtins.__import__
def _guard(name, *a, **k):
    if name == "FreeCAD" or name.startswith("FreeCAD."):
        raise ImportError("FreeCAD must not be imported by the pure package")
    return _real(name, *a, **k)
builtins.__import__ = _guard

import cad
from cad import material_props, cad_assembly, config_sync, export_manifest
print("OK")
""".format(root=str(ROOT))


def _run(code):
    return subprocess.run([sys.executable, "-c", code],
                          capture_output=True, text=True)


def test_package_imports_without_freecad():
    res = _run(_BOOTSTRAP)
    assert res.returncode == 0, res.stderr
    assert "OK" in res.stdout


def test_freecad_guard_blocks_freecad_import():
    # Sanity check that the guard really blocks FreeCAD (so the test above
    # proves something): importing FreeCAD under the guard must fail.
    res = _run(_BOOTSTRAP + "\ntry:\n    import FreeCAD\nexcept ImportError:\n"
               "    print('BLOCKED')\n")
    assert "BLOCKED" in res.stdout
