"""Test bootstrap for the standalone cad-to-gazebo repository.

The suite runs **without FreeCAD** and **without the maritime_ws workspace**:

* the repository root is mounted as the ``cad`` package (mirroring how
  maritime_ws consumes this repo as the ``src/cad`` submodule), so that
  ``from cad import ...`` resolves even when the repo directory is not
  literally named ``cad``;
* a minimal ``ws_root`` module is injected so ``cad.export_manifest`` (which
  does ``from ws_root import workspace_root``) imports without the parent
  workspace being present.

Nothing here imports FreeCAD: only pure-Python units are exercised.
"""

import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# ``cad`` package == repository root.
if "cad" not in sys.modules:
    _pkg = types.ModuleType("cad")
    _pkg.__path__ = [str(ROOT)]
    _pkg.__file__ = str(ROOT / "__init__.py")
    sys.modules["cad"] = _pkg

# ``ws_root`` shim for standalone runs (absent in the bare repo).
if "ws_root" not in sys.modules:
    _ws_root = types.ModuleType("ws_root")
    _ws_root.workspace_root = lambda: str(ROOT)
    sys.modules["ws_root"] = _ws_root

# ``mooring`` package shim: ``cad.config_sync`` lazily imports
# ``mooring.mooring_build.spawn_transform`` when estimating mooring line
# lengths. That module ships in maritime_ws, not here, so provide a minimal
# faithful implementation for the standalone suite.
if "mooring" not in sys.modules:
    import numpy as _np

    _mooring = types.ModuleType("mooring")
    _mooring.__path__ = []  # namespace-ish, submodule injected below
    _build = types.ModuleType("mooring.mooring_build")

    def _spawn_transform(pose):
        """pose [x,y,z,qx,qy,qz,qw] or [x,y,z,roll,pitch,yaw] -> (origin, rot)."""
        import math

        pose = list(pose or [0.0] * 6)
        pos = _np.asarray(pose[:3], dtype=float)
        if len(pose) >= 7:  # quaternion
            x, y, z, w = (float(v) for v in pose[3:7])
            n = _np.sqrt(x * x + y * y + z * z + w * w) or 1.0
            x, y, z, w = x / n, y / n, z / n, w / n
            rot = _np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ])
        else:  # roll, pitch, yaw
            r, p, yw = (float(v) for v in pose[3:6])
            cr, sr = math.cos(r), math.sin(r)
            cp, sp = math.cos(p), math.sin(p)
            cy, sy = math.cos(yw), math.sin(yw)
            rot = _np.array([
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ])
        return pos, rot

    _build.spawn_transform = _spawn_transform
    _mooring.mooring_build = _build
    sys.modules["mooring"] = _mooring
    sys.modules["mooring.mooring_build"] = _build

# ``simulation.app_config`` shim: ``cad.config_sync.derived_break_tension``
# reads the standard gravity from the single place maritime_ws defines it.
if "simulation" not in sys.modules:
    _sim = types.ModuleType("simulation")
    _sim.__path__ = []
    _app = types.ModuleType("simulation.app_config")
    _app.STANDARD_GRAVITY_M_S2 = 9.81
    _sim.app_config = _app
    sys.modules["simulation"] = _sim
    sys.modules["simulation.app_config"] = _app
