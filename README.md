# cad-to-gazebo

Turns the mechanical design into a simulatable model for the Origami WEC
simulator. It sits between the CAD in `Mechanism` and the Gazebo pipeline in
`maritime_ws`: it reads the FreeCAD assembly, writes a neutral intermediate
representation, derives collision hulls and material properties from it, and
emits the SDF-ready artefacts.

The package is **installable and testable without FreeCAD**: the FreeCAD API is
imported lazily, only inside the export layer that actually runs under
`freecadcmd`. Everything else is plain Python.

## Pipeline

```
   CAD                 IR                    hulls / materials           SDF
  ─────────────       ──────────────────     ─────────────────────      ──────────────
  cad/*.FCStd   ──▶   cad_manifest.json ──▶  meshes/visual/*.stl    ──▶  SDF model
  (FreeCAD           (bodies, joints,        material_props              (links, joints,
   assembly:          poses, materials)      cad_assembly                inertials)
   joints, solids,                                                       │
   poses, mats)                                                          ▼
  ─────────────       ──────────────────     ─────────────────────      ──────────────
  freecadcmd          export_manifest        cad_assembly               build_simulation
  (FreeCAD layer)     (manifest + cache)     material_props             (maritime_ws)
```

| Stage | Module | Role |
|---|---|---|
| CAD → IR | `freecad_exporter.py` | Runs under `freecadcmd`; reads the `.FCStd`, writes `config/cad_manifest.json` and the visual meshes. Cached on the source file SHA-256. Validates the manifest before writing it. |
| IR | `config/cad_manifest.json` | The intermediate representation: bodies, joints (parent/child/axis/limits), solid poses and material cards. Versioned contract: `schema/cad_ir_v1.schema.json`. |
| IR contract | `ir_schema.py`, `naming.py` | The versioned JSON Schema + fail-fast validator, and the single implementation of the link/joint/frame naming rules. |
| IR → hulls/materials | `cad_assembly.py`, `material_props.py` | Pose/joint maths; density and colour resolution from FreeCAD material cards or name keywords. |
| IR → SDF | `export_manifest.py`, `config_sync.py` | Regenerates the manifest + meshes and reconciles the derived configs (`pipeline.json`, `hydrodynamics_params.json`, `mooring.json`). |
| Demo fallback | `create_demo_model.py` | Builds a manifest without CAD when no `.FCStd` is available. |
| STEP → IR (headless) | `step_import.py` | Reads a STEP AP242/AP214/AP203 exchange file with the OCCT kernel and emits the IR: named, located assemblies and parts, in millimetres. No FreeCAD, no GUI, no X. |

## IR contract v1 (`cad-to-gazebo/cad-ir/1`)

`config/cad_manifest.json` is a versioned document, not an implementation
detail: the CAD front-end writes it, every emitter reads it. The contract is
pinned in three places:

* [`schema/cad_ir_v1.schema.json`](schema/cad_ir_v1.schema.json) — the JSON
  Schema (draft 2020-12): `meta`, `bodies` (geometry, mass/inertia, materials),
  `joints` (DOF topology), `frames`/LCS, `anchors` (sensor/jet mount points),
  `sensors`, plus the legacy `links` shape for read-only compatibility.
* [`docs/ir-v1.md`](docs/ir-v1.md) — section-by-section field contract,
  versioning policy, the validator's two passes and where it runs, and the
  documented pre-v1 → v1 migration.
* [`docs/naming-contract.md`](docs/naming-contract.md) — who names what, the
  SDF ↔ URDF ↔ ROS name mapping and the sanitisation rules. One implementation
  (`cad.naming.sanitize`), no second variant: `freecad_exporter.py` binds it
  instead of carrying its own.
* [`docs/sdf-artifacts.md`](docs/sdf-artifacts.md) — the SDF-ready artefact
  (`cad.sdf_artifacts`): collision-hull ladder, PBR materials, aggregated
  inertia, frames/anchors, the degradation table and the reproducibility gate.

The validator is **fail-fast**: `freecad_exporter.py` and `create_demo_model.py`
refuse to write a manifest that breaks the contract, `export_manifest.py` gates
the manifest again before syncing the derived configs, and CI validates the
shipped fixtures. Every violation is reported in one shot with a JSON-pointer
path:

```bash
python -m cad.ir_schema config/cad_manifest.json     # exit 0 = valid, 1 = not
python -m cad.ir_schema --allow-legacy old.json      # migrate in memory, then validate
python -m cad.ir_schema --print-schema
```

```python
from cad import ir_schema
ir_schema.validate(manifest)          # raises IRSchemaError with .errors
ir_schema.migrate_legacy(old_manifest)
```

`jsonschema` is a runtime dependency (the gate runs on every export).

## STEP import, headless (`cad.step_import`)

`freecad_exporter.py` needs a FreeCAD host. `step_import.py` does not: it reads
the STEP export of the same assembly through the OCCT kernel (the `OCP`
bindings shipped by `cadquery-ocp`) and emits the IR directly. That is the
supported path for a container or a CI runner with no display.

```bash
python -m pip install -e ".[step]"        # pulls cadquery-ocp (pulls vtk)
python -m cad.step_import --step Transmission/Assemblies/full_transmission.step \
                          --out config/step_ir.json
```

Without `--out` the IR goes to stdout. Exit status is `0` on success and `1`
with the reason on stderr for a missing file, a file that is not STEP, a
truncated or malformed DATA section, or a host without the kernel.

What the IR carries (`schema: "cad-to-gazebo/step-ir/1"`):

* `source` — path, SHA-256, byte size, declared `FILE_SCHEMA` and the
  application protocol derived from it (`AP242`, `AP214`, `AP203`,
  `AP_UNKNOWN`). A file that declares AP214 is reported as AP214; the reader
  never relabels it.
* `units` — OCCT reads into its cascade unit, so every coordinate is in
  millimetres.
* `assemblies` — one record per assembly, with its `path` from the root, its
  `parent`, its depth and a row-major 4x4 placement matrix.
* `parts` — one record per leaf shape, with its `path`, `parent`, placement,
  world-space `bbox`, `volume` (mm³, `null` for a non-solid leaf), `faces`,
  `solids` and `color`.
* `counts` — assembly/part/solid/face totals.

Names come from the STEP product names, not from traversal indices: a part is
`full_transmission/spare_transmission/spare_structure/profiles/alluminum 311/Body001`.
Where a name repeats among the siblings of one parent, a `#2`, `#3`, ... suffix
is appended in document order, so every `path` is unique and stable across
runs.

The kernel is imported lazily: `import cad.step_import` works, and the rest of
the package imports, lints and tests, on a host where `cadquery-ocp` cannot be
installed. Only calling `import_step()` requires it, and the failure names the
extra to install.

## Install

```bash
python -m pip install -e ".[dev]"     # editable + pytest + ruff
```

FreeCAD is **not** a pip dependency (it ships as an AppImage / conda build).
The `freecad` extra only pulls the pure-Python mesh helpers used by the export
and demo paths; the exporter imports FreeCAD lazily and fails only if it is
actually invoked without FreeCAD present.

## Run the tests

```bash
python -m pip install -e ".[dev,freecad]"   # `freecad` = trimesh, for the demo path
python -m pytest        # runs tests/ without FreeCAD and without maritime_ws
ruff check .            # lint (same rules the CI enforces)
```

The suite never imports FreeCAD. It injects minimal stand-ins for the
`ws_root` module and the `mooring` helper that live in the parent workspace, so
the pure units (`material_props`, `export_manifest`, `config_sync`) run in
isolation. See `tests/conftest.py`.

## How it is consumed by `maritime_ws`

This repository is mounted as the `src/cad` submodule of `maritime_ws`, and its
modules are imported as a package:

```python
from cad import cad_assembly, config_sync, export_manifest, material_props
```

The parent workspace provides the `ws_root` module (`workspace_root()`), which
points at the `maritime_ws` root where `config/` and `meshes/` live. When the
package runs standalone, `tests/conftest.py` supplies a stub for it.

## Branch / PR convention

- One branch per issue: `issue-<number>-<short-slug>`, cut from `main`.
- Open the PR against `main`; the PR body documents the approach and any
  discarded alternative.
- CI (`.github/workflows/ci.yml`) runs `ruff` and `python -m pytest` on every
  push and PR. A failing test returns a non-zero exit and fails the job — the
  gate is real, not syntax-only.
- Open design questions belong in PR review comments, not as `TODO`s in the
  code or this README.
