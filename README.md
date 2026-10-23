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
| IR → URDF (ROS) | `urdf_skeleton.py` | Emits the kinematic skeleton for ROS: one `<link>` per body, one `<joint>` per joint, and nothing else — no geometry, no dynamics, no material. TF / RViz / `robot_state_publisher`. |

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
* [`docs/cad-contract.md`](docs/cad-contract.md) — the `--contract` the caller
  writes: `parts`, `joints`, the `frames`/LCS, `anchors` and `sensors` sections,
  the reference-resolution rules, and the five non-native FreeCAD joint types
  with their `resolved_fc_type` escape hatch.
* [`docs/naming-contract.md`](docs/naming-contract.md) — who names what, the
  SDF ↔ URDF ↔ ROS name mapping and the sanitisation rules. One implementation
  (`cad.naming.sanitize`), no second variant: `freecad_exporter.py` binds it
  instead of carrying its own.
* [`docs/urdf-skeleton.md`](docs/urdf-skeleton.md) — the kinematic URDF skeleton
  (`cad.urdf_skeleton`): the only object this repository derives from the IR, the
  joint-type mapping and its escape hatch, and what "equivalent to the IR" means.

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

## STEP → IR bridge (`cad.step_to_ir`) — why it lives here

`step_import.py` (C2) describes the *geometry* of a STEP file:
`schema: "cad-to-gazebo/step-ir/1"`, with `source`, `units`, `counts`,
`assemblies`, `parts`. The contract every consumer in the chain reads —
`config_sync.py` and `build_simulation.py` in `maritime_ws` —
is `cad-ir/1` (C3): `meta`, `bodies`, `joints`. They are two different
documents by design, and `step_to_ir.py` is the one bridge between them:

```bash
python -m cad.step_to_ir --step Transmission/Assemblies/full_transmission.step \
                         --ws . --contract contracts/transmission.json \
                         --out config/cad_manifest.json
```

The split of responsibility is explicit and is the decision this block records:

* **geometry comes from the CAD.** Names, placements, volume, CoM and the
  inertia tensor are computed by the kernel on the B-rep. The STEP file is read
  once: `step_import.import_step(shapes_out=...)` hands the OCCT shapes to the
  bridge, which tessellates them into `meshes/visual/<body>__<solid>.stl`.
* **a part may hold more than one solid.** A STEP part can be a compound of N
  solids — the real `full_transmission.step` has two (the SBR20 linear
  supported rails, each modelled as rail + slider in one CAD part). The bridge
  emits **one IR solid per OCCT solid**, each with its own volume, mass, CoM,
  inertia and mesh (`<body>__<solid>.stl`), and sets `num_solids` to the number
  of solids *emitted*, so the contract invariant `num_solids == len(solids)`
  (see `docs/ir-v1.md`) holds by construction. Collapsing them into a single
  aggregate solid would sum the masses but lose the per-solid split the SDF
  collision/model step needs, and was the defect this fixes.
* **joint topology and materials are *declared*, not inferred.** A joint is not
  a geometric fact — a bolt and a slider have the same faces — and no density is
  written in a STEP file. They arrive in a JSON contract (`--contract`).
  **Without a contract the bridge emits `joints: []` and density defaults: it
  never invents topology from the assembly tree or from coincident faces.**
  Inferring DOF from geometry was the alternative and it is discarded: it
  guesses design intent and would be wrong for every device whose joints are
  chosen rather than shaped.

The bridge lives in `cad-to-gazebo`, not in `maritime_ws`, because both document
shapes are defined and versioned in this repository (`docs/ir-v1.md`,
`schema/`), and because pairing the CAD geometry with the declared physics is a
CAD-side concern. `maritime_ws` stays a consumer: it reads the produced
`cad-ir/1` manifest and builds.

## SDF gate (`cad.sdf_validate`)

`build_simulation.py` used to accept the emitted SDF with `gz sdf -k` *when the
Gazebo CLI happened to be on `PATH`*, and otherwise print
`[ATTENZIONE] CLI 'gz' non trovato: salto la validazione SDF`. A build that
skips validation proves nothing. `sdf_validate.py` closes that hole and is the
gate C5 requires:

```bash
python -m cad.sdf_validate --strict description/model.sdf
python -m cad.sdf_validate --backend gz model.sdf   # fail if `gz` is absent
```

* `gz` present → `gz sdf -p <file>`, the reference parser; its exit code is the
  verdict.
* `gz` absent → an embedded SDFormat backend: a real XML parse plus conformance
  checks (required attributes, joint type, `parent`/`child` resolving to an
  existing link, duplicate names, `<pose>` shape, limits ordering), never a
  string comparison. The backend that ran is always printed, so the two are
  never confused.

Two severity levels: **errors** are what `gz sdf` itself rejects and always
fail; **warnings** are physical hygiene SDFormat silently defaults (missing
`<inertial>`, non-positive mass, a link with no collision, a disconnected link).
`--strict` promotes warnings to failures, which is what the harness uses.

## Kinematic URDF skeleton (`cad.urdf_skeleton`)

The ROS side needs a *kinematic* description (TF, RViz, `robot_state_publisher`),
not a second dynamics model. `sdformat_urdf` would translate the SDF into URDF,
but only understands URDF's joint vocabulary: a CAD model with a cylindrical,
ball or screw joint fails that translation. So the skeleton is derived from the
same single source of truth the SDF is derived from — the IR:

```bash
python -m cad.urdf_skeleton --manifest config/cad_manifest.json --out m.urdf
python -m cad.urdf_skeleton --manifest config/cad_manifest.json --strict
```

One `<link>` per IR body, one `<joint>` per IR joint, same names, same
parent/child edges, same axes, same origins — and **nothing else**: no
`<inertial>`, `<visual>`, `<collision>`, `<material>`, `<gazebo>`, `<plugin>`,
`<transmission>` or `<sensor>` element exists in the output, and the validator
treats one appearing as an *error*. A skeleton cannot drift from the IR because
it carries nothing that can drift; geometry and dynamics stay in
the SDF emitter of `maritime_ws`.

The IR's CAD vocabulary is collapsed onto URDF's four joint types by a
documented table (`Fixed`, `Revolute`, `Continuous`, `Slider`/`Prismatic`,
`Cylindrical`, `Screw`, `RackPinion`/`Gear`/`Belt`, `Ball`, `Distance`), a
`revolute` joint with no enabled bound becomes `continuous`, and a joint type
outside the table **fails the run** instead of being guessed — unless the joint
declares its own `urdf_type`. A body that no joint attaches stays its own root
and is reported as a warning (never silently bolted to the first link):
`--strict` rejects it, which is what CI does.

`tests/fixtures/urdf_skeleton_sample.urdf` is the byte-exact output for
`tests/fixtures/cad_manifest_urdf_skeleton.json`, and CI regenerates it and
diffs. The test suite proves equivalence with the IR field by field (link set,
joint set, edges, axes, poses, the type table written out in the test), asserts
the lean tag set as an equality, and runs `check_urdf` when the ROS CLI is
present. Full contract: [`docs/urdf-skeleton.md`](docs/urdf-skeleton.md).

## End-to-end: one command

From a clean clone, with a `maritime_ws` checkout next to this repository:

```bash
python -m pip install -e ".[dev,freecad,step]"
MARITIME_WS=../maritime_ws tools/e2e_maritime_ws.sh --strict
```

The single command runs five stages and fails on the first one that cannot run
(a skipped stage is a failure, not a silent pass):

1. CAD → IR v1 + visual meshes (`create_demo_model.py`, the no-FreeCAD path);
2. IR → `config/*.json` (`config_sync.py`), twice, to prove idempotence;
3. IR + config → `description/model.sdf` (`maritime_ws/build_simulation.py`);
4. SDF gate (`cad.sdf_validate`).

That is the demo path. Stage 5 reruns the same pipeline **from a real STEP
file** — `cad.step_import` → `cad.step_to_ir` (with
`tests/fixtures/contract_point_absorber.json`) → `config_sync` → the
`maritime_ws` build → `cad.sdf_validate` — so the command proves the STEP →
IR → artefacts → `maritime_ws` → valid SDF chain, not just that the demo
generator is green. `--submodule` runs it against the pinned
`maritime_ws/src/cad` checkout instead of this working tree.

## Pinned in `maritime_ws` (submodule)

`maritime_ws` mounts this repository as `src/cad`. The pointer is a **commit
hash, not a floating branch**, so a `maritime_ws` build is reproducible from the
CAD side. Moving the pin is one command:

```bash
tools/pin_cad_submodule.sh ../maritime_ws <commit-sha>      # commit on cad-to-gazebo
```

It fetches the commit into the submodule, checks it out, stages `src/cad` and
commits the pointer in the `maritime_ws` checkout. To verify a pin already in
place, `git -C <maritime_ws> submodule status src/cad` prints the commit, and
`tools/e2e_maritime_ws.sh --submodule --strict` runs the whole chain against it.

## Branch / PR convention

- One branch per issue: `issue-<number>-<short-slug>`, cut from `main`.
- Open the PR against `main`; the PR body documents the approach and any
  discarded alternative.
- CI (`.github/workflows/ci.yml`) runs `ruff` and `python -m pytest` on every
  push and PR. A failing test returns a non-zero exit and fails the job — the
  gate is real, not syntax-only.
- Open design questions belong in PR review comments, not as `TODO`s in the
  code or this README.
