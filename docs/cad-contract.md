# The import contract (`--contract`)

`cad.step_import` reads a STEP file and describes the **geometry**: assembly
tree, placements, volume, bounding boxes. It does not describe the **physics**
of the WEC: joint topology, materials, densities, local coordinate systems,
mount points, sensors. The IR v1 (`cad-to-gazebo/cad-ir/1`) needs both, and the
bridge `cad.step_to_ir` does not invent the second half: it is *declared* in a
JSON contract passed with `--contract`.

```
python -m cad.step_to_ir --step device.step --ws . \
                         --contract contracts/device.json \
                         --out config/cad_manifest.json
```

Two responsibilities, kept apart on purpose:

* geometry comes from the CAD (STEP): names, placement, volume, CoM, inertia
  computed by the kernel on the B-rep;
* topology (joints, LCS, anchors, sensors) and materials/densities are
  **declared** in the contract, because a joint is not a geometric fact and a
  density is written in no STEP file.

Without a contract the bridge emits `joints: []`, `frames: []`, `anchors: []`,
`sensors: []` and says so on stderr: it does not invent topology.

## Closed keys, fail-fast

Every level is closed: an unknown key is a typo, not a feature. A key that is
*present* but empty (`null`) is an error too — absence is absence, `null` is
`null`. The contract is validated once, and the same rules apply whether it is
read from disk or passed as a dict, so the two paths cannot diverge.

Top-level keys: `device_type`, `root`, `tolerance_mm`, `density_kg_m3`,
`material_card`, `skip`, `parts`, `joints`, `frames`, `anchors`, `sensors`,
`source_document`.

`parts` and `skip` accept the CAD name **or** the IR (sanitised) name: both
indices exist, so the contract stays readable when the CAD uses spaces or
capitals (see `docs/naming-contract.md`).

## `joints`

Each entry declares `fc_type`, `parent`, `child`, and — for a movable joint —
`axis` and `limits` (SI numbers; the unit is deduced from the joint type, the
numbers are yours). Optional: `name`, `label`, `origin`, `activated`,
`distance`, `distance2`.

### The five non-native joint types

FreeCAD Assembly has joint types that are **not** a free degree of freedom:
they are *constraints* between two bodies. SDF/URDF cannot express them as a
single free DOF, so they must be **resolved upstream**, in the contract, with
the field `resolved_fc_type`. Without it the bridge stops and names the five
types.

| `fc_type`      | nature                      | typical `resolved_fc_type`      | residual |
|----------------|-----------------------------|---------------------------------|----------|
| `Distance`     | holds a distance            | `Fixed` (frozen) or the DOF that releases it | the distance is not a DOF |
| `Parallel`     | holds an orientation        | `Fixed`                         | orientation only |
| `Perpendicular`| holds an orientation        | `Fixed`                         | orientation only |
| `Angle`        | holds an angle              | `Fixed`, or `Revolute` if the angle *is* the DOF | the angle's range is not carried |
| `RackPinion`   | rack-and-pinion coupling    | `Revolute` (its primary DOF)    | the gear ratio is not in the v1 dialect |

Two ways to resolve:

* a **native type** that represents the constraint's primary DOF
  (`Fixed`, `Revolute`, `Continuous`, `Slider`, `Prismatic`, `Cylindrical`,
  `Screw`, `Ball`, `Spherical`, `Planar`) — then `axis`/`limits` are required
  as for any movable joint;
* **`Fixed`** — freeze the constraint as a rigid pose. A rigid pose has no
  stroke, so `limits` may be omitted and become `0 .. 0`, disabled; the bridge
  does not invent a travel.

In the IR the joint carries the *resolved* type, the original constraint in
`cad_constraint`, and `frozen: true` when it was frozen. The residual is
therefore traceable: whoever reads the IR can see that this DOF does not come
from a native joint, and must be re-declared downstream if it matters.

## `frames` — the LCS the CAD does not have

A local coordinate system is declared with `body`, `pose` and an optional
`name`/`description`. The IR name is composed with the shared naming
(`<body>__<local>`, e.g. `hull__pto_lcs`), so a frame is always anchored to a
body.

`pose` is `[x, y, z]` (translation only, identity rotation) or
`[x, y, z, qx, qy, qz, qw]`, in SI, in the body's local frame. A non-unit
quaternion is **refused**, not silently normalised: a normalisation would hide
a mistyped contract.

## `anchors` — where things mount

An anchor is a mount point (`sensor`, `jet`, `mooring`, `other`). It declares
`kind` and at least one of `body` / `frame`; `pose` and `direction` (a 3-vector)
are optional. The IR name is `<kind>_<body>_<local>`.

## `sensors` — what is read, and where

A sensor declares `type` (a class matching `^[a-z][a-z0-9_]*$`: `imu`, `gps`,
`camera`, `force_torque`, ...) and `frame` (required: a sensor reads in a
frame, and the bridge does not invent one). Optional: `name`, `pose`,
`rate_hz` (> 0), `topic`, `params` (an object). The IR name is
`<type>_<local>`.

## References resolve, or it stops

Every `body` reference in `frames`/`anchors` and every `frame` reference in
`anchors`/`sensors` must resolve to a body that survived the contract's `skip`
filter, or to a declared frame. A frame reference may be written with the local
name (`pto_lcs`) or the full IR name (`hull__pto_lcs`); the latter is bound to
the body, so a renamed or removed body is a **hard error** — not a section that
silently empties out.

## Where it is checked

`plan()` (kernel-free) resolves and verifies everything declarative; `convert()`
adds mass, inertia and meshes. Both raise `StepToIrError` with the offending
`contract frames[i]` / `contract joints[i]` / ... path. The emitted document is
then sealed by the fail-fast validator `ir_schema.validate`, which re-checks
that `frame.body` exists and that `anchor`/`sensor` frame references resolve.

## Which path honours the contract

There are two CAD front-ends in this repository, and only one of them takes a
contract. Said plainly, so nobody looks for a merge that does not exist:

* **STEP → IR** (`cad.step_to_ir --contract ...`): this is the SUB-3 entry
  point. It is the one that consumes the contract, because a STEP file carries
  geometry and *nothing else*: joints, LCS, anchors and sensors are declared
  here, in full.
* **FCStd → manifest** (`cad.export_manifest` → `freecadcmd` →
  `freecad_exporter.py`): the legacy path. It runs *inside* FreeCAD and reads
  joints natively from the `Assembly::Joint` objects of the document — they are
  declared objects there, not geometry inferred from the B-rep. It does not
  consume a contract and does not emit `frames`/`anchors`/`sensors`.

The two are not merged on purpose: merging the declared topology into the FCStd
manifest would mean two sources for the same joints (the document's joints and
the contract's), and "which one wins" has no clean answer. The STEP path is
authoritative for anything that carries topology; the FCStd path stays the
geometry-only exporter.

## Full example

```json
{
  "device_type": "point_absorber",
  "root": "hull",
  "density_kg_m3": 2700.0,
  "material_card": "aluminium",
  "parts": { "pto_shaft": { "density_kg_m3": 7850.0, "material_card": "steel" } },
  "joints": [
    {
      "name": "pto slider", "fc_type": "Slider",
      "parent": "hull", "child": "pto_shaft",
      "axis": [0.0, 0.0, 1.0], "limits": { "lower": -1.5, "upper": 1.5 }
    }
  ],
  "frames": [
    { "name": "pto_lcs", "body": "hull", "pose": [0.0, 0.0, 0.0] },
    { "name": "shaft_axis", "body": "pto_shaft", "pose": [0.0, 0.0, 0.5] }
  ],
  "anchors": [
    { "name": "mooring_fairlead", "kind": "mooring", "body": "hull",
      "pose": [0.5, 0.0, 0.0], "direction": [1.0, 0.0, 0.0] }
  ],
  "sensors": [
    { "name": "heave", "type": "imu", "frame": "pto_lcs",
      "rate_hz": 50.0, "topic": "/pto/heave" }
  ]
}
```

`tests/fixtures/contract_point_absorber.json` and
`tests/fixtures/contract_oscillator.json` are this shape and are exercised both
by the kernel-free suite (`tests/test_step_to_ir.py`) and by the end-to-end
harness (`tools/e2e_maritime_ws.sh`).
