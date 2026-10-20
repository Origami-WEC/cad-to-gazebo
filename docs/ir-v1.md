# CAD IR v1 — schema, validator, migration

Status: **pinned**, version tag `cad-to-gazebo/cad-ir/1`.

* schema: [`schema/cad_ir_v1.schema.json`](../schema/cad_ir_v1.schema.json) (JSON Schema draft 2020-12)
* validator: [`ir_schema.py`](../ir_schema.py) (`cad.ir_schema`)
* naming rules: [`naming-contract.md`](naming-contract.md)

The IR is `config/cad_manifest.json`: the document the CAD front-end writes and
every emitter (SDF, URDF, hydro, mooring, pipeline) reads. It is the boundary
between "what the CAD says" and "what the model is", so it is versioned, and it
is validated before anything is written.

## 1. Sections

| Section          | Cardinality | Content                                                                       |
|------------------|-------------|-------------------------------------------------------------------------------|
| `schema`         | required    | version tag, `cad-to-gazebo/cad-ir/1`. Selects the contract.                   |
| `meta`           | required    | provenance: `generator`, `emitter_version`, `device_type`, `source_hash`, `units`, `created`, `root`, plus optional `source_document`, `source_file`, `tessellation_tolerance_mm`. **Closed**: unknown keys are an error. |
| `bodies`         | required¹   | rigid bodies (SDF links): `name`, `label`, `pose`, `solids`, `num_solids`.    |
| `bodies[].solids`| –           | geometry + mass/inertia + material: `density_kg_m3`, `density_source`, `reference_mass_kg`, `reference_volume_m3`, `reference_com_m`, `reference_inertia_kg_m2`, `visual_mesh`, `collision_mesh`, `stl_path`, `stl_collision_path`, `material{card,diffuse,ambient}`, `num_vertices`. |
| `joints`         | required¹   | DOF topology: `name`, `fc_type`, `parent`, `child`, `origin`, `axis`, `limits{lower,upper,lower_enabled,upper_enabled,unit,offset_m}`, `activated`. |
| `skipped_joints` | optional    | CAD joints deliberately not emitted, each with `label` + `reason`.            |
| `frames`         | optional    | local coordinate systems (LCS): `name`, `body`, `pose`. A joint origin is one of them. |
| `anchors`        | optional    | sensor / jet / mooring mount points: `name`, `kind`, `body`, `frame`, `pose`, `direction`. An anchor is a pose, never an SDF link. |
| `sensors`        | optional    | `name`, `type`, `frame`, `rate_hz`, `pose`, `topic`, `params`.                |
| `links`          | legacy      | pre-v1 flat link list. Read-only compatibility; see §5.                        |

¹ a document with at least `meta` is structurally valid so that a partially
emitted IR is diagnosable; the emitters always write `bodies` and `joints`.

Units are SI everywhere (`meta.units` is the constant `SI (m, kg, rad)`):
metres, kilograms, radians, quaternions as `[x, y, z, qx, qy, qz, qw]`. The
mm/deg conversions belong to the producers, never to the IR.

## 2. The validator

```bash
python -m cad.ir_schema config/cad_manifest.json     # exit 0 = valid, 1 = not
python -m cad.ir_schema --allow-legacy old.json      # migrate in memory, then validate
python -m cad.ir_schema --allow-legacy --migrate new.json old.json
python -m cad.ir_schema --print-schema
```

From Python:

```python
from cad import ir_schema

ir_schema.validate(manifest)                  # raises IRSchemaError
ir_schema.validate_file("config/cad_manifest.json")
ir_schema.collect_errors(manifest)            # list[str], never raises
```

Two passes, in this order:

1. **structural** — the JSON Schema, evaluated with `jsonschema`
   (a core dependency). Types, required keys, name grammar, pose arity, unit
   enums, and the closed `meta` contract.
2. **semantic** — the invariants JSON Schema cannot express:
   names unique inside their scope; `joints[].parent` / `child` and
   `meta.root` referencing an existing body; `frame.body`, `anchor.body`,
   `anchor.frame`, `sensor.frame` resolving; every pose carrying a unit
   quaternion; inertia matrices symmetric with a non-negative diagonal; joint
   axes non-zero and limits ordered (`lower <= upper`); `num_solids` agreeing
   with `len(solids)`.

A structurally broken document stops at pass 1: reporting semantic noise about a
document whose shape is wrong helps nobody. Otherwise **every** violation is
reported in one shot, with a JSON-pointer path and the reason:

```
IR non valido (config/cad_manifest.json): 2 violazione/i
  - joints[0].child: 'ghost' non e' un body del documento
  - bodies[0].solids: nome duplicato 'hull__shell'
```

### Where it runs

| Moment                       | Gate                                                                 |
|------------------------------|----------------------------------------------------------------------|
| every export, before writing | `freecad_exporter.py::_validate_ir()` and `create_demo_model.py`: nothing is written when the manifest breaks the contract (fail-fast: no half-valid IR ever reaches the emitters). |
| every export, before syncing configs | `export_manifest.py::validate_manifest()`: a second gate for manifests that came from elsewhere (an older checkout, a hand edit). Rejects a broken v1 document, warns about a legacy one. |
| CI                           | `.github/workflows/ci.yml` validates the shipped fixtures with the CLI, and `tests/test_ir_schema.py` covers the accepted/rejected cases. |

## 3. Names

All identifiers (bodies, solids, joints, frames, anchors, sensors) follow one
grammar and are produced by one function. See
[`naming-contract.md`](naming-contract.md).

## 4. Versioning policy

* The tag lives in the document (`schema`) and in `ir_schema.IR_SCHEMA`.
* **Additive** changes (new optional section, new optional key inside an open
  section) do **not** bump the tag; they bump `IR_VERSION`.
* **Breaking** changes (rename/remove a key, tighten a type, change the meaning
  of a value, change `meta.units`) bump the tag to `cad-to-gazebo/cad-ir/2` and
  ship a migration. A reader must refuse a tag it does not know instead of
  guessing: `cad.ir_schema` validates against exactly one contract.
* `meta` is the closed part of the document precisely because another repository
  matches on it; other sections stay open to producer extension
  (`additionalProperties: true`) so an emitter can carry extra data without
  forcing a version bump.
* `emitter_version` is `<package>/<version>` of the writing emitter
  (`cad-to-gazebo/0.1.0`), so a manifest can be traced back to the code that
  produced it. `device_type` is the WEC class (`point_absorber`,
  `oscillating_surge_converter`, `oscillating_water_column`, `overtopping`,
  `transmission`, `generic`, …); the exporter reads it from
  `MARITIME_DEVICE_TYPE` (default `generic`), the demo writes `point_absorber`.

## 5. Migration pre-v1 → v1 (explicit, documented)

Pre-v1 manifests — written before this contract — have no `schema` tag and
carry a flat `links` list instead of `bodies`, and no
`emitter_version` / `device_type`. They are **not** silently reinterpreted:

* `ir_schema.validate_file()` rejects them (`'schema' is a required property`);
* `ir_schema.migrate_legacy(manifest)` upgrades the document **in memory**:

  1. stamp `schema = "cad-to-gazebo/cad-ir/1"`;
  2. rebuild `meta` with the v1 fields — `emitter_version` (defaults to this
     package), `device_type` (defaults to `generic`), `source_hash`
     (defaults to `""`), `units` (normalised to the SI constant), `created`
     (kept, or now) — and carry over `source_document`, `source_file`, `root`,
     `tessellation_tolerance_mm`;
  3. fold each `links[i]` into `bodies`: a legacy link is both the link and its
     one geometry entry, so it becomes a body whose single solid carries the
     link's own density, mass, material and mesh. A link that already has
     `solids` keeps them and only gains a pose;
  4. leave `joints` and `skipped_joints` untouched (their shape is unchanged),
     drop the legacy `links` key;
  5. validate the result and refuse to return a document that still breaks the
     contract (e.g. names colliding once two links are folded together).

* `--allow-legacy` / `allow_legacy=True` runs that migration before validating,
  for readers (the `export_manifest.py` gate) that must not block a run because
  of an old file on disk. **Producers never do this**: they must write v1.

The new fields are *additive* for a reader and *required* for a writer, which is
why v1 keeps the legacy `links` shape readable (see `$defs.legacy_link`) while
refusing to emit it.
