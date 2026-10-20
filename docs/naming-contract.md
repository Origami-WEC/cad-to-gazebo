# Naming contract — link / joint / frame

Status: **pinned** (IR v1, `cad-to-gazebo/cad-ir/1`).
Machine-checked by `tests/test_naming.py`; the identifier grammar itself is
`$defs.name` in [`schema/cad_ir_v1.schema.json`](../schema/cad_ir_v1.schema.json).

This document is the single source of truth for "who names what" between the CAD
front-end, the IR, the SDF/URDF emitters and ROS. Any change here is a contract
change: it needs a schema revision, not a quiet edit in a producer.

## 1. The rules

1. **One identifier, one grammar.** Every link, joint, frame (LCS), anchor and
   sensor name in the IR matches
   `^[a-z0-9](?:[a-z0-9_]*[a-z0-9])?$`, at most 64 characters
   (`cad.naming.NAME_PATTERN`, `MAX_NAME_LEN`). Lowercase snake_case, no
   leading/trailing separator, no doubled *semantics* (a literal `__` is legal
   and is used by the mesh convention, see §5).
2. **The label is not the name.** The raw CAD label travels with the entity in
   its `label` field, for traceability. Nothing downstream may use a `label` as
   an identifier.
3. **One implementation.** `cad.naming.sanitize()` is the only sanitiser in the
   repository; `cad.naming.unique()` is the only uniqueness rule.
   `freecad_exporter.py` binds `_sanitize = naming.sanitize` and
   `_unique_name = naming.unique`; it does **not** define its own. A second
   variant is a defect: `tests/test_naming.py::test_no_second_sanitizer_in_the_repository`
   fails the build if one appears.
4. **Sanitisation is idempotent and total.** `sanitize(sanitize(x)) == sanitize(x)`
   and `sanitize(x)` is always a valid IR name; unreadable labels fall back to
   `link` (or the caller's `fallback`) instead of producing an anonymous entity.
5. **Uniqueness is per scope, in document order.** Scopes: bodies, solids *of a
   body*, joints, frames, anchors, sensors. The first occurrence keeps the bare
   name, later ones become `<name>_2`, `<name>_3`, …
6. **Determinism.** The same CAD document with the same traversal order produces
   the same names. Nothing in the pipeline uses hashes, timestamps or random
   suffixes in an identifier.

### Sanitisation, step by step

| Input              | Step                                                        | Output        |
|--------------------|-------------------------------------------------------------|---------------|
| `"Hull Shell"`     | `strip().lower()`, spaces → `_`                             | `hull_shell`  |
| `"flange 3/4\""`   | drop everything outside `[a-z0-9_]`                          | `flange_34`   |
| `"_spar_"`         | strip stray leading/trailing separators                      | `spar`        |
| `"mooring #2"`     | `#` is not an identifier character (it is a path locator)    | `mooring_2`   |
| `"x" * 200`        | truncate to 64 characters, then strip separators             | `x`*64        |
| `""`, `None`, `"!!!"` | fall back                                                 | `link`        |

## 2. Who names what

| Entity          | Producer                        | Source                                                    | Function                          |
|-----------------|---------------------------------|-----------------------------------------------------------|-----------------------------------|
| body / link     | `freecad_exporter.py`, `create_demo_model.py` | CAD part label                              | `naming.sanitize(label)` then `naming.unique(...)` |
| solid           | `freecad_exporter.py`           | item label inside the part (`item["label"]`)               | `naming.unique(naming.sanitize(label), taken)` |
| joint           | `freecad_exporter.py`           | FreeCAD Assembly joint label                               | `naming.unique(naming.sanitize(label), taken)` |
| frame (LCS)     | emitter / anchor author         | the body it belongs to + the local name                    | `naming.frame_name(body, local)`  |
| anchor          | model author (sensor/jet mount) | kind + body + local name                                   | `naming.anchor_name(kind, body, local)` |
| sensor          | model author                    | sensor class + local name                                  | `naming.sensor_name(type, local)` |
| mesh file       | exporters                       | body + solid names                                         | `naming.visual_mesh_name`, `naming.collision_mesh_name` |
| STEP path       | `step_import.py` (CAD front-end)| component path inside the STEP assembly                    | *not an IR name*, see §4           |

No other component may invent a name. If a new producer needs one, it uses the
`naming` helpers; if a new grammar is needed, the schema pattern moves and this
document moves with it.

## 3. SDF ↔ URDF ↔ ROS mapping

The IR name *is* the emitted name. There is no per-target renaming table, no
prefixing and no camel-casing — that is the whole point of the contract: a name
that is legal in the IR is legal in SDF, URDF and ROS at the same time.

| IR                    | SDF (Gazebo)                          | URDF                              | ROS                                            |
|-----------------------|---------------------------------------|-----------------------------------|------------------------------------------------|
| `bodies[].name`       | `<model><link name=…>`                | `<robot><link name=…>`            | tf frame `/world/<name>`, topic `/<name>/…`    |
| `joints[].name`       | `<joint name=… type=…>`               | `<joint name=… type=…>`           | joint state entry `<name>`, controller `<name>`|
| `joints[].parent`     | `joint/parent`                        | `joint/parent@link`               | parent frame in tf                              |
| `joints[].child`      | `joint/child`                         | `joint/child@link`                | child frame in tf                               |
| `joints[].origin`     | `joint/pose`                          | `joint/origin@xyz+rpy`            | tf static transform                             |
| `joints[].axis`       | `joint/axis/xyz`                      | `joint/axis@xyz`                  | effort/velocity direction                       |
| `joints[].limits`     | `joint/axis/limit`                    | `joint/limit`                     | `joint_limits.yaml`                             |
| `frames[].name`       | `<link><pose>` / static `<frame>`     | `<link>` fixed child               | tf frame `<name>`                               |
| `anchors[].name`      | sensor or jet mount pose in the model | fixed-joint mount point           | topic prefix for the mounted device             |
| `sensors[].name`      | `<sensor name=…>`                     | `<gazebo>` plugin + `<sensor>`    | node name / topic from `sensors[].topic`        |
| `meta.root`           | model `<static>` on that link         | base link (`<robot>` root)        | tf root frame `/world/<root>`                   |

Type mapping is documented with the emitters; this table fixes the *names* only.

## 4. STEP paths are locators, not identifiers

`step_import.py` addresses CAD components with an assembly path
(`full_transmission/spare_structure/profiles/311`, with a `#N` suffix on
duplicated siblings). A path is a *locator*: it must stay stable inside the CAD
tree to be searchable, so it keeps its own spelling and its `#N` sibling
suffix — that is not a second sanitiser, it is an addressing scheme.

The crossing point is explicit: when a path has to become an IR identifier, it
goes through `naming.ir_name_from_path()`, which strips the `#N` locator and
runs the *one* sanitiser:

```
full_transmission/spare_structure/profiles/311 -> full_transmission_spare_structure_profiles_311
float/#2                                        -> float
```

## 5. Meshes and files

Merged geometry files are named from sanitised pieces joined by `__`:

```
meshes/visual/<body>__<solid>.stl
meshes/collision/<body>__<solid>_convex.stl
```

`__` is a *file* separator. The manifest references both the relative path
(`stl_path`, `stl_collision_path`) and the Gazebo URI (`model://…`, in
`visual_mesh` / `collision_mesh`). Because `_` is a legal name character, a
consumer must never derive a body/solid name by splitting a file name — read
`bodies[].solids[].name`.

## 6. How this is enforced

| Rule                       | Enforced by                                                     |
|----------------------------|-----------------------------------------------------------------|
| name grammar               | `$defs.name` pattern in the schema, checked by `cad.ir_schema`   |
| uniqueness per scope       | semantic pass of `cad.ir_schema` (`duplicato`)                    |
| one sanitiser              | `tests/test_naming.py::test_no_second_sanitizer_in_the_repository` |
| exporter reuse             | `tests/test_naming.py::test_exporter_reuses_the_single_implementation` |
| idempotence / totality     | `tests/test_naming.py` parametrised cases                         |
| schema ⟷ code agreement    | `test_schema_name_pattern_is_the_naming_contract`                 |
