# SDF-ready artefacts (`cad-to-gazebo/sdf-artifacts/1`)

Contract of the document produced by `cad.sdf_artifacts` from *one* IR manifest
(`docs/ir-v1.md`). This is the last CAD-side artefact: the Gazebo emitter (C5+)
reads it and must not read the IR or the STL itself.

* module: `sdf_artifacts.py`
* generator: `python -m cad.sdf_artifacts --manifest <ir.json> --ws <workspace> --out <artefact.json>`
* schema tag: `cad-to-gazebo/sdf-artifacts/1` (`ARTIFACT_SCHEMA`)
* emitter version: `EMITTER_VERSION` (`cad-to-gazebo/sdf-artifacts` + package version)

The generator is pure Python and FreeCAD-free: it consumes the IR plus the
visual meshes under `<ws>/meshes/visual/`. It never re-reads the CAD.

## 1. Why this document exists

The IR describes the *intent* of the CAD (bodies, solids, mass properties,
frames). Gazebo/SDF needs *decidable geometry*: a collision shape per link, a
PBR material per visual, an inertial block per link, named LCS to hang sensors
and jets on. Everything the emitter would otherwise guess is decided here,
once, and written down — including every guess that had to be made and every
degradation that occurred (field `warnings`).

The generator is therefore **closed**: the same IR and the same visual meshes
produce the same bytes (section 7). No timestamp, no absolute path, no random
decimation, no dependence on the host.

## 2. Document layout

```
{
  "schema":    "cad-to-gazebo/sdf-artifacts/1",
  "meta":      { ... },    # provenance and budgets
  "bodies":    [ ... ],    # one entry per IR body (link), same order as the IR
  "frames":    [ ... ],    # named LCS: authored + derived from joints
  "anchors":   [ ... ],    # authored mount points (sensors, jets, fairleads)
  "sensors":   [ ... ]     # authored sensors, carried through verbatim
}
```

`bodies` keeps the IR order, so a diff of two artefacts lines up with a diff of
two IRs. Unknown-but-present IR fields are *not* invented here: the artefact is
an explicit allow-list of what the emitter may rely on.

### 2.1 `meta`

| field | meaning |
| --- | --- |
| `generator`, `emitter_version` | `sdf_artifacts.py`, `<package>/<version>` |
| `ir_schema` | the IR schema tag the input declared |
| `ir_hash` | `sha256:…` over the canonical IR with `meta.created` removed |
| `ir_source_hash` | the STEP hash the IR carries (`meta.source_hash`), or `""` |
| `root` | the IR root body (`model://` root link) |
| `units` | free-text unit statement, copied from the IR |
| `budget` | `{max_collision_faces, max_hull_inflation, primitive_cover, primitive_tolerance, primitive_volume_tolerance}` |

`ir_hash` is what makes the artefact traceable: the same IR content always has
the same hash (the IR's own `meta.created` is a wall-clock field, so it is
excluded); a changed IR always changes the hash. `ir_source_hash` links back to
the STEP file when the exporter recorded one. Every `budget` value is the
constant that was actually used for this run, so the numbers in the file can be
read without reading the generator.

### 2.2 `bodies[]`

```
{ "name": <body>, "label": <human label>, "pose": [7],
  "visuals":    [ ... ],   # one per IR solid
  "collisions": [ ... ],   # one per IR solid
  "inertial":   { ... }    # one per body, aggregated over its solids
}
```

`pose` is the body placement in the model frame, straight from the IR.

## 3. Visuals and materials (PBR)

One visual per IR solid:

```
{ "name": <solid>, "solid": <solid>, "mesh": "meshes/visual/<body>__<solid>.stl",
  "pose": [7], "material": { ... } }
```

`pose` is the solid placement in the body frame (identity for the common case).
Meshes and geometry are expressed in the *solid-local* frame, so a solid the IR
rotated or translated — `float__ballast` sits at z=-0.45 rotated 90 deg about x
— is placed correctly by the emitter without it re-deriving anything.

`material` is the SDF/`<material>` + PBR block the emitter copies into the
visual. It is always complete — never `null`, never partially filled:

| field | meaning |
| --- | --- |
| `card` | the CAD material card, verbatim (`""` when the solid had none) |
| `albedo`, `ambient` | the CAD diffuse colour, or the default `[0.6,0.6,0.6,1.0]` |
| `emissive` | always `[0,0,0,1]`: emissive is a rendering intent, not CAD data |
| `metalness`, `roughness` | from the BRDF table of the matched card |
| `fallback` | `true` when no table row matched: default BRDF applied |

Matching is a **longest-keyword** match on the lower-cased card against
`BRDF_TABLE` (so `Stainless Steel 316` matches the stainless row, not the
generic steel one). Placeholder cards (`""`, `default`, `demo`, `none`,
`unnamed`, `material`, …) never match: a demo card must not fabricate a
material. An unmatched card is *not* an error — it is `fallback: true` plus a
documented default, because a missing material must not stop the simulation. A
matched row owns `metalness`/`roughness`; the CAD colour is kept as albedo so
the render still looks like the CAD.

## 4. Collisions: the approximation ladder

Every IR solid gets exactly one collision entry. The ladder is evaluated in
this fixed order and the first rung that fits *wins*:

1. `box` — the AABB of the mesh;
2. `cylinder` — fitted on the principal axis of the mesh, with a radial
   coverage test on a fixed angle ladder;
3. `sphere` — fitted on the centroid: accepted only if every vertex is at the
   same radius;
4. `convex_hull` — qhull over the vertices, decimated to
   `MAX_COLLISION_FACES` by deterministic grid snapping;
5. `mesh` — the visual mesh itself (only when nothing above could be built).

Entry shape:

```
{ "name": <solid>, "approximation": "box|cylinder|sphere|convex_hull|mesh",
  "geometry": {"type": ...}, "pose": [7],
  "volume_m3": <float>,          # measured on the visual mesh
  "source_mesh": "meshes/visual/<body>__<solid>.stl",
  "collision_mesh": "meshes/collision/<body>__<solid>_convex.stl" | null,
  "hull_volume_m3": <float>, "hull_inflation": <float>,
  "faces": <int>, "decimation_cell_m": <float>|null,
  "fallback": <bool>, "warnings": [ ... ] }
```

* `geometry` is the SDF element to emit, always carrying `type`. The primitives
  also carry `cover` (the fraction of mesh vertices the shape covers, `1.0` for
  an exact fit) and `volume_m3` (the *analytic* volume of the fitted shape):
  `{"type":"box","size_m":[x,y,z],"centre_m":[x,y,z]}`,
  `{"type":"cylinder","radius_m":r,"length_m":l,"axis":"x|y|z","centre_m":…}`,
  `{"type":"sphere","radius_m":r,"centre_m":…}` or
  `{"type":"mesh","mesh":"meshes/collision/…convex.stl"}`. The mesh rung carries
  the path only — the accepted shape's volume is the record's `hull_volume_m3`,
  because a triangle soup has no analytic volume to declare.
  For the primitives, `centre_m` is the offset of the shape inside the
  solid-local frame; `pose` then places the whole thing in the body frame.
* `pose` is the solid pose, the same value as the matching visual: geometry and
  meshes are local, `pose` is absolute in the body.
* `volume_m3` is the volume *measured* on the visual mesh (it therefore carries
  the tessellation deficit: the sphere fixture is 2.8 % under the ideal
  `4/3 pi r^3`). `hull_volume_m3` is the volume of the accepted shape, and
  `hull_inflation = hull_volume_m3 / volume_m3`. Both are `null`-free by
  construction; `decimation_cell_m` and `collision_mesh` are only used by the
  `convex_hull` rung.
* `face budget`: a hull is decimated until it fits `MAX_COLLISION_FACES`
  (default 1500). The cell size is `bbox_diagonal / K`, `K` from the fixed
  `_GRID_LADDER`; snapping is deterministic, so the budget never changes the
  result between hosts. `faces == 0` for the primitives.
* `hull_inflation` above `MAX_HULL_INFLATION` (default 1.35) gets a warning: a
  convex hull of a very non-convex solid can be much bigger than the solid,
  which changes contact behaviour. The entry is still emitted — the warning is
  the signal.
* `fallback: true` only on the `mesh` rung: the collision is the visual mesh
  (fine for rendering, expensive for physics). It happens when neither qhull
  nor the source mesh is available; both cases are in `warnings`.
* If the ladder cannot meet the budget, the *smallest hull found* is returned
  (the face count is not monotonic in the cell size) and `warnings` says so:
  the artefact never silently ships an over-budget hull.
* The `convex_hull` rung needs qhull (`scipy`); without it the ladder stops at
  the primitives. `scipy` is part of the `freecad` extra — see §7.

The collision mesh, when written (`--write-collision`), is the exact triangle
soup that `faces` and `hull_volume_m3` describe, in `meshes/collision/`.

## 5. Inertia

`bodies[].inertial` is one entry per body, aggregated over the body's solids:

| field | meaning |
| --- | --- |
| `mass_kg`, `com_m` | total mass and body-frame centre of mass |
| `pose` | `com_m` + identity quaternion: an inertial pose the emitter can use as-is |
| `inertia_kg_m2` | 3x3 symmetric tensor about `com_m`, in the body frame |
| `sdf` | the same tensor as `ixx…izz` in the SDF `<inertia>` shape |
| `num_solids`, `solids[]` | per-solid mass/CoM/tensor/pose as consumed |
| `warnings[]` | e.g. a solid with a mass but no tensor: translational contribution only |

Aggregation is the standard parallel-axis composition over
`cad.cad_assembly.MassPoint` (same code path as the assembly report — one
implementation, not two). A solid without a tensor is *not* invented: its zero
tensor is used and the body says so in `warnings`.

A body whose total mass is zero is a hard error (`SdfArtifactError`): Gazebo
cannot simulate a massless link, and silently injecting a mass would be a
fabricated number.

## 6. Frames, anchors, sensors

`frames[]` is the *named LCS* of the model. Entry:
`{ "name", "body", "pose": [7], "description", "source": "ir"|"derived" }`.

* every IR frame is carried through with `source: "ir"`;
* every activated joint contributes a JCS on its **parent** (at `joint.origin`)
  and on its **child** (identity), named by `cad.naming.frame_name` — the single
  naming implementation — with `source: "derived"`;
* a frame that is already in the IR is never overwritten (idempotent re-runs).

`anchors[]` are the authored mount points (sensors on the pad, jets on the
thruster, fairleads). Entry:
`{ "name", "kind", "body", "frame", "pose": [7], "direction": [3], "source" }`.
Every anchor's `frame` must be an artefact frame or a body name, otherwise the
run fails (`SdfArtifactError`): an emitter that looks up a missing frame at
runtime is a runtime bug, so it is caught here.

`sensors[]` is the IR sensor list carried through verbatim plus
`source: "ir"`: `{ "name", "type", "frame", "pose": [7], "rate_hz", "topic",
"params"?, "source" }`. The emitter resolves their `frame` with the same rule.

`source` is on every frame/anchor/sensor so a reader can tell *authored* intent
from *derived* geometry, and so a diff between two runs is readable.

## 7. Reproducibility and the gate

* Output is canonical JSON (`json.dumps(sort_keys=True, indent=2)` + `"\n"`) and
  contains no absolute path, no timestamp and no host name.
* Two runs from the same IR + meshes are byte-identical; the committed fixture
  `tests/fixtures/sdf_artifacts_sample.json` (+ the collision STL next to it) is
  the reference output of `tests/fixtures/make_sample_meshes.py` +
  `cad_manifest_sample_assembly.json`.
* `tests/test_sdf_artifacts.py` regenerates from those fixtures and compares
  bytes, so any drift in the generator fails the suite; the CI job
  `Reproducible artefact (C4)` runs the two commands and `git diff --exit-code`
  on the same paths, so a hand-edited artefact fails too.
* The reference run exercises all four geometric rungs (box, cylinder, sphere,
  convex hull) and the decimation path, so the gate covers every branch.

## 8. How to regenerate and consume

```bash
# 1. (Only needed to rebuild the fixtures from scratch) sample meshes + IR
python tests/fixtures/make_sample_meshes.py --out tests/fixtures

# 2. the artefact (stdout by default; --out writes a file)
python -m cad.sdf_artifacts \
    --manifest tests/fixtures/cad_manifest_sample_assembly.json \
    --ws       tests/fixtures \
    --out      tests/fixtures/sdf_artifacts_sample.json \
    --write-collision
```

Flags: `--manifest` (IR, required), `--ws` (workspace root holding
`meshes/visual/`, default `.`), `--out` (default: stdout), `--write-collision`
(write the `convex_hull` soups under `<ws>/meshes/collision/`),
`--faces-max` (collision face budget, default `MAX_COLLISION_FACES`).

The emitter reads: `bodies[].visuals[].mesh` + `.material`, `bodies[].collisions[]`
(`geometry` + `pose`), `bodies[].inertial` (`pose`, `mass_kg`, `sdf`), and
resolves sensor/jet frames against `frames[] | bodies[].name`.

## 9. Degradation table

Every degraded path is local (one solid or one body), reported in
`warnings`, and never a crash — except the `SdfArtifactError` cases, which are
*contract* violations and must fail the run:

| condition | result |
| --- | --- |
| no `scipy` (no qhull) | hull rung unavailable; warnings says `scipy assente`; ladder stops at primitives/`mesh` |
| visual mesh missing on disk | `approximation: "mesh"`, `fallback: true`, warning |
| mesh not closed / zero volume | no primitive and no hull fit; `approximation: "mesh"`, `fallback: true`, warning |
| hull bigger than `MAX_HULL_INFLATION` | hull kept, warning `over-approximazione` |
| face budget unreachable | smallest hull found, warning `budget … non raggiungibile` |
| solid without inertia tensor | zero tensor for that solid, warning |
| material card unknown / placeholder | default BRDF, `fallback: true` |
| `SdfArtifactError` | invalid IR (validator), zero-mass body, anchor on an unknown frame |

