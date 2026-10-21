# Kinematic URDF skeleton (`cad-to-gazebo/urdf-skeleton/1`)

Block **C6** of the chain: `cad_manifest.json` (`cad-to-gazebo/cad-ir/1`) →
`<robot>.urdf`, a *kinematic skeleton* for ROS — TF, RViz, `robot_state_publisher`,
`robot_description`. One `<link>` per IR body, one `<joint>` per IR joint, and
nothing else.

## 1. What this artefact is, and what it is not

It **is** the DOF topology of the device expressed in URDF: the same bodies, the
same parent/child edges, the same axes, the same origins as the IR, in the four
joint types URDF actually has.

It is **not** a second twin of the SDF model. There is no inertia, no mass, no
inertial frame, no mesh reference, no `<visual>`, no `<collision>`, no
`<material>`, no `<gazebo>`, no `<plugin>`, no `<transmission>`, no `<sensor>`.
Two artefacts with the same geometry and dynamics would be two things to keep in
sync; a skeleton cannot drift because it carries nothing that can drift. Dynamics
and geometry belong to `cad.sdf_artifacts` (`docs/sdf-artifacts.md`) and to the
SDF gate (`cad.sdf_validate`).

The reason the URDF exists at all is *this* pipeline: `sdformat_urdf` loads an
SDF file into ROS by translating it, and it only understands URDF's joint
vocabulary (`fixed`, `revolute`, `continuous`, `prismatic`, `floating`, `planar`).
A CAD model that uses a cylindrical, ball or screw joint fails that translation.
Deriving the ROS-side skeleton from the IR removes the dependency on that
translator and keeps the ROS side derivable from the same single source of truth.

The file is closed and quiet: no `TODO`, no question, no comment that asks the
reader to decide something. Open points live in the PR, not in an artefact the
ROS stack parses.

## 2. Artefacts, commands, consumption

| Artefact | Producer | Consumers |
|---|---|---|
| `<m>.urdf` | `python -m cad.urdf_skeleton --manifest <ir.json> --out <m>.urdf` | ROS: `robot_state_publisher`, `rviz2`, TF, `xacro`-free pipelines |

```bash
python -m cad.urdf_skeleton --manifest config/cad_manifest.json            # stdout
python -m cad.urdf_skeleton --manifest config/cad_manifest.json --out m.urdf
python -m cad.urdf_skeleton --manifest config/cad_manifest.json --strict   # one root required
```

```python
from cad import urdf_skeleton
text, warnings = urdf_skeleton.generate_urdf(manifest)   # validates the IR first
urdf_skeleton.collect_errors(text, require_single_root=True)   # [] = accepted
```

The IR is validated with `cad.ir_schema` before anything is emitted, so a broken
manifest fails here with the same JSON-pointer error it would fail with anywhere
else, not with a half-written URDF.

## 3. The mapping (IR → URDF)

### 3.1 Joints

The URDF type is resolved in this order; the first rule that applies wins.

| # | Condition on the IR joint | URDF type | Why |
|---|---|---|---|
| R1 | `activated == false` | `fixed` | the joint is not a DOF, but its child body is still attached: the edge stays in the tree, rigid |
| R2 | `urdf_type` present on the joint | that value (validated against the four) | the escape hatch for a joint whose DOF the model author has already decided |
| R3 | `fc_type == "Fixed"` | `fixed` | the CAD says the two bodies are one rigid body |
| R4 | `fc_type == "Revolute"` and **both** bounds disabled (`lower_enabled`/`upper_enabled` false) | `continuous` | the IR carries no bound at all; `continuous` is URDF's unbounded rotation |
| R5 | `fc_type` in the table of §3.2 | the mapped type | the CAD vocabulary is not URDF's vocabulary |
| R6 | nothing matched | **hard error** | a DOF is never guessed (see below) |

`urdf_type` is read from the joint when present and is *not* part of the v1
contract: the IR does not require it and no front-end has to write it. It exists
so that a joint whose *intended* DOF is a design decision (a cylindrical joint
used as an actuator, a ball joint used as a hinge) is not silently flattened by
the table of §3.2. Nothing else on the joint can be declared implicitly.

### 3.2 The collapse table

FreeCAD's joint vocabulary is larger than URDF's, and a URDF skeleton has to
choose one DOF for each joint. The choice is a documented collapse, applied
identically for every model:

| FreeCAD `fc_type` | URDF | Which DOF survives |
|---|---|---|
| `Fixed` | `fixed` | none — rigid edge |
| `Revolute` | `revolute` | the rotation |
| `Continuous` | `continuous` | the rotation, unbounded |
| `Slider`, `Prismatic` | `prismatic` | the translation |
| `Cylindrical` | `prismatic` | the actuator's translation; the co-axial spin is not carried (URDF has no cylindrical joint) |
| `Screw` | `prismatic` | the linear DOF a screw converts rotation into |
| `RackPinion`, `Gear`, `Belt` | `revolute` | the primary angular DOF |
| `Ball` | `continuous` | one rotation out of three; the other two are not carried |
| `Distance` | `fixed` | none — a distance joint is a constraint, not a free coordinate |

A joint whose `fc_type` is **not** in this table is a hard error
(`UrdfSkeletonError`, exit 1). A DOF is never guessed: either the mapping is
documented here, or the joint declares `urdf_type`, or the run stops and says so.
The error names the joint and the two ways to resolve it.

### 3.3 Names

Names are transported verbatim, never re-derived: the URDF `<link name>` is the
IR body `name` and the `<joint name>` is the IR joint `name`. There is no second
implementation of `cad.naming` here, so the SDF side, the URDF side and the ROS
side cannot disagree about what a part is called
(`docs/naming-contract.md`).

### 3.4 Origins

The IR joint `origin` is a pose7 `[x, y, z, qx, qy, qz, qw]`; URDF wants
`<origin xyz="..." rpy="..."/>`. The translation is copied, the rotation is
converted to fixed-axis roll-pitch-yaw (`R = Rz(yaw)·Ry(pitch)·Rx(roll)`) and
both are emitted rounded to 9 decimals, always — an absent origin becomes an
explicit `xyz="0 0 0" rpy="0 0 0"`, because a document that omits a default and a
document that states it are not byte-identical.

At `|pitch| = 90°` the quaternion does not determine a unique triple. The
convention is pinned: `roll = 0` and the remaining rotation folded into `yaw`
(the singularity is detected with a `1e-12` tolerance on `|pitch| - π/2`). It is
a convention, not a measurement: the matrix it produces is the same, and the
choice is deterministic across runs and platforms.

### 3.5 Axes

`<axis>` is emitted for every movable joint, normalised to unit length, and
omitted for `fixed` joints, where an axis says nothing. A null axis in the IR is
a hard error rather than a silent `(0, 0, 0)`, which would make the joint
undefined in a way only the simulator would notice.

### 3.6 Limits

* `fixed` → no `<limit>` element.
* `revolute`, `prismatic` → `<limit lower upper effort velocity>`: the numeric IR
  bounds verbatim, with `effort`/`velocity` taken from the joint's `effort` /
  `velocity` keys when the IR carries them.
* `continuous` → `<limit effort velocity>`, no bounds: URDF ignores `lower`/`upper`
  on an unbounded joint and emitting them would suggest a range that does not exist.

URDF requires `effort` and `velocity` on every limit and the IR does not carry
them (they are control parameters, not geometry). The placeholders used when the
joint declares neither are the module constants `DEFAULT_EFFORT = 1000.0` and
`DEFAULT_VELOCITY = 100.0` — a *kinematic* skeleton has no controller behind them,
so they only have to be positive and explicit. The substitution is **never
silent**: every placeholder use is reported as a `[WARN]` line with the joint
name, on stderr when `--out` is used and on stderr next to the document on
stdout. `--strict` does not promote it to an error; declaring real effort and
velocity belongs to the joint, and a run that stops for a placeholder would stop
on every manifest.

### 3.7 What a bound means here

The `*_enabled` flags decide exactly one thing: the R3 collapse. Everywhere else
the numeric `lower`/`upper` are emitted as they stand, because the IR producer
already writes the *effective* bound: a bound the CAD did not set is written as
its clamp value and the flag records that the CAD never constrained it. URDF has
no way to express "this side is unbounded", so the emitter does not invent one:
it transports the number and takes the one documented decision in R3.

## 4. Order, determinism, byte identity

* Links are emitted in IR `bodies` order, joints in IR `joints` order.
* No timestamp, no absolute path, no environment-dependent value, no dictionary
  iteration in the output path.
* Floats are rounded to 9 decimals and formatted with a fixed rule, so the same
  IR always produces the same bytes on any machine.

`tests/fixtures/urdf_skeleton_sample.urdf` is that byte-exact output for
`tests/fixtures/cad_manifest_urdf_skeleton.json`, and CI regenerates it and
diffs: a change in the emitter's mind shows up as a failing diff, not as a model
that quietly moved.

## 5. The gate

Two layers, same severity model as the SDF gate.

**Structural validator** (`cad.urdf_skeleton.collect_errors`) — always runs
before anything is written, and fails the run. It re-parses the emitted XML and
checks: `robot` root with a name, at least one link, unique link and joint names,
a joint type inside the URDF vocabulary, `parent`/`child` resolving to existing
links, no self-loop, `<limit>` present (with `effort` and `velocity`) on bounded
joints and absent on fixed ones, no bounds on continuous joints, `<axis>` present
and non-null on movable joints, the three- and six-number vector shapes, and the
leanness rule of §1 — a forbidden element is an **error**, not a warning. Finally
it walks the joint graph: no root means a cycle, and more than one root is
reported when the caller asks for a single tree.

**`check_urdf`** (urdfdom) — the reference parser, used by the test suite when
the ROS CLI is on `PATH`, and skipped with an explicit reason when it is not (CI
is headless). The embedded validator is the always-on gate; `check_urdf` is the
independent second opinion.

Severities: `errors` are things a parser or `robot_state_publisher` rejects and
always fail the run; `warnings` are things that are legal but worth saying
(a placeholder effort, more than one root for a manifest the CAD did not
connect). Warnings are printed, never suppressed, and `--strict` turns the
multi-root warning into a failure — a body that no joint touches stays in the
document as its own root, and the emitter says so instead of bolting it to the
first link it finds.

## 6. Equivalence with the IR (what the tests actually prove)

The acceptance criterion for C6 is that the tree is *equivalent* to the IR, not
that it looks similar. `tests/test_urdf_skeleton.py` proves it field by field,
against the IR, on the golden manifest:

1. same link set, in the same order as `bodies`;
2. same joint set, in the same order as `joints`;
3. same `parent`/`child` for every joint — the transport is an identity on
   topology, so an edge can never be rewired;
4. same axis for every movable joint, after normalisation, and a unit vector;
5. same pose7: `xyz` equal to the translation and `rpy` reconstructing the
   quaternion within `1e-9`;
6. the joint types match a table written out *in the test* (§3.1/§3.2), not
   re-derived from the emitter — a test that shares the code under test proves
   nothing;
7. the document contains exactly the tags `robot`, `link`, `joint`, `parent`,
   `child`, `origin`, `axis`, `limit` — the leanness assertion of §1, stated as
   an equality over the tag set rather than a search for known offenders;
8. one connected root, so `robot_state_publisher` can build it.

The same tests run the emitter over the committed C4 fixture
(`cad_manifest_sample_assembly.json`). Its `float` body is touched by no joint in
the IR, so the skeleton has two roots and the run reports it as a warning: the
emitter does not invent the missing joint to make the file tidier. The strict
mode CI uses would reject that manifest, which is the correct outcome until the
CAD connects the body or the author says how.

## 7. Failure modes

| Symptom | Cause | What to do |
|---|---|---|
| `UrdfSkeletonError: … fc_type 'X' non e' mappabile` | joint type outside §3.2 | add the type to the table with its DOF, or declare `urdf_type` on the joint |
| `… asse nullo` | an IR joint with a `(0, 0, 0)` axis | fix the export: a joint with no axis has no DOF |
| `… quaternione nullo` | an IR pose with a zero quaternion | fix the export |
| `[WARN] effort/velocity placeholder` | the joint declares neither | put `effort`/`velocity` on the joint if the values matter for your controller |
| `il grafo ha N radici (…)` in `--strict` | a body that no joint attaches | connect it in the CAD, or accept the warning and use the non-strict mode |
| `robot_state_publisher` refuses the tree | the same, non-strict | as above; ROS needs a single root |

## 8. Non-goals

* No geometry, no dynamics, no materials: those are `cad.sdf_artifacts`.
* No `xacro` macros, no ROS package metadata, no URDF parameters: the emitter
  writes one plain document, and there is one of them.
* No second naming implementation: names come from the IR, which comes from
  `cad.naming`.
* No `<floating>` or `<planar>` joints: neither is in the IR's vocabulary, and a
  free body is a CAD modelling statement (`activated == false` gives a rigid
  edge), not something an emitter invents.
