"""naming.py -- THE naming contract implementation for the CAD IR.

Every link / body / joint / frame / anchor / sensor name that lands in the IR
(``config/cad_manifest.json``) is produced by :func:`sanitize` and made unique by
:func:`unique` **in this module and nowhere else**. There is no second
sanitiser: :mod:`cad.freecad_exporter` imports :func:`sanitize` from here
instead of carrying its own copy, and the STEP front-end routes CAD paths
through :func:`ir_name_from_path` when it has to name an IR entity.

The contract itself (who names what, the SDF <-> URDF <-> ROS mapping and the
sanitisation rules) is pinned in ``docs/naming-contract.md``; the machine
checked half of it is the ``name`` pattern in
``schema/cad_ir_v1.schema.json``, which :data:`NAME_PATTERN` mirrors and
``tests/test_naming.py`` reconciles against the schema file.

Rules (normative, enforced by tests)
------------------------------------

* An IR name is ``[a-z0-9]`` start, lowercase snake_case, no leading, trailing
  or doubled separator, at most :data:`MAX_NAME_LEN` characters.
* The name is derived from the CAD *label*; the raw label is preserved next to
  it in the IR as ``label`` for traceability, never as an identifier.
* Uniqueness inside a scope is ``<name>_2``, ``<name>_3``, ... in document
  order. The suffix is applied to the sanitised name, so the result is still a
  valid IR name.
* A name that sanitises to nothing falls back to :data:`DEFAULT_FALLBACK`
  instead of raising: an export without names is worse than an export with a
  placeholder, and the caller has already failed loudly upstream when the label
  was unreadable.

Mesh and file naming
--------------------

Merged / exported geometry files are named from the sanitised pieces joined by
a double underscore: ``meshes/visual/<body>__<solid>.stl`` and
``meshes/collision/<body>__<solid>_convex.stl``. The ``__`` separator is a
*file* convention; it is still a legal IR name character (see
:data:`NAME_PATTERN`), so a solid name may legally repeat it.
"""

from __future__ import annotations

import re
from typing import Iterable, Set

__all__ = [
    "DEFAULT_FALLBACK",
    "MESH_SEPARATOR",
    "MAX_NAME_LEN",
    "NAME_PATTERN",
    "anchor_name",
    "body_name",
    "collision_mesh_name",
    "frame_name",
    "ir_name_from_path",
    "is_valid_name",
    "joint_name",
    "sanitize",
    "sensor_name",
    "unique",
    "visual_mesh_name",
]

#: What an IR name may look like. Mirrored by ``$defs.name`` in the schema.
NAME_PATTERN = r"^[a-z0-9](?:[a-z0-9_]*[a-z0-9])?$"
_NAME_RE = re.compile(NAME_PATTERN)

#: Upper bound on an IR name. Long CAD labels are truncated, never rejected.
MAX_NAME_LEN = 64

#: Separator between the body and the solid part of a mesh file name.
MESH_SEPARATOR = "__"

#: Used when a label sanitises to the empty string.
DEFAULT_FALLBACK = "link"

_SIZE_SUFFIX_RE = re.compile(r"^(?P<base>.+?)_(?P<ordinal>[2-9][0-9]*)$")


def sanitize(raw: object, fallback: str = DEFAULT_FALLBACK) -> str:
    """Return the single canonical IR name of a CAD label.

    ``strip -> lower -> spaces become underscores -> drop anything outside
    [a-z0-9_] -> strip stray separators -> cap the length``. Idempotent:
    ``sanitize(sanitize(x)) == sanitize(x)``.
    """
    if raw is None:
        return fallback
    name = str(raw).strip().lower().replace(" ", "_")
    name = re.sub(r"[^a-z0-9_]", "", name)
    name = name.strip("_")
    if len(name) > MAX_NAME_LEN:
        name = name[:MAX_NAME_LEN].strip("_")
    return name if _NAME_RE.match(name) else fallback


def is_valid_name(name: object) -> bool:
    """True when ``name`` is already a well-formed IR name."""
    return isinstance(name, str) and bool(_NAME_RE.match(name))


def unique(name: str, taken: Set[str]) -> str:
    """Return ``name`` or the first free ``<name>_<n>`` and reserve it.

    ``taken`` is mutated: pass the per-scope set (bodies, joints, frames, ...).
    """
    candidate = name
    ordinal = 2
    while candidate in taken:
        candidate = "{}_{}".format(name, ordinal)
        ordinal += 1
    taken.add(candidate)
    return candidate


def ir_name_from_path(path: object, fallback: str = DEFAULT_FALLBACK) -> str:
    """Turn a CAD/STEP path into an IR name through the single sanitiser.

    ``full_transmission/spare_structure/profiles/311`` -> ``full_transmission_spare_structure_profiles_311``.
    Path *locators* keep their ``#N`` sibling suffix inside the CAD front-end
    (``cad.step_import``); that is an addressing scheme, not an IR name. The
    moment a path has to become an identifier it goes through here.
    """
    if path is None:
        return fallback
    # "#2" sibling suffixes are locators: strip them before sanitising.
    parts = [re.sub(r"#[0-9]+$", "", seg) for seg in str(path).strip("/").split("/")]
    joined = "_".join(seg for seg in parts if seg)
    return sanitize(joined, fallback=fallback)


def body_name(label: object) -> str:
    """IR name of a body/link: the sanitised CAD label."""
    return sanitize(label)


def joint_name(label: object) -> str:
    """IR name of a joint: the sanitised CAD label (FRC: ``j_<dof>`` when the label is empty)."""
    return sanitize(label, fallback="j_joint")


def frame_name(body: str, local: object) -> str:
    """IR name of an LCS: ``<body>_<local>``, e.g. ``hull__pto_joint``."""
    local_name = sanitize(local, fallback="lcs")
    base = sanitize(body, fallback="link")
    return "{}{}{}".format(base, MESH_SEPARATOR, local_name)


def anchor_name(kind: str, body: str, local: object) -> str:
    """IR name of an anchor: ``<kind>_<body>_<local>``."""
    return "{}_{}_{}".format(sanitize(kind, fallback="anchor"),
                             sanitize(body, fallback="link"),
                             sanitize(local, fallback="mount"))


def sensor_name(sensor_type: str, local: object) -> str:
    """IR name of a sensor: ``<type>_<local>``."""
    return "{}_{}".format(sanitize(sensor_type, fallback="sensor"),
                          sanitize(local, fallback="mount"))


def visual_mesh_name(body: str, solid: str) -> str:
    """Relative path of the visual mesh of a solid (POSIX separators)."""
    return "meshes/visual/{}{}{}.stl".format(body, MESH_SEPARATOR, solid)


def collision_mesh_name(body: str, solid: str) -> str:
    """Relative path of the collision hull of a solid (POSIX separators)."""
    return "meshes/collision/{}{}{}_convex.stl".format(body, MESH_SEPARATOR, solid)


def split_size_suffix(name: str) -> tuple:
    """Split ``hull_2`` into ``("hull", 2)``; ``("hull", 1)`` when unsuffixed.

    Exposed so a consumer can tell a duplicate that was disambiguated by
    :func:`unique` from a label that legitimately ends in a number.
    """
    match = _SIZE_SUFFIX_RE.match(name or "")
    if match is None:
        return (name, 1)
    return (match.group("base"), int(match.group("ordinal")))


def valid_unique(names: Iterable[str]) -> bool:
    """True when ``names`` are all valid IR names and pairwise distinct."""
    seen = set()
    for name in names:
        if not is_valid_name(name) or name in seen:
            return False
        seen.add(name)
    return True
