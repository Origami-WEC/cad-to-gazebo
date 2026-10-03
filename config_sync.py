#!/usr/bin/env python3
"""config_sync.py -- Allinea i config dipendenti da geometria/masse al manifest CAD.

Quando `config/cad_manifest.json` viene rigenerato (freecad_exporter.py o
create_demo_model.py), questo modulo riallinea gli altri file di config:

  config/pipeline.json
    - model.root_link              <- meta.root del manifest
    - model.joints[]               <- topologia + limiti esatti dal CAD
                                       (lower/upper/lower_enabled/
                                       upper_enabled/unit, SI)
    - model.joint_dynamics.*       <- una chiave per joint mobile (dinamica
                                       preservata per nome, default per i nuovi)
  config/hydrodynamics_params.json
    - enable_links                 <- intersezione con i body/link del manifest
  config/mooring.json
    - validazione fairlead (link esistente, pos_local nel bbox del link)
    - break_tension_n / massa cavi  <- solo con write_mooring_derived=True
                                       (formule euristiche in mooring.heuristics)

La topologia dei joint resta di pertinenza del CAD: qui non si inventano
parent/child/limiti, si copiano dal manifest e si tiene solo la DINAMICA
(damping, friction, molle, finecorsa) come overlay tarabile a mano.

Uso
---
  python3 src/cad/config_sync.py
  python3 src/cad/config_sync.py --write-mooring-derived
  python3 src/cad/config_sync.py --manifest PATH --ws PATH
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

if __package__ in (None, ""):
    _src = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _src not in sys.path:
        sys.path.insert(0, _src)

from cad import cad_assembly as ca

DEFAULT_WS = None  # risolto pigramente da ws_root

DEFAULT_JOINT_DYNAMICS = {
    "damping": 0.0,
    "friction": 0.0,
    "spring_stiffness": 0.0,
    "spring_reference": 0.0,
    "limit_stiffness": 1.0e6,
    "limit_dissipation": 1500.0,
}

# Default pre-2026: limit_stiffness/dissipation a 0 disabilitavano i
# finecorsa. In fase di merge un valore pari al vecchio default viene
# sostituito dal nuovo (finecorsa attivi di fabbrica).
_LEGACY_ZERO_DEFAULTS = {"limit_stiffness", "limit_dissipation"}

DYNAMICS_KEYS = tuple(DEFAULT_JOINT_DYNAMICS.keys())

DEFAULT_HEURISTICS = {
    "break_tension_safety_factor": 2.5,
    "max_line_mass_fraction_of_device": 0.05,
    "gravity_m_s2": 9.81,
}

BREAK_TENSION_REL_TOL = 0.20
BBOX_MARGIN_RATIO = 0.05


@dataclass
class SyncReport:
    updated: List[str] = field(default_factory=list)
    preserved: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    def log(self) -> None:
        for item in self.updated:
            print("[config-sync] aggiornato: {}".format(item))
        for item in self.preserved:
            print("[config-sync] preservato: {}".format(item))
        for item in self.warnings:
            print("[ATTENZIONE] {}".format(item))


def _resolve_ws(ws: Optional[str] = None) -> str:
    if ws:
        return os.path.abspath(ws)
    from ws_root import workspace_root
    return workspace_root()


def _load_json(path: str) -> Optional[dict]:
    if not os.path.exists(path):
        return None
    with open(path) as fh:
        return json.load(fh)


def _write_json(path: str, data: dict) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as fh:
        json.dump(data, fh, indent=2, ensure_ascii=False)
        fh.write("\n")


def _cfg_paths(ws: str) -> Dict[str, str]:
    cfg = os.path.join(ws, "config")
    return {
        "pipeline": os.path.join(cfg, "pipeline.json"),
        "hydro": os.path.join(cfg, "hydrodynamics_params.json"),
        "mooring": os.path.join(cfg, "mooring.json"),
        "manifest": os.path.join(cfg, "cad_manifest.json"),
    }


def total_mass_kg(manifest: dict) -> float:
    """Massa totale di riferimento dal manifest (assembly o legacy)."""
    total = 0.0
    for body in manifest.get("bodies") or []:
        for solid in body.get("solids") or []:
            total += float(solid.get("reference_mass_kg") or 0.0)
    for link in manifest.get("links") or []:
        total += float(link.get("reference_mass_kg") or 0.0)
    return total


def body_names(manifest: dict) -> List[str]:
    names = [b["name"] for b in manifest.get("bodies") or [] if b.get("name")]
    names += [l["name"] for l in manifest.get("links") or [] if l.get("name")]
    return names


def movable_joints(manifest: dict) -> List[dict]:
    """Joint del manifest che diventano DOF SDF (no Fixed, no unmapped)."""
    out = []
    for j in manifest.get("joints") or []:
        if not j.get("activated", True):
            continue
        try:
            mapping = ca.map_joint_type(j.get("fc_type", ""))
        except ValueError:
            continue
        if mapping.mapped and not mapping.merge:
            out.append(j)
    return out


def _limits_snapshot(joint: dict) -> dict:
    """Copia esatta dei limiti SI del manifest (valori + flag + unita')."""
    return ca.JointLimits.from_dict(joint.get("limits")).to_dict()


def _sdf_type(joint: dict) -> str:
    return ca.map_joint_type(joint.get("fc_type", "")).sdf_type


def _merge_dynamics(existing: Optional[dict]) -> dict:
    merged = dict(DEFAULT_JOINT_DYNAMICS)
    if isinstance(existing, dict):
        for key in DYNAMICS_KEYS:
            if key in existing and existing[key] is not None:
                value = existing[key]
                if key in _LEGACY_ZERO_DEFAULTS and value == 0.0:
                    continue  # vecchio default "disabilitato": usa il nuovo
                merged[key] = value
    return merged


def _migrate_entry_dynamics(entry: dict, current: dict,
                            report: SyncReport) -> dict:
    """Importa i valori di dinamica rimasti nello snapshot joints[].

    Transizione one-time dal formato vecchio (dinamica duplicata in
    joints[]): i valori nello snapshot vincono, perche' e' lì che venivano
    editati. Dopo il primo sync joints[] non contiene piu' quelle chiavi e
    joint_dynamics resta l'unica fonte.
    """
    merged = dict(current or {})
    moved = []
    for key in DYNAMICS_KEYS:
        value = entry.get(key)
        if value is None:
            continue
        if merged.get(key) != value:
            merged[key] = value
            moved.append(key)
    if moved:
        report.warnings.append(
            "pipeline.model.joints.{}: {} migrato in joint_dynamics "
            "(lo snapshot joints[] e' read-only)".format(
                entry.get("name", "?"), ", ".join(moved)))
    return merged


def sync_pipeline_joints(manifest: dict, pipeline: dict,
                         report: SyncReport) -> bool:
    """Rigenera model.joints / model.joint_dynamics dai joint del CAD."""
    joints = movable_joints(manifest)
    if not joints and not (manifest.get("joints") or []):
        if "links" in manifest and "bodies" not in manifest:
            report.preserved.append("pipeline.model.joints (manifest legacy)")
            return False
        report.preserved.append("pipeline.model.joints (nessun joint mobile)")
        return False

    model = pipeline.setdefault("model", {})
    old_dyn = model.get("joint_dynamics") or {}
    if not isinstance(old_dyn, dict):
        old_dyn = {}
    old_joints = {e.get("name"): e for e in (model.get("joints") or [])
                  if isinstance(e, dict) and e.get("name")}

    names = [j["name"] for j in joints]
    new_dyn = {}
    if "$comment" in old_dyn:
        new_dyn["$comment"] = old_dyn["$comment"]
    else:
        new_dyn["$comment"] = (
            "UNICO posto editabile per la dinamica dei joint: damping, "
            "friction, molle, finecorsa. Le chiavi di model.joints[] sono "
            "rigenerate dal CAD e i valori qui presenti sono preservati.")
    for joint in joints:
        name = joint["name"]
        dyn = _merge_dynamics(old_dyn.get(name))
        if name in old_joints:
            dyn = _migrate_entry_dynamics(old_joints[name], dyn, report)
        if name in old_dyn and isinstance(old_dyn[name], dict):
            report.preserved.append("pipeline.model.joint_dynamics.{}".format(name))
        else:
            report.updated.append("pipeline.model.joint_dynamics.{}".format(name))
        new_dyn[name] = dyn
    for key, value in old_dyn.items():
        if key.startswith("$"):
            continue
        if key not in new_dyn:
            report.warnings.append(
                "pipeline.model.joint_dynamics.{} rimosso: nessun joint "
                "corrispondente nel manifest".format(key))
    model["joint_dynamics"] = new_dyn

    new_joints = []
    for joint in joints:
        name = joint["name"]
        new_joints.append({
            "name": name,
            "type": _sdf_type(joint),
            "parent": joint["parent"],
            "child": joint["child"],
            "axis": [float(a) for a in (joint.get("axis") or [0.0, 0.0, 1.0])],
            "limits": _limits_snapshot(joint),
        })
    if model.get("joints") != new_joints:
        model["joints"] = new_joints
        report.updated.append(
            "pipeline.model.joints ({} joint)".format(len(new_joints)))
    else:
        report.preserved.append("pipeline.model.joints")
    model["$comment_joints_generated"] = (
        "Snapshot READ-ONLY rigenerato da config/cad_manifest.json: "
        "topologia e limiti (valori assoluti FreeCAD + offset_m) dal CAD. "
        "La dinamica si imposta SOLO in model.joint_dynamics.<nome>.")
    return True


def sync_root_link(manifest: dict, pipeline: dict, report: SyncReport) -> None:
    root = (manifest.get("meta") or {}).get("root")
    if not root:
        return
    model = pipeline.setdefault("model", {})
    current = model.get("root_link")
    if current == root:
        report.preserved.append("pipeline.model.root_link")
        return
    model["root_link"] = root
    report.updated.append("pipeline.model.root_link = {}".format(root))


def sync_enable_links(manifest: dict, hydro: dict, report: SyncReport) -> None:
    valid = set(body_names(manifest))
    if not valid:
        return
    current = list(hydro.get("enable_links") or [])
    kept = [n for n in current if n in valid]
    dropped = [n for n in current if n not in valid]
    for name in dropped:
        report.warnings.append(
            "hydrodynamics_params.enable_links: '{}' rimosso (non e' un "
            "body/link del manifest)".format(name))
    if kept != current:
        hydro["enable_links"] = kept
        report.updated.append("hydrodynamics_params.enable_links = {}".format(kept))
    else:
        report.preserved.append("hydrodynamics_params.enable_links")


def _pose7_to_matrix(pose: Sequence[float]):
    if pose is None:
        return ca.pose_to_matrix([0, 0, 0, 0, 0, 0, 1])
    return ca.pose_to_matrix(list(pose))


def body_aabb(body: dict, ws: str) -> Optional[Tuple[List[float], List[float]]]:
    """AABB del body nel frame del body, dai mesh visual dei solidi."""
    try:
        import trimesh
    except ImportError:
        return None
    mins = None
    maxs = None
    for solid in body.get("solids") or []:
        rel = solid.get("stl_path") or ""
        path = os.path.join(ws, rel) if rel else ""
        if not path or not os.path.exists(path):
            continue
        try:
            mesh = trimesh.load(path, force="mesh", process=False)
        except Exception:
            continue
        verts = getattr(mesh, "vertices", None)
        if verts is None or len(verts) == 0:
            continue
        t = _pose7_to_matrix(solid.get("pose"))
        rot, trans = t[:3, :3], t[:3, 3]
        world = verts @ rot.T + trans
        lo = world.min(axis=0)
        hi = world.max(axis=0)
        mins = lo if mins is None else np_minimum(mins, lo)
        maxs = hi if maxs is None else np_maximum(maxs, hi)
    if mins is None:
        return None
    return (list(mins), list(maxs))


def np_minimum(a, b):
    return [min(float(x), float(y)) for x, y in zip(a, b)]


def np_maximum(a, b):
    return [max(float(x), float(y)) for x, y in zip(a, b)]


def validate_mooring_geometry(manifest: dict, mooring: dict, ws: str,
                              report: SyncReport) -> None:
    valid = set(body_names(manifest))
    aabbs = {}
    for body in manifest.get("bodies") or []:
        box = body_aabb(body, ws)
        if box is not None:
            aabbs[body["name"]] = box
    for link in manifest.get("links") or []:
        box = body_aabb({"solids": [link]}, ws)
        if box is not None:
            aabbs[link["name"]] = box

    for fl in mooring.get("fairleads") or []:
        name = fl.get("name", "?")
        link = fl.get("link", "hull")
        if valid and link not in valid:
            report.warnings.append(
                "mooring.fairlead '{}': link '{}' non presente nel manifest "
                "(body: {})".format(name, link, ", ".join(sorted(valid))))
            continue
        box = aabbs.get(link)
        if box is None:
            continue
        mins, maxs = box
        pos = [float(v) for v in (fl.get("pos_local") or [0, 0, 0])]
        if len(pos) != 3:
            report.warnings.append(
                "mooring.fairlead '{}': pos_local non e' [x, y, z]".format(name))
            continue
        span = [maxs[i] - mins[i] for i in range(3)]
        margin = [s * BBOX_MARGIN_RATIO for s in span]
        outside = []
        for i, axis in enumerate("xyz"):
            lo = mins[i] - margin[i]
            hi = maxs[i] + margin[i]
            if pos[i] < lo or pos[i] > hi:
                outside.append("{}={:.3f} fuori [{:.3f}, {:.3f}]"
                               .format(axis, pos[i], lo, hi))
        if outside:
            report.warnings.append(
                "mooring.fairlead '{}': pos_local {} fuori dal bbox di "
                "'{}' (+/-{:.0f}%): {}".format(
                    name, pos, link, 100 * BBOX_MARGIN_RATIO,
                    "; ".join(outside)))


def derived_break_tension(manifest: dict, mooring: dict) -> float:
    heur = mooring.get("heuristics") or {}
    factor = float(heur.get("break_tension_safety_factor",
                            DEFAULT_HEURISTICS["break_tension_safety_factor"]))
    g = float(heur.get("gravity_m_s2",
                       DEFAULT_HEURISTICS["gravity_m_s2"]))
    return factor * total_mass_kg(manifest) * g


def _mean_line_length(mooring: dict) -> float:
    import numpy as np
    from mooring import mooring_build

    pose = mooring.get("model_spawn_pose") or [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    origin, rot = mooring_build.spawn_transform(pose)
    fairleads = {f.get("name"): f for f in mooring.get("fairleads") or []}
    lengths = []
    for line in mooring.get("lines") or []:
        fl = fairleads.get(line.get("fairlead"))
        if not fl:
            continue
        pos = np.asarray(fl.get("pos_local") or [0, 0, 0], dtype=float)
        anchor = np.asarray(line.get("anchor_world") or [0, 0, 0], dtype=float)
        dist = float(np.linalg.norm(anchor - (rot @ pos + origin)))
        if dist > 0:
            lengths.append(dist)
    return sum(lengths) / len(lengths) if lengths else 0.0


def update_mooring_derived(manifest: dict, mooring: dict, report: SyncReport,
                           write: bool) -> None:
    heur = mooring.setdefault("heuristics", dict(DEFAULT_HEURISTICS))
    for key, value in DEFAULT_HEURISTICS.items():
        heur.setdefault(key, value)

    mass = total_mass_kg(manifest)
    if mass <= 0.0:
        report.warnings.append(
            "mooring: massa totale nulla nel manifest, salto derive ormeggio")
        return

    derived = derived_break_tension(manifest, mooring)
    current = float(mooring.get("break_tension_n") or 0.0)
    if write:
        if abs(current - derived) > 1e-6:
            mooring["break_tension_n"] = round(derived, 1)
            report.updated.append(
                "mooring.break_tension_n = {:.1f} N (k={:g} * m={:.1f} kg * g)"
                .format(derived,
                        float(heur["break_tension_safety_factor"]),
                        mass))
        else:
            report.preserved.append("mooring.break_tension_n")
    else:
        if derived > 0 and abs(current - derived) / derived > BREAK_TENSION_REL_TOL:
            report.warnings.append(
                "mooring.break_tension_n={:.1f} N vs euristica {:.1f} N "
                "(k={:g} * m={:.1f} kg * g): desincronizzato ({:+.0f}%). "
                "Riscrivi con --write-mooring-derived.".format(
                    current, derived,
                    float(heur["break_tension_safety_factor"]), mass,
                    100 * (current / derived - 1.0)))

    lt = mooring.get("line_type") or {}
    mu = float(lt.get("mass_per_m_kg_m") or 0.0)
    n_lines = len(mooring.get("lines") or [])
    mean_len = _mean_line_length(mooring)
    line_mass = n_lines * mean_len * mu
    limit = float(heur["max_line_mass_fraction_of_device"]) * mass
    if line_mass > limit > 0:
        report.warnings.append(
            "mooring.line_type.mass_per_m_kg_m={:g}: massa cavi stimata "
            "{:.1f} kg ({:g} x {:.1f} m) > {:.0f}% della massa device "
            "({:.1f} kg)".format(mu, line_mass, n_lines, mean_len,
                                 100 * float(
                                     heur["max_line_mass_fraction_of_device"]),
                                 mass))


def validate_joint_dynamics(manifest: dict, pipeline: dict,
                            report: SyncReport) -> None:
    """Warning se spring_reference (coordinate FreeCAD) e' fuori corsa."""
    dyn_all = (pipeline.get("model") or {}).get("joint_dynamics") or {}
    for joint in movable_joints(manifest):
        name = joint["name"]
        dyn = dyn_all.get(name) or {}
        ref = dyn.get("spring_reference")
        if ref is None:
            continue
        limits = ca.JointLimits.from_dict(joint.get("limits"))
        if not (limits.lower_enabled and limits.upper_enabled):
            continue
        lo, hi = sorted((limits.lower, limits.upper))
        if not (lo <= float(ref) <= hi):
            report.warnings.append(
                "joint_dynamics.{}.spring_reference={} fuori dai limiti "
                "CAD [{}, {}] {}: la molla premera' sempre contro un "
                "finecorsa. Imposta il valore nella stessa scala dei limiti "
                "del joint (coordinate FreeCAD).".format(
                    name, ref, lo, hi, limits.unit))


def sync_from_manifest(manifest: Optional[dict] = None,
                       ws: Optional[str] = None,
                       write: bool = True,
                       write_mooring_derived: bool = False,
                       report: Optional[SyncReport] = None) -> SyncReport:
    """Allinea pipeline/hydro/mooring al manifest. Idempotente."""
    ws = _resolve_ws(ws)
    paths = _cfg_paths(ws)
    if manifest is None:
        manifest = _load_json(paths["manifest"])
    if not manifest:
        raise FileNotFoundError(
            "cad_manifest.json non trovato in {}".format(paths["manifest"]))
    if report is None:
        report = SyncReport()

    pipeline = _load_json(paths["pipeline"])
    if pipeline is not None:
        sync_root_link(manifest, pipeline, report)
        if "bodies" in manifest or "joints" in manifest:
            sync_pipeline_joints(manifest, pipeline, report)
        validate_joint_dynamics(manifest, pipeline, report)
        if write:
            _write_json(paths["pipeline"], pipeline)

    hydro = _load_json(paths["hydro"])
    if hydro is not None:
        sync_enable_links(manifest, hydro, report)
        if write:
            _write_json(paths["hydro"], hydro)

    mooring = _load_json(paths["mooring"])
    if mooring is not None:
        validate_mooring_geometry(manifest, mooring, ws, report)
        update_mooring_derived(manifest, mooring, report,
                               write=write_mooring_derived)
        if write:
            _write_json(paths["mooring"], mooring)

    return report


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Allinea config joint/root/enable_links/ormeggio al "
                    "manifest CAD")
    parser.add_argument("--manifest", default=None,
                        help="path a cad_manifest.json (default: config/)")
    parser.add_argument("--ws", default=None,
                        help="root del workspace (default: ws_root)")
    parser.add_argument("--write-mooring-derived", action="store_true",
                        help="riscrive break_tension_n dall'euristica di massa")
    parser.add_argument("--dry-run", action="store_true",
                        help="calcola e stampa senza scrivere i config")
    args = parser.parse_args(argv)

    ws = _resolve_ws(args.ws)
    manifest = None
    if args.manifest:
        manifest = _load_json(args.manifest)
    report = sync_from_manifest(
        manifest=manifest,
        ws=ws,
        write=not args.dry_run,
        write_mooring_derived=args.write_mooring_derived)
    report.log()
    return 0


if __name__ == "__main__":
    sys.exit(main())
