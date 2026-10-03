#!/usr/bin/env python3
"""
freecad_exporter.py  --  Macro FreeCAD / script headless (Layer 2: DATA EXTRACTION)

Legge un documento FreeCAD con un Assembly nativo (FreeCAD 1.0) e produce:
  - meshes/visual/<body>__<solid>.stl   (mesh nel frame LOCALE del body)
  - config/cad_manifest.json            (bodies + joints + materiali)

Cosa viene estratto
-------------------
  1. bodies: ogni componente dell'Assembly (App::Part / App::Link) diventa un
     body del manifest; ogni solido conserva la propria densita' e viene
     tessellato nel frame del proprio body (link-locale);
  2. joints: ogni Assembly::Joint (JointObject) con tipo FreeCAD, parent
     (=Reference2) e child (=Reference1), origin dal connector
     (Placement * Offset), asse Z del connector, limiti gia' in SI;
  3. GroundedJoint: radice dell'albero cinematico (meta.root) -- NON e' una
     saldatura al mondo, il modello resta libero di muoversi;
  4. materiali: densita' per solido con catena (vedi material_props.py):
     proprieta' Density > card con modello fisico > ereditata dai parent
     (Body/Part/Material link) > keyword card/label (WARNING) > ERRORE.
     Nessun default silenzioso: un corpo senza materiale/densita' blocca
     l'export elencando tutti i corpi irrisolti.

Unita'
------
FreeCAD lavora in millimetri: l'exporter converte TUTTO in metri (STL,
manifest) e radianti per gli angoli, cosi' la pipeline a valle e' interamente SI.
Le densita' delle card materiale sono in kg/mm^3 (unita' interne FreeCAD) e
vengono convertite in kg/m^3 da material_props.

Come si usa
-----------
  GUI:   Macro > Esegui freecad_exporter.py  (con il documento assembly aperto)
  CLI:   freecadcmd freecad_exporter.py
         MARITIME_CAD=/path/to/file.FCStd freecadcmd freecad_exporter.py

Il file CAD viene scelto con, in ordine: variabile MARITIME_CAD, documento
gia' aperto in FreeCAD, cad/wec_assembly.FCStd, cad/assembly_test.FCStd.
"""

import datetime
import hashlib
import json
import math
import os
import re
import struct
import sys

import FreeCAD as App
import Part

try:
    from cad import material_props as mp
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    ".."))
    from cad import material_props as mp

MAX_VERTICES = 300000
MM_TO_M = 1.0e-3
DEG_TO_RAD = 3.141592653589793 / 180.0
MM5_TO_KGM2 = 1.0e-15


def _log(msg):
    App.Console.PrintMessage("[exporter] {}\n".format(msg))


def _warn(msg):
    App.Console.PrintWarning("[exporter] {}\n".format(msg))


def _err(msg):
    App.Console.PrintError("[exporter] {}\n".format(msg))


def _find_workspace():
    env = os.environ.get("MARITIME_WS")
    if env and os.path.isdir(env):
        return os.path.abspath(env)
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.abspath(os.path.join(here, "..", ".."))
    if os.path.isdir(os.path.join(root, "worlds")):
        return root
    raise RuntimeError(
        "maritime_ws non trovato: imposta MARITIME_WS oppure mantieni la "
        "struttura src/cad/freecad_exporter.py dentro il workspace.")


def _read_config(ws):
    path = os.path.join(ws, "config", "pipeline.json")
    with open(path) as fh:
        return json.load(fh)


def _sanitize(name):
    name = name.strip().lower().replace(" ", "_")
    return re.sub(r"[^a-z0-9_]", "", name) or "link"


def _unique_name(base, taken):
    name = base
    idx = 2
    while name in taken:
        name = "{}_{}".format(base, idx)
        idx += 1
    taken.add(name)
    return name


def _file_hash(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return "sha256:" + h.hexdigest()


def _pose7(placement):
    p = placement.Base
    q = placement.Rotation.Q
    return [round(p.x * MM_TO_M, 9), round(p.y * MM_TO_M, 9),
            round(p.z * MM_TO_M, 9),
            round(q[0], 9), round(q[1], 9), round(q[2], 9), round(q[3], 9)]


def _chain(*placements):
    out = placements[0]
    for p in placements[1:]:
        out = out.multiply(p)
    return out


def _invert(placement):
    return placement.inverse()


def _linked_object(obj):
    linked = getattr(obj, "LinkedObject", None)
    if isinstance(linked, tuple) and linked:
        return linked[0]
    return linked


def _object_material(obj):
    """ShapeMaterial dell'oggetto, o del target se e' un App::Link."""
    sm = getattr(obj, "ShapeMaterial", None)
    if sm is not None:
        return sm
    target = _linked_object(obj) if getattr(obj, "TypeId", "") == "App::Link" \
        else None
    if target is not None:
        return getattr(target, "ShapeMaterial", None)
    return None


def _part_material_link(obj):
    """App::Material linkato da App::Part (se presente e con densita')."""
    mat = getattr(obj, "Material", None)
    if mat is None or isinstance(mat, dict):
        return None
    return mat


def _density_ctx_push(ctx, obj):
    """
    Aggiorna il contesto di ereditarieta' densita' scendendo nell'albero.
    L'ultimo DensityResolution presente vince come parent.
    """
    try:
        res = mp.resolve_density(obj=obj, label=getattr(obj, "Label", "?"))
    except mp.DensityError:
        res = None
    if res is not None and res.source and not res.source.startswith(
            ("card_keyword", "label_keyword")):
        # solo sorgenti esplicite (property/card/ereditata) propagano
        ctx["parent"] = res
        ctx["path"] = ctx.get("path", []) + [getattr(obj, "Label", "?")]
        for w in res.warnings:
            _warn(w)
    else:
        # prova il link Material di App::Part
        linked = _part_material_link(obj)
        if linked is not None:
            try:
                res_mat = mp.resolve_density(
                    obj=linked, label=getattr(obj, "Label", "?"))
                if res_mat is not None:
                    res_mat.source = "part_material_link:{}".format(
                        res_mat.source)
                    ctx["parent"] = res_mat
            except mp.DensityError:
                pass


def _material_info(obj, density_ctx):
    """Densita' + colori del solido, con ereditarieta' dai parent."""
    label = getattr(obj, "Label", "?")
    sm = _object_material(obj)
    inherited = density_ctx.get("parent") if density_ctx else None
    try:
        res = mp.resolve_density(obj=obj, inherited=inherited, label=label)
    except mp.DensityError:
        raise
    for w in res.warnings:
        _warn(w)
    info = mp.material_colors(sm)
    return res, info


def _iter_solids(container, body_pose_inv, density_ctx, collected,
                 density_errors):
    ctx = dict(density_ctx or {})
    _density_ctx_push(ctx, container)
    for child in getattr(container, "Group", []):
        if child.TypeId in ("App::Part", "App::DocumentObjectGroup",
                            "App::LinkGroup"):
            _iter_solids(child, body_pose_inv, ctx, collected, density_errors)
            continue
        if child.TypeId == "App::Link":
            linked = _linked_object(child)
            target = linked if linked is not None and hasattr(linked, "Shape") \
                else None
            if target is not None:
                _add_shape_obj(target, child, body_pose_inv, ctx,
                               collected, density_errors)
            continue
        _add_shape_obj(child, child, body_pose_inv, ctx,
                       collected, density_errors)


def _add_shape_obj(shape_owner, place_owner, body_pose_inv, density_ctx,
                   collected, density_errors):
    shape = getattr(shape_owner, "Shape", None)
    if shape is None or shape.isNull() or not shape.Solids:
        return
    label = shape_owner.Label
    try:
        res, material = _material_info(shape_owner, density_ctx)
    except mp.DensityError as exc:
        density_errors.append(str(exc))
        return
    rel = body_pose_inv.multiply(place_owner.getGlobalPlacement())
    for i, solid in enumerate(shape.Solids):
        s = solid.copy()
        s.Placement = rel
        if not s.isValid():
            _warn("solido non valido in '{}': saltato".format(label))
            continue
        collected.append({
            "owner": place_owner,
            "shape_owner": shape_owner,
            "label": label,
            "index": i,
            "shape": s,
            "density_kg_m3": res.density_kg_m3,
            "density_source": res.source,
            "material": material,
        })


def _write_binary_stl(path, vertices, facets):
    normals = []
    for tri in facets:
        v0, v1, v2 = vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
        u = [v1[i] - v0[i] for i in range(3)]
        w = [v2[i] - v0[i] for i in range(3)]
        nx = u[1] * w[2] - u[2] * w[1]
        ny = u[2] * w[0] - u[0] * w[2]
        nz = u[0] * w[1] - u[1] * w[0]
        norm = (nx * nx + ny * ny + nz * nz) ** 0.5 or 1.0
        normals.append((nx / norm, ny / norm, nz / norm))
    with open(path, "wb") as fh:
        fh.write(b"\0" * 80)
        fh.write(struct.pack("<I", len(facets)))
        pack3 = lambda v: struct.pack("<3f", v[0], v[1], v[2])
        for tri, n in zip(facets, normals):
            fh.write(pack3(n))
            for idx in tri:
                fh.write(pack3(vertices[idx]))
            fh.write(struct.pack("<H", 0))


def _export_body(part_obj, ws, cfg, taken_names, density_errors):
    name = _unique_name(_sanitize(part_obj.Label), taken_names)
    body_pose_inv = _invert(part_obj.getGlobalPlacement())

    collected = []
    density_ctx = {}
    _iter_solids(part_obj, body_pose_inv, density_ctx, collected,
                 density_errors)
    if not collected:
        _warn("body '{}' senza solidi: saltato".format(part_obj.Label))
        return None

    tol = float(cfg["export"]["tessellation_tolerance_mm"])
    vis_dir = os.path.join(ws, "meshes", "visual")
    os.makedirs(vis_dir, exist_ok=True)

    solids = []
    taken_solids = set()
    for item in collected:
        solid_name = _unique_name(_sanitize(item["label"]), taken_solids)
        stl_rel = os.path.join("meshes", "visual",
                               "{}__{}.stl".format(name, solid_name))
        shape = item["shape"]
        # FreeCAD 1.0: tessellate(deflection) 1-arg; la 2-arg float fallisce
        try:
            pts, facets = shape.tessellate(tol)
        except TypeError:
            pts, facets = shape.tessellate(float(tol))
        if len(pts) > MAX_VERTICES:
            _warn("'{}__{}' ha {} vertici: alza la tolleranza di tessellatura"
                  .format(name, solid_name, len(pts)))
        verts_m = [tuple(c * MM_TO_M for c in p) for p in pts]
        _write_binary_stl(os.path.join(ws, stl_rel), verts_m, facets)

        volume_m3 = shape.Volume * MM_TO_M ** 3
        density = item["density_kg_m3"]
        mass = volume_m3 * density
        com = shape.CenterOfMass
        com_m = (com.x * MM_TO_M, com.y * MM_TO_M, com.z * MM_TO_M)
        moi = shape.MatrixOfInertia
        inertia = [[moi.A11 * MM5_TO_KGM2 * density,
                    moi.A12 * MM5_TO_KGM2 * density,
                    moi.A13 * MM5_TO_KGM2 * density],
                   [moi.A21 * MM5_TO_KGM2 * density,
                    moi.A22 * MM5_TO_KGM2 * density,
                    moi.A23 * MM5_TO_KGM2 * density],
                   [moi.A31 * MM5_TO_KGM2 * density,
                    moi.A32 * MM5_TO_KGM2 * density,
                    moi.A33 * MM5_TO_KGM2 * density]]
        solids.append({
            "name": solid_name,
            "label": item["label"],
            "density_kg_m3": density,
            "density_source": item["density_source"],
            "reference_mass_kg": round(mass, 6),
            "reference_volume_m3": round(volume_m3, 9),
            "reference_com_m": [round(c, 9) for c in com_m],
            "reference_inertia_kg_m2": [[round(v, 12) for v in row]
                                        for row in inertia],
            "visual_mesh": "model://" + stl_rel.replace(os.sep, "/"),
            "collision_mesh":
                "model://meshes/collision/{}__{}_convex.stl".format(
                    name, solid_name),
            "stl_path": stl_rel.replace(os.sep, "/"),
            "stl_collision_path":
                "meshes/collision/{}__{}_convex.stl".format(name, solid_name),
            "material": item["material"],
            "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            "num_vertices": len(verts_m),
        })
        _log("{}__{} | rho={} ({}) | m={:.2f} kg | CoM=({:.3f}, {:.3f}, {:.3f}) m"
             .format(name, solid_name, density, item["density_source"],
                     mass, com_m[0], com_m[1], com_m[2]))

    return {
        "name": name,
        "label": part_obj.Label,
        "pose": _pose7(part_obj.getGlobalPlacement()),
        "solids": solids,
        "num_solids": len(solids),
    }


def _reference_object(ref):
    if ref is None:
        return None
    if isinstance(ref, tuple):
        holder = ref[0]
        subs = ref[1] if len(ref) > 1 else []
        if isinstance(subs, str):
            subs = [subs]
    else:
        holder, subs = ref, []
    for sub in subs:
        if not sub:
            continue
        name = str(sub).split(".")[0]
        if name and holder is not None:
            doc = holder.Document
            target = doc.getObject(name) if doc else None
            if target is not None:
                return target
    return holder


def _owning_body(target, body_names):
    seen = set()
    obj = target
    while obj is not None and id(obj) not in seen:
        seen.add(id(obj))
        if obj.Name in body_names:
            return obj.Name
        inlist = list(getattr(obj, "InList", []))
        obj = inlist[0] if inlist else None
    return None


def _is_assembly_joint(obj):
    return (hasattr(obj, "JointType") and hasattr(obj, "Reference1")
            and hasattr(obj, "Reference2"))


def _is_grounded_joint(obj):
    return hasattr(obj, "ObjectToGround")


def _joint_limits(joint, fc_type, offset_m=0.0):
    """Limiti FreeCAD (valori assoluti del DOF) + coordinata corrente.

    LengthMin/Max [mm] e AngleMin/Max [gradi] sono posizioni ASSOLUTE del
    DOF (0 = JCS coincidenti). offset_m e' il valore corrente del DOF
    ricavato dalla geometria dei connector: serve a convertire in
    spostamenti SDF (freecad_limits_to_sdf).
    """
    if fc_type in ("Slider", "Cylindrical", "Screw"):
        scale = MM_TO_M
    elif fc_type == "Revolute":
        scale = DEG_TO_RAD
    else:
        scale = 1.0
    lower_en = bool(getattr(joint, "EnableLengthMin", False)
                    or getattr(joint, "EnableAngleMin", False))
    upper_en = bool(getattr(joint, "EnableLengthMax", False)
                    or getattr(joint, "EnableAngleMax", False))
    if fc_type == "Revolute":
        lower_en = bool(getattr(joint, "EnableAngleMin", False))
        upper_en = bool(getattr(joint, "EnableAngleMax", False))
        lower = float(getattr(joint, "AngleMin", 0.0))
        upper = float(getattr(joint, "AngleMax", 0.0))
    else:
        lower = float(getattr(joint, "LengthMin", 0.0))
        upper = float(getattr(joint, "LengthMax", 0.0))
    unit = "rad" if fc_type == "Revolute" else "m"
    return {
        "lower": round(lower * scale, 9),
        "upper": round(upper * scale, 9),
        "lower_enabled": lower_en,
        "upper_enabled": upper_en,
        "unit": unit,
        "offset_m": round(offset_m, 9),
    }


def _joint_dof_current(joint, fc_type, child_obj, parent_obj):
    """Coordinata corrente del DOF (stessa convenzione di Length/AngleMin-Max).

    I Placement dei connector sono nel frame del proprio body: vanno
    portati in world con le pose globali dei body prima di confrontarli.
    Lineare: (JCS_parent - JCS_child) · asse, in metri.
    Rotazionale: angolo del JCS_parent nel JCS_child attorno all'asse, in rad.
    """
    c1 = _connector_placement(joint, 1).copy()
    c2 = _connector_placement(joint, 2).copy()
    c1_w = child_obj.getGlobalPlacement().multiply(c1)
    c2_w = parent_obj.getGlobalPlacement().multiply(c2)
    axis_world = c1_w.Rotation.multVec(App.Vector(0, 0, 1))
    delta = c2_w.Base.sub(c1_w.Base)
    d = delta.dot(axis_world) * MM_TO_M
    if fc_type != "Revolute":
        return d
    rel = c1_w.inverse().multiply(c2_w)
    m = rel.toMatrix()
    return math.atan2(m.A21, m.A11)


def _connector_placement(joint, which):
    placement = getattr(joint, "Placement{}".format(which),
                        App.Placement())
    offset = getattr(joint, "Offset{}".format(which), App.Placement())
    return _chain(placement, offset)


def _sync_dependent_configs(ws, manifest):
    """Allinea pipeline/hydro/mooring al manifest appena scritto."""
    try:
        src = os.path.join(ws, "src")
        if src not in sys.path:
            sys.path.insert(0, src)
        from cad import config_sync
        report = config_sync.sync_from_manifest(manifest, ws=ws)
        report.log()
    except Exception as exc:
        _warn("config_sync non riuscito: {}".format(exc))


def _joint_entry(joint, body_names):
    ref1 = _reference_object(joint.Reference1)
    ref2 = _reference_object(joint.Reference2)
    child = _owning_body(ref1, body_names) if ref1 is not None else None
    parent = _owning_body(ref2, body_names) if ref2 is not None else None
    if not parent or not child:
        _warn("joint '{}': reference non risolvibili (parent={}, child={})"
              .format(joint.Label, parent, child))
        return None

    fc_type = str(joint.JointType)
    doc = joint.Document
    child_obj = doc.getObject(child)
    parent_obj = doc.getObject(parent)
    offset_m = _joint_dof_current(joint, fc_type, child_obj, parent_obj)
    connector1 = _connector_placement(joint, 1)
    entry = {
        "name": _sanitize(joint.Label),
        "label": joint.Label,
        "fc_type": fc_type,
        "parent": parent,
        "child": child,
        "origin": _pose7(connector1),
        "axis": [0.0, 0.0, 1.0],
        "limits": _joint_limits(joint, fc_type, offset_m=offset_m),
        "activated": bool(getattr(joint, "Activated", True)),
        "distance": float(getattr(joint, "Distance", 0.0)),
        "distance2": float(getattr(joint, "Distance2", 0.0)),
    }
    return entry


def _find_components(doc):
    for obj in doc.Objects:
        if obj.TypeId == "Assembly::AssemblyObject":
            names = [child.Name for child in obj.Group
                     if child.TypeId in ("App::Part", "App::Link")]
            return {n: doc.getObject(n) for n in names}, obj
    components = {o.Name: o for o in doc.Objects
                  if o.TypeId in ("App::Part", "App::Link")
                  and not any(p.TypeId == "App::Part" for p in o.InList)}
    return components, None


def _find_ground_root(doc, body_names):
    for obj in doc.Objects:
        if _is_grounded_joint(obj):
            target = _reference_object(obj.ObjectToGround)
            if target is not None:
                owner = _owning_body(target, body_names)
                if owner:
                    return owner
    return None


def export_wec_cad_pipeline():
    ws = _find_workspace()
    cfg = _read_config(ws)

    doc = App.ActiveDocument
    source_path = os.environ.get("MARITIME_CAD", "")
    if source_path:
        doc = App.openDocument(source_path)
    elif doc is None:
        for rel in (os.path.join("cad", "wec_assembly.FCStd"),
                    os.path.join("cad", "assembly_test.FCStd")):
            path = os.path.join(ws, rel)
            if os.path.exists(path):
                doc = App.openDocument(path)
                source_path = path
                break
    else:
        source_path = getattr(doc, "FileName", "") or ""
    if doc is None:
        _err("nessun documento: apri l'assembly oppure imposta MARITIME_CAD")
        return None
    if not source_path:
        source_path = getattr(doc, "FileName", "") or ""

    components, _assembly = _find_components(doc)
    if not components:
        _err("nessun App::Part/App::Link nel documento '{}'".format(doc.Name))
        return None

    taken = set()
    bodies = []
    fc_to_body = {}
    density_errors = []
    for fc_name in sorted(components, key=lambda n: components[n].Label):
        obj = components[fc_name]
        body = _export_body(obj, ws, cfg, taken, density_errors)
        if body:
            bodies.append(body)
            fc_to_body[fc_name] = body["name"]

    if density_errors:
        _err("EXPORT INTERROTTO: {} corpo/i senza materiale o densita' "
             "associati:".format(len(density_errors)))
        for msg in density_errors:
            _err("  - " + msg)
        _err("Assegna la proprieta' Density (float, kg/m^3) sul Body/Part "
             "oppure una card materiale con modello fisico Density.")
        return None

    if not bodies:
        _err("nessun body con solidi nel documento '{}'".format(doc.Name))
        return None

    joints = []
    taken_joints = set()
    skipped = []
    for obj in doc.Objects:
        if _is_grounded_joint(obj) or not _is_assembly_joint(obj):
            continue
        entry = _joint_entry(obj, set(fc_to_body))
        if entry is None:
            skipped.append({"label": obj.Label,
                            "reason": "reference irrisolvibili"})
            continue
        entry["parent"] = fc_to_body[entry["parent"]]
        entry["child"] = fc_to_body[entry["child"]]
        entry["name"] = _unique_name(entry["name"], taken_joints)
        joints.append(entry)

    root = _find_ground_root(doc, set(fc_to_body))
    root_body = fc_to_body.get(root) if root else None
    if root_body is None and bodies:
        root_body = bodies[0]["name"]

    manifest = {
        "meta": {
            "generator": "freecad_exporter.py",
            "source_document": doc.Name,
            "source_file": os.path.relpath(source_path, ws)
            if source_path.startswith(ws) else source_path,
            "source_hash": _file_hash(source_path) if source_path
            and os.path.exists(source_path) else "",
            "units": "SI (m, kg, rad)",
            "created": datetime.datetime.now().isoformat(timespec="seconds"),
            "tessellation_tolerance_mm":
                cfg["export"]["tessellation_tolerance_mm"],
            "root": root_body,
        },
        "bodies": bodies,
        "joints": joints,
        "skipped_joints": skipped,
    }
    path = os.path.join(ws, "config", "cad_manifest.json")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as fh:
        json.dump(manifest, fh, indent=2)

    _log("OK: {} body, {} joint -> {}".format(len(bodies), len(joints), path))
    _sync_dependent_configs(ws, manifest)
    for j in joints:
        lim = j["limits"]
        _log("joint {} [{}] {} -> {}  DOF [{}, {}] {} @ offset {}"
             .format(j["name"], j["fc_type"], j["parent"], j["child"],
                     lim["lower"], lim["upper"], lim["unit"],
                     lim["offset_m"]))
    for s in skipped:
        _warn("joint '{}' saltato: {}".format(s["label"], s["reason"]))
    return manifest


export_wec_cad_pipeline()
