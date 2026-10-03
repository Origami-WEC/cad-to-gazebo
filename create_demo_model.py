#!/usr/bin/env python3
"""
create_demo_model.py  --  Generatore di modello WEC dimostrativo (opzionale)

A cosa serve
------------
Permette di testare l'intera pipeline (collision hull, merge incastri,
massa/inerzia, SDF, mondi, telemetria) SENZA FreeCAD: genera un point
absorber elementare (scafo cilindrico + stelo PTO) nel nuovo schema
manifest (bodies + joints), identico a quello prodotto da freecad_exporter.py.

Il file viene creato SOLO se config/cad_manifest.json non esiste gia'.

Geometria demo (metri) -- topologia RM3
---------------------------------------
  hull       : cilindro r=0.70 m, h=3.60 m, asse Z, rho=550
  pto_shaft  : piastra r=0.40 m, h=0.30 m (heave plate), rho=2700
  pto_joint  : Slider (prismatic) Z hull -> pto_shaft, corsa [-1.5, 1.5] m

Uso: python3 create_demo_model.py [--force]
"""

import datetime
import json
import os
import sys

import trimesh

from ws_root import workspace_root

WS_DIR = workspace_root()
MANIFEST = os.path.join(WS_DIR, "config", "cad_manifest.json")

HULL_RADIUS = 0.70
HULL_HEIGHT = 3.60
HULL_Z_CENTER = -1.20
HULL_DENSITY = 550.0

SHAFT_RADIUS = 0.40
SHAFT_HEIGHT = 0.30
SHAFT_Z_CENTER = 0.00
SHAFT_DENSITY = 2700.0

HULL_COM_M = [0.0, 0.0, HULL_Z_CENTER]
SHAFT_COM_M = [0.0, 0.0, SHAFT_Z_CENTER]


def _write(path, mesh):
    mesh.export(path, file_type="stl")


def _solid_entry(name, label, mesh, density, com=None, material=None):
    volume = mesh.volume
    if com is None:
        com = [float(c) for c in mesh.center_mass]
    mesh.density = density
    inertia = mesh.moment_inertia
    return {
        "name": name,
        "label": label,
        "density_kg_m3": density,
        "density_source": "demo",
        "reference_mass_kg": round(volume * density, 6),
        "reference_volume_m3": round(volume, 9),
        "reference_com_m": [round(float(c), 6) for c in com],
        "reference_inertia_kg_m2": [
            [round(float(inertia[i, j]), 12) for j in range(3)]
            for i in range(3)
        ],
        "visual_mesh": "model://meshes/visual/{}.stl".format(name),
        "collision_mesh": "model://meshes/collision/{}_convex.stl".format(name),
        "stl_path": os.path.join("meshes", "visual", name + ".stl"),
        "stl_collision_path":
            os.path.join("meshes", "collision", name + "_convex.stl"),
        "material": material or {"card": "demo", "diffuse": [0.6, 0.6, 0.6, 1.0],
                                 "ambient": [0.6, 0.6, 0.6, 1.0]},
        "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "num_vertices": len(mesh.vertices),
    }


def create_demo_model(force=False):
    if os.path.exists(MANIFEST) and not force:
        print("[demo] config/cad_manifest.json gia' presente: nessun modello demo generato.")
        return 0

    hull = trimesh.creation.cylinder(
        radius=HULL_RADIUS, height=HULL_HEIGHT,
        sections=64, transform=trimesh.transformations.identity_matrix())
    shaft = trimesh.creation.cylinder(radius=SHAFT_RADIUS, height=SHAFT_HEIGHT,
                                      sections=48)
    shaft.apply_translation([0.0, 0.0, SHAFT_Z_CENTER])

    vis_dir = os.path.join(WS_DIR, "meshes", "visual")
    os.makedirs(vis_dir, exist_ok=True)
    _write(os.path.join(vis_dir, "hull__shell.stl"), hull)
    _write(os.path.join(vis_dir, "pto_shaft__mass.stl"), shaft)

    hull_solid = _solid_entry("hull__shell", "shell", hull, HULL_DENSITY,
                              HULL_COM_M)
    shaft_solid = _solid_entry("pto_shaft__mass", "mass", shaft, SHAFT_DENSITY,
                               SHAFT_COM_M)

    manifest = {
        "meta": {
            "generator": "create_demo_model.py",
            "source_document": "demo (nessun CAD)",
            "source_file": "",
            "source_hash": "",
            "units": "SI (m, kg, rad)",
            "created": datetime.datetime.now().isoformat(timespec="seconds"),
            "root": "hull",
        },
        "bodies": [
            {
                "name": "hull",
                "label": "hull",
                "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                "solids": [hull_solid],
                "num_solids": 1,
            },
            {
                "name": "pto_shaft",
                "label": "pto_shaft",
                "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                "solids": [shaft_solid],
                "num_solids": 1,
            },
        ],
        "joints": [
            {
                "name": "pto_joint",
                "label": "pto_joint",
                "fc_type": "Slider",
                "parent": "hull",
                "child": "pto_shaft",
                "origin": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                "axis": [0.0, 0.0, 1.0],
                "limits": {
                    "lower": -1.5,
                    "upper": 1.5,
                    "lower_enabled": True,
                    "upper_enabled": True,
                    "unit": "m",
                    "offset_m": 0.0,
                },
                "activated": True,
                "distance": 0.0,
                "distance2": 0.0,
            }
        ],
        "skipped_joints": [],
    }
    os.makedirs(os.path.dirname(MANIFEST), exist_ok=True)
    with open(MANIFEST, "w") as fh:
        json.dump(manifest, fh, indent=2)

    try:
        from cad import config_sync
        report = config_sync.sync_from_manifest(manifest, ws=WS_DIR)
        report.log()
    except Exception as exc:
        print("[ATTENZIONE] config_sync non riuscito: {}".format(exc))

    total = sum(b["solids"][0]["reference_mass_kg"] for b in manifest["bodies"])
    disp = total / 1025.0
    print("[demo] Modello dimostrativo creato (hull + pto_shaft + pto_joint).")
    print("[demo] Massa totale {:.1f} kg -> volume dislocato {:.3f} m^3"
          .format(total, disp))
    print("[demo] ATTENZIONE: modello placeholder per collaudo pipeline,")
    print("[demo] sostituire con l'assieme FreeCAD reale (cad/).")
    return 0


if __name__ == "__main__":
    force = "--force" in sys.argv
    sys.exit(create_demo_model(force=force))
