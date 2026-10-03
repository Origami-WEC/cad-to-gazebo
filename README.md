# cad-to-gazebo — FreeCAD to SDF/Gazebo converter

Turns the mechanical design into a simulatable model: **FreeCAD assemblies →
`cad_manifest.json` → collision hulls / visual meshes → SDF**, consumed by the
Gazebo pipeline in `maritime_ws`.

## Layout

| File | What |
|---|---|
| `freecad_exporter.py` | Export from `.FCStd` via `freecadcmd` (cached on the file's SHA-256) |
| `cad_assembly.py` | Assembly handling |
| `export_manifest.py` | Regenerates `config/cad_manifest.json`, meshes and derived configs |
| `config_sync.py` | `cad_manifest.json` → `pipeline.json`, `hydrodynamics_params.json`, `mooring.json` |
| `material_props.py` | Material properties |
| `create_demo_model.py` | Demo fallback when no CAD is available |

## Chain

```
cad/*.FCStd  --freecadcmd-->  config/cad_manifest.json + meshes/visual
cad_manifest.json  --sync-->  config/pipeline.json
                              config/hydrodynamics_params.json
                              config/mooring.json
```

## Part of

The Origami simulation stack — upstream of `maritime_ws`, downstream of the CAD
in `Mechanism`.
