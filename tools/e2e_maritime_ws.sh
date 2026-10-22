#!/usr/bin/env bash
# e2e_maritime_ws.sh -- catena end-to-end: CAD -> IR v1 -> config -> SDF -> gate.
#
# Questo e' il comando unico che C5 promette: parte da zero (workspace
# temporaneo), genera il modello CAD di collaudo, attraversa IR e config,
# lascia che maritime_ws emetta description/model.sdf e valida l'SDF con il
# parser SDFormat. Ogni stadio che non si puo' eseguire e' un FALLIMENTO, non
# un blocco silenzioso: e' esattamente il foro che C5 chiude
# (``build_simulation.py`` saltava la validazione quando `gz` mancava).
#
# Uso:
#   tools/e2e_maritime_ws.sh                 # esercita questo checkout (default)
#   tools/e2e_maritime_ws.sh --submodule     # esercita maritime_ws/src/cad (pin)
#   tools/e2e_maritime_ws.sh --strict        # i warning SDF diventano errori
#   tools/e2e_maritime_ws.sh --keep          # non cancella il workspace
#   MARITIME_WS=/path/ws tools/e2e_maritime_ws.sh
#   CAD_SRC=/path/cad-to-gazebo tools/e2e_maritime_ws.sh
#   CAD_PY=/path/venv/bin/python tools/e2e_maritime_ws.sh
#
# Variabili:
#   MARITIME_WS  checkout di maritime_ws            (default: ../maritime_ws)
#   CAD_SRC      checkout di questo repo da provare (default: radice dello
#                script; con --submodule: $MARITIME_WS/src/cad)
#   CAD_PY       interprete Python con le dipendenze (default: .venv/bin/python)
#   TMPDIR       dove creare il workspace temporaneo

set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MWS=${MARITIME_WS:-$(cd "$ROOT/.." && pwd)/maritime_ws}
CAD_SRC=${CAD_SRC:-$ROOT}
PY=${CAD_PY:-$ROOT/.venv/bin/python}
KEEP=0
STRICT=""

usage() {
    sed -n '2,25p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

while [ $# -gt 0 ]; do
    case "$1" in
        --submodule) CAD_SRC="$MWS/src/cad" ;;
        --keep)      KEEP=1 ;;
        --strict)    STRICT="--strict" ;;
        --ws)        MWS="$2"; shift ;;
        --cad-src)   CAD_SRC="$2"; shift ;;
        --python)    PY="$2"; shift ;;
        -h|--help)   usage; exit 0 ;;
        *) echo "e2e: opzione sconosciuta: $1 (usa --help)" >&2; exit 2 ;;
    esac
    shift
done

fail() { printf '\ne2e: FALLITO -- %s\n' "$1" >&2; exit 1; }
step() { printf '\n=== %s ===\n' "$1"; }

# ---------------------------------------------------------------- precondizioni
[ -d "$MWS" ] || fail "maritime_ws non trovato in '$MWS' (imposta MARITIME_WS)"
[ -f "$MWS/src/simulation/build_simulation.py" ] \
    || fail "'$MWS' non sembra maritime_ws (manca src/simulation/build_simulation.py)"
[ -f "$MWS/src/ws_root.py" ] || fail "manca '$MWS/src/ws_root.py'"
[ -d "$CAD_SRC" ] || fail "CAD_SRC='$CAD_SRC' non esiste"
[ -f "$CAD_SRC/config_sync.py" ] || fail "CAD_SRC='$CAD_SRC' non e' questo repo"
[ -x "$PY" ] || command -v "$PY" >/dev/null \
    || fail "interprete '$PY' non eseguibile (imposta CAD_PY)"
[ -d "$MWS/config" ] || fail "manca '$MWS/config' (config di riferimento)"

echo "e2e: cad   = $CAD_SRC"
echo "e2e: mws   = $MWS"
echo "e2e: python= $PY"

# ------------------------------------------------------------------- workspace
WORK=$(mktemp -d "${TMPDIR:-/tmp}/cad-e2e-XXXXXX")
WS="$WORK/ws"
BOOT="$WORK/boot"
mkdir -p "$WS/config" "$BOOT"

cleanup() {
    if [ "$KEEP" = 1 ]; then
        printf 'e2e: workspace conservato: %s\n' "$WORK"
    else
        rm -rf "$WORK"
    fi
}
trap cleanup EXIT

# config di riferimento e mondi: la parte statica viene dal checkout maritime_ws
cp "$MWS"/config/*.json "$WS/config/"
cp -r "$MWS"/worlds "$WS/worlds"

# Il pacchetto `cad` deve essere QUELLO SCELTO (non quello che l'ambiente
# risolve per conto suo: di norma il submodule src/cad). Uno shim su PYTHONPATH
# lo rende esplicito e verificabile.
ln -s "$CAD_SRC" "$BOOT/cad"
export PYTHONPATH="$BOOT:$MWS/src${PYTHONPATH:+:$PYTHONPATH}"
export MARITIME_WS="$WS"

resolved=$("$PY" -c 'import cad, os; print(os.path.realpath(cad.__file__))' 2>/dev/null || true)
echo "e2e: cad importato = $resolved"
[ -n "$resolved" ] || fail "il pacchetto 'cad' non e' importabile con PYTHONPATH=$PYTHONPATH"
case "$resolved" in
    "$CAD_SRC"/*) ;;
    *) fail "import cad risolve '$resolved', atteso dentro '$CAD_SRC'" ;;
esac

# ------------------------------------------------------------ 1/4 CAD -> IR v1
step "1/5  CAD -> IR v1 + mesh  (create_demo_model.py)"
t0=$SECONDS
"$PY" "$ROOT/create_demo_model.py" --force > "$WORK/ir.log" 2>&1 \
    || { tail -25 "$WORK/ir.log"; fail "create_demo_model.py (vedi $WORK/ir.log)"; }
grep -q "config_sync non riuscito" "$WORK/ir.log" \
    && { tail -25 "$WORK/ir.log"; fail "config_sync fallito dentro create_demo_model"; }
[ -f "$WS/config/cad_manifest.json" ] || fail "cad_manifest.json non generato"
"$PY" - <<'PYEOF' || fail "il manifest generato non e' un CAD IR v1 valido"
import os
from cad import ir_schema
path = os.path.join(os.environ["MARITIME_WS"], "config", "cad_manifest.json")
ir = ir_schema.validate_file(path)
print("  ok: schema={} bodies={} joints={}".format(
    ir["schema"], len(ir.get("bodies", [])), len(ir.get("joints", []))))
PYEOF
echo "  ($((SECONDS - t0))s)"

# ------------------------------------------------------- 2/4 IR -> config/*.json
step "2/5  IR -> config/*.json  (config_sync.py, due volte: idempotenza)"
t0=$SECONDS
"$PY" -m cad.config_sync --ws "$WS" > "$WORK/sync1.log" 2>&1 \
    || { cat "$WORK/sync1.log"; fail "config_sync.py (1a passata)"; }
cp "$WS/config/pipeline.json" "$WORK/pipeline.1.json"
"$PY" -m cad.config_sync --ws "$WS" > "$WORK/sync2.log" 2>&1 \
    || { cat "$WORK/sync2.log"; fail "config_sync.py (2a passata)"; }
cmp -s "$WORK/pipeline.1.json" "$WS/config/pipeline.json" \
    || fail "config_sync non e' idempotente su config/pipeline.json"
echo "  config_sync idempotente"
echo "  ($((SECONDS - t0))s)"

# ------------------------------------------------------------- 3/4 IR -> model.sdf
step "3/5  IR + config -> description/model.sdf  (build_simulation.py)"
t0=$SECONDS
"$PY" "$MWS/src/simulation/build_simulation.py" > "$WORK/build.log" 2>&1 \
    || { tail -35 "$WORK/build.log"; fail "build_simulation.py (vedi $WORK/build.log)"; }
if grep -q "salto la validazione SDF" "$WORK/build.log"; then
    tail -5 "$WORK/build.log"
    fail "build-simulation ha SALTATO la validazione SDF dell'artefatto"
fi
[ -s "$WS/description/model.sdf" ] || fail "description/model.sdf non emesso"
echo "  ok: model.sdf ($(wc -c < "$WS/description/model.sdf") byte)"
echo "  ($((SECONDS - t0))s)"

# -------------------------------------------------------------------- 4/4 gate
step "4/5  gate SDF  (gz sdf -p, altrimenti parser SDFormat incluso)"
t0=$SECONDS
# shellcheck disable=SC2086
if ! "$PY" -m cad.sdf_validate $STRICT "$WS/description/model.sdf" > "$WORK/gate.log" 2>&1; then
    cat "$WORK/gate.log"
    fail "gate SDF (vedi $WORK/gate.log)"
fi
cat "$WORK/gate.log"
echo "  ($((SECONDS - t0))s)"

# ---------------------------------------- 5/5 catena reale: STEP -> IR -> SDF
# La catena 1..4 parte dal generatore di demo (create_demo_model.py), che e' il
# percorso FreeCAD.  Questa parte cinque rifa' gli stessi stadi partendo da un
# vero file STEP attraverso il ponte step_to_ir.py: e' il pezzo che rende C5
# "integrazione maritime_ws" invece di "demo verde".
step "5/5  STEP -> IR -> config -> SDF  (step_to_ir.py + contratto di esempio)"
t0=$SECONDS
STEPWS="$WORK/ws_step"
export STEP_WS="$STEPWS"
mkdir -p "$STEPWS/config"
cp "$MWS"/config/*.json "$STEPWS/config/"
cp -r "$MWS"/worlds "$STEPWS/worlds"
# Il file STEP lo si genera qui: un binario in git invecchia e nessuno sa piu'
# rifarlo.  Lo scrive la stessa fixture usata dai test OCP, ma con i nomi di un
# galleggiante (hull + pto_shaft): la config di maritime_ws bagna il link
# `hull`, e senza un body con quel nome l'equilibrio di build_simulation non ha
# nessuna mesh bagnata su cui lavorare.  I default frame/shaft restano per i
# test del ponte.
"$PY" -c "
import sys
sys.path.insert(0, '$ROOT/tests')
from step_fixtures import write_ap242_assembly
# Dimensioni realistiche in mm (scafo 1.4 x 1.4 x 3.6 m, heave plate 0.4 m sotto):
# con il rig 20x10x5 mm il volume dello scafo e' 1e-6 m^3 e la soglia della build
# (0.005 m^3) lo scarta. Il vecchio fixture passava solo perche' le mesh in mm
# gonfiavano il volume di 10^9.
write_ap242_assembly('$WORK/fixture_ap242.step',
                     root_part='hull', moving_part='pto_shaft',
                     root_size_mm=(1400.0, 1400.0, 3600.0),
                     moving_size_mm=(400.0, 400.0, 600.0),
                     moving_offset_mm=(700.0, 700.0, -600.0))
" || fail "generazione del file STEP di collaudo"

"$PY" -m cad.step_to_ir --step "$WORK/fixture_ap242.step" --ws "$STEPWS" \
    --contract "$ROOT/tests/fixtures/contract_point_absorber.json" \
    --out "$STEPWS/config/cad_manifest.json" \
    > "$WORK/step_ir.log" 2>&1 \
    || { tail -25 "$WORK/step_ir.log"; fail "step_to_ir.py (vedi $WORK/step_ir.log)"; }
[ -f "$STEPWS/config/cad_manifest.json" ] || fail "step_to_ir non ha emesso config/cad_manifest.json"
"$PY" - <<'PYEOF' || fail "il manifest del ponte non e' un CAD IR v1 valido"
import os
from cad import ir_schema
path = os.path.join(os.environ["STEP_WS"], "config", "cad_manifest.json")
ir = ir_schema.validate_file(path)
bodies = [body["name"] for body in ir.get("bodies", [])]
if not bodies:
    raise SystemExit("il ponte non ha emesso nessun body")
for body in ir["bodies"]:
    solid = body["solids"][0]
    if "reference_mass_kg" not in solid or not solid.get("stl_path"):
        raise SystemExit("body %r senza massa o senza mesh" % body["name"])
print("  ok: schema={} bodies={} joints={} (dal STEP, non dal generatore demo)".format(
    ir["schema"], bodies, len(ir.get("joints", []))))
PYEOF

"$PY" -m cad.config_sync --ws "$STEPWS" > "$WORK/step_sync.log" 2>&1 \
    || { cat "$WORK/step_sync.log"; fail "config_sync sul manifesto del ponte"; }
MARITIME_WS="$STEPWS" "$PY" "$MWS/src/simulation/build_simulation.py" \
    > "$WORK/step_build.log" 2>&1 \
    || { tail -35 "$WORK/step_build.log"; fail "build_simulation sul manifesto del ponte"; }
if grep -q "salto la validazione SDF" "$WORK/step_build.log"; then
    fail "build-simulation ha SALTATO la validazione SDF (catena STEP)"
fi
[ -s "$STEPWS/description/model.sdf" ] || fail "description/model.sdf non emesso (catena STEP)"
"$PY" -m cad.sdf_validate $STRICT "$STEPWS/description/model.sdf" > "$WORK/step_gate.log" 2>&1 \
    || { cat "$WORK/step_gate.log"; fail "gate SDF sulla catena STEP"; }
echo "  ok: STEP -> model.sdf -> gate SDF"
echo "  ($((SECONDS - t0))s)"

printf '\ne2e: OK -- catena completa CAD -> IR -> config -> SDF -> gate\n'
printf 'e2e: catena STEP -> IR -> config -> SDF -> gate (percorso reale)\n'
printf 'e2e: artefatto   %s\n' "$WS/description/model.sdf"
if [ "$KEEP" = 1 ]; then
    printf 'e2e: workspace   %s\n' "$WS"
fi