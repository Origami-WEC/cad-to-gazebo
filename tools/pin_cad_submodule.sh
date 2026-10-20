#!/usr/bin/env bash
# pin_cad_submodule.sh -- pin the src/cad submodule of a maritime_ws checkout.
#
# One command, one commit, no floating branch. The submodule entry in
# maritime_ws/.gitmodules declares `branch = main`, but the *recorded pointer*
# is a gitlink: this script moves that gitlink to an exact cad-to-gazebo commit
# and commits it, so a maritime_ws checkout always rebuilds the CAD artefacts
# from a known commit.
#
# Uso:
#   tools/pin_cad_submodule.sh <maritime_ws-checkout> <commit-sha> [branch]
#
#   <commit-sha>  un commit esistente in Origami-WEC/cad-to-gazebo (sha, tag o
#                 branch remoto: viene risolto in un oggetto locale e poi
#                 committato come hash, mai come nome mobile).
#   [branch]      branch di maritime_ws su cui committare il pin (default: la
#                 branch corrente).
#
# Esempio:
#   tools/pin_cad_submodule.sh ../maritime_ws 1a2b3c4
#
# Esito: la submodule entry `src/cad` di maritime_ws punta a <commit-sha> e il
# commit del pointer e' pronto. Il push e la PR sono passi separati.

set -euo pipefail

usage() { sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

[ $# -ge 2 ] || { usage; exit 2; }
MWS=$1
TARGET=$2
BRANCH=${3:-}

[ -d "$MWS/.git" ] || { echo "pin-cad: '$MWS' non e' un checkout git" >&2; exit 2; }
[ -e "$MWS/.gitmodules" ] || { echo "pin-cad: '$MWS/.gitmodules' assente" >&2; exit 2; }
[ -d "$MWS/src/cad" ] || { echo "pin-cad: '$MWS/src/cad' assente (submodule non inizializzato)" >&2; exit 2; }

if [ -n "$BRANCH" ]; then
    git -C "$MWS" rev-parse --verify --quiet "refs/heads/$BRANCH" >/dev/null \
        || { echo "pin-cad: branch '$BRANCH' inesistente in $MWS" >&2; exit 2; }
    git -C "$MWS" checkout "$BRANCH"
fi

git -C "$MWS" submodule update --init src/cad
git -C "$MWS/src/cad" fetch origin
git -C "$MWS/src/cad" checkout --detach "$TARGET"

# The commit the pointer will record: a hash, never a branch name.
SHA=$(git -C "$MWS/src/cad" rev-parse HEAD)

if git -C "$MWS" diff --quiet -- src/cad; then
    echo "pin-cad: src/cad punta gia' a $SHA: nessun commit necessario"
    exit 0
fi

git -C "$MWS" add src/cad
git -C "$MWS" commit -m "chore(cad): pin src/cad submodule to $SHA"

echo "pin-cad: src/cad -> $SHA"
git -C "$MWS" log --oneline -1 -- src/cad
echo "pin-cad: prossimo passo -- git -C $MWS push origin \$(git -C $MWS branch --show-current)"
