#!/bin/bash
# Snapshot the current build's bitstream to bitstreams/ so it survives
# the next `--build` overwrite. Names carry the git short SHA + a
# timestamp; a sidecar .info captures `git log -1` + uncommitted diff
# stat so the .bit always travels with its build recipe.
#
# Usage:  bitstreams/archive.sh [label]
# Output: bitstreams/<sha>[-dirty]_YYYY-MM-DD_HHMM[_label].bit (+ .info)

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIT_SRC="$REPO_ROOT/build/colorlight_i9plus/gateware/colorlight_i9plus.bit"
DEST_DIR="$REPO_ROOT/bitstreams"

if [ ! -f "$BIT_SRC" ]; then
    echo "ERROR: no bitstream at $BIT_SRC — run a build first" >&2
    exit 1
fi

cd "$REPO_ROOT"

SHA=$(git rev-parse --short HEAD)
if ! git diff --quiet || ! git diff --cached --quiet; then
    SHA="${SHA}-dirty"
fi
STAMP=$(date +%Y-%m-%d_%H%M)
LABEL="${1:+_${1}}"
NAME="${SHA}_${STAMP}${LABEL}"

mkdir -p "$DEST_DIR"
cp "$BIT_SRC" "$DEST_DIR/${NAME}.bit"

{
    echo "# $NAME"
    echo "# archived:  $(date -Iseconds)"
    echo "# source:    $BIT_SRC ($(stat -c '%y' "$BIT_SRC" | cut -d. -f1))"
    echo "# size:      $(stat -c '%s' "$BIT_SRC") bytes"
    echo ""
    git log -1
    echo ""
    UD=$(git diff --shortstat HEAD || true)
    echo "## uncommitted diff vs HEAD: ${UD:-none}"
    git diff --stat HEAD || true
} > "$DEST_DIR/${NAME}.info"

echo "$DEST_DIR/${NAME}.bit"
echo "$DEST_DIR/${NAME}.info"
