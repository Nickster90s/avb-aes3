#!/bin/bash
# Snapshot the current build's bitstream into the per-board/per-PHY tree so it
# survives the next `--build`, which always overwrites the SAME fixed path
# (build/colorlight_i9plus/gateware/colorlight_i9plus.bit). Losing a verified
# build to that overwrite has cost this project two known-good bitstreams.
#
# Layout:
#   bitstreams/<board>/<phy>/<YYYY-MM-DD_HHMM>_<sha>[-dirty][_label].bit (+ .info, .log)
#   bitstreams/known-good/<board>-<phy>[-<label>].bit   -> relative symlink (--good)
#   bitstreams/_legacy/                                 pre-convention files
#
# board/phy come from the SAME env vars that select the pins, so an archived
# .bit cannot end up filed under the wrong PHY:
#   BOARD=ext|breakout        (default ext)
#   USB2=1                    -> phy dual (BOTH ULPIs in one bitstream)
#   USB_BACKUP=1              -> phy usb2, else usb1
#
# Usage:
#   bitstreams/archive.sh [label] [--good] [--src FILE] [--log FILE]
#                         [--board B] [--phy P]
#   BOARD=breakout USB_BACKUP=1 bitstreams/archive.sh usb2-first-light --good
#
# Refuses to store a byte-identical copy of something already archived, and
# tells you where that copy is.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST_ROOT="$REPO_ROOT/bitstreams"
BIT_SRC="${BIT_SRC:-$REPO_ROOT/build/colorlight_i9plus/gateware/colorlight_i9plus.bit}"
LOG_SRC="${BUILD_LOG:-}"
LABEL=""
PROMOTE=0
BOARD_IN="${BOARD:-ext}"
PHY_IN=""

while [ $# -gt 0 ]; do
    case "$1" in
        --good|-g)   PROMOTE=1 ;;
        --src)       BIT_SRC="$2"; shift ;;
        --log)       LOG_SRC="$2"; shift ;;
        --board)     BOARD_IN="$2"; shift ;;
        --phy)       PHY_IN="$2"; shift ;;
        -h|--help)   sed -n '2,30p' "$0"; exit 0 ;;
        -*)          echo "ERROR: unknown option $1" >&2; exit 2 ;;
        *)           LABEL="$1" ;;
    esac
    shift
done

if [ -z "$PHY_IN" ]; then
    # USB2=1 builds BOTH ULPIs into one bitstream -> its own subdir. Filing a
    # dual bitstream under usb1/ makes it indistinguishable from a single-PHY
    # build, and the recipe below would then rebuild the WRONG design.
    if   [ "${USB2:-}" = "1" ];       then PHY_IN="dual"
    elif [ "${USB_BACKUP:-}" = "1" ]; then PHY_IN="usb2"
    else                                   PHY_IN="usb1"; fi
fi

BOARD=$(echo "$BOARD_IN" | tr '[:upper:]' '[:lower:]' | tr -cd 'a-z0-9_-')
PHY=$(echo "$PHY_IN"     | tr '[:upper:]' '[:lower:]' | tr -cd 'a-z0-9_-')
[ -n "$BOARD" ] || { echo "ERROR: empty --board" >&2; exit 2; }
[ -n "$PHY" ]   || { echo "ERROR: empty --phy" >&2; exit 2; }

if [ ! -f "$BIT_SRC" ]; then
    echo "ERROR: no bitstream at $BIT_SRC - run a build first" >&2
    exit 1
fi
# Absolute BEFORE the cd, or a caller-relative --src/--log breaks.
BIT_SRC=$(readlink -f "$BIT_SRC")
if [ -n "$LOG_SRC" ] && [ -f "$LOG_SRC" ]; then LOG_SRC=$(readlink -f "$LOG_SRC"); fi

cd "$REPO_ROOT"

# ---- refuse silent duplicates ------------------------------------------------
SUM=$(sha256sum "$BIT_SRC" | cut -d' ' -f1)
# NOTE no `exit` in awk and `|| true` on the producer: an early-exiting awk
# SIGPIPEs sha256sum, and `set -o pipefail` would turn that into a fatal error.
EXISTING=$( { find "$DEST_ROOT" -name '*.bit' -type f -print0 2>/dev/null \
              | xargs -0 -r sha256sum 2>/dev/null || true; } \
            | awk -v s="$SUM" '$1==s && !f {print $2; f=1}' || true)
if [ -n "$EXISTING" ]; then
    echo "ALREADY ARCHIVED (byte-identical): $EXISTING" >&2
    echo "$EXISTING"
    exit 0
fi

SHA=$(git rev-parse --short HEAD 2>/dev/null || echo nogit)
if ! git diff --quiet 2>/dev/null || ! git diff --cached --quiet 2>/dev/null; then
    SHA="${SHA}-dirty"
fi
STAMP=$(date +%Y-%m-%d_%H%M)
NAME="${STAMP}_${SHA}${LABEL:+_${LABEL}}"
DEST_DIR="$DEST_ROOT/$BOARD/$PHY"
mkdir -p "$DEST_DIR"

cp "$BIT_SRC" "$DEST_DIR/${NAME}.bit"

# ---- the recipe, so the .bit can always be rebuilt --------------------------
{
    echo "# $BOARD / $PHY / $NAME"
    echo "# archived:  $(date -Iseconds)"
    echo "# source:    $BIT_SRC ($(stat -c '%y' "$BIT_SRC" | cut -d. -f1))"
    echo "# size:      $(stat -c '%s' "$BIT_SRC") bytes"
    echo "# sha256:    $SUM"
    echo ""
    echo "## REBUILD RECIPE"
    echo "cd $REPO_ROOT"
    echo "(cd firmware && make)"
    printf 'BOARD=%s' "$BOARD"
    [ "$PHY" = "usb2" ] && printf ' USB_BACKUP=1'
    # USB2 FIRST: it selects a structurally different design (2 ULPI stacks,
    # 3rd PLL, A/B mux), so a recipe missing it rebuilds a single-PHY bitstream
    # that looks identical in every other way. 2026-10-29: the first dual build
    # archived with the recipe "BOARD=breakout ./build.sh" -- silently wrong.
    for v in USB2 USB_PHASE SAMPLE_EDGE USB_PULLUP_DATA NEXTPNR_USB_SPLIT \
             NEXTPNR_USB_REGION NEXTPNR_USB2_REGION NXT_W4 USB_SWAP_D67 \
             USB_MINIMAL USB_IDELAY_TAP USB2_FLOORPLAN PLACER SYS_PERIOD_NS; do
        eval "val=\${$v:-}"
        [ -n "$val" ] && printf ' %s=%s' "$v" "$val"
    done
    echo " ./build.sh"
    echo ""
    echo "## ENV AT ARCHIVE TIME"
    for v in BOARD USB2 USB_BACKUP USB_PHASE SAMPLE_EDGE PLACER SYS_PERIOD_NS \
             CHIPDB PRJXRAY_DB_DIR PYTHONHASHSEED; do
        eval "val=\${$v:-}"
        echo "#   $v=${val:-<unset>}"
    done
    echo ""
    git log -1 2>/dev/null || echo "(no git)"
    echo ""
    UD=$(git diff --shortstat HEAD 2>/dev/null || true)
    echo "## uncommitted diff vs HEAD: ${UD:-none}"
    git diff --stat HEAD 2>/dev/null || true
    if [ -n "$LOG_SRC" ] && [ -f "$LOG_SRC" ]; then
        echo ""
        echo "## build log highlights ($LOG_SRC)"
        grep -E 'Max frequency for clock|Device utilisation|Info: Max frequency|overused|Checksum' \
             "$LOG_SRC" 2>/dev/null | tail -25 || true
    fi
} > "$DEST_DIR/${NAME}.info"

if [ -n "$LOG_SRC" ] && [ -f "$LOG_SRC" ]; then
    cp "$LOG_SRC" "$DEST_DIR/${NAME}.log"
fi

# ---- known-good promotion ---------------------------------------------------
if [ "$PROMOTE" = "1" ]; then
    mkdir -p "$DEST_ROOT/known-good"
    LINK="$DEST_ROOT/known-good/${BOARD}-${PHY}${LABEL:+-${LABEL}}.bit"
    ln -sfn "../$BOARD/$PHY/${NAME}.bit" "$LINK"
    echo "PROMOTED: $LINK -> ../$BOARD/$PHY/${NAME}.bit"
fi

echo "$DEST_DIR/${NAME}.bit"
echo "$DEST_DIR/${NAME}.info"
