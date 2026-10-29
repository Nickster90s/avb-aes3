#!/bin/bash
# Deterministic openXC7 build: ASLR off (setarch -R) so nextpnr's pointer-hashed
# placement is reproducible run-to-run, + PYTHONHASHSEED=0 for migen/floorplan.
# Without ASLR off, the SAME seed thrashes one run and places fine the next.
#
# On success the bitstream is AUTO-ARCHIVED into bitstreams/<board>/<phy>/.
# This is deliberate and not optional: --build always overwrites the same fixed
# path, and two verified bitstreams have already been lost to that. Controls:
#   ARCHIVE_LABEL=<name>   name the archived copy (else it is tagged _auto)
#   NO_ARCHIVE=1           skip archiving (throwaway build)
# Promote to known-good/ only after HARDWARE confirms it:
#   BOARD=... USB_BACKUP=... bitstreams/archive.sh <label> --good
export CHIPDB=/home/lisp/FPGA/demo-projects/chipdb
export PRJXRAY_DB_DIR=/home/lisp/openxc7/openxc7/opt/nextpnr-xilinx/external/prjxray-db
export PYTHONHASHSEED=0

REPO_ROOT="$(cd "$(dirname "$0")" && pwd)"
LOG="$REPO_ROOT/build/last-build.log"
mkdir -p "$REPO_ROOT/build"

set -o pipefail
setarch -R python3 "$REPO_ROOT/avb_soc.py" --build \
    --firmware "$REPO_ROOT/firmware/firmware.bin" "$@" 2>&1 | tee "$LOG"
rc=$?
if [ "$rc" -ne 0 ]; then
    echo "BUILD FAILED (rc=$rc) - nothing archived. Log: $LOG" >&2
    exit "$rc"
fi

if [ "${NO_ARCHIVE:-}" = "1" ]; then
    echo "NO_ARCHIVE=1 - bitstream NOT archived; the next build will overwrite it."
else
    BUILD_LOG="$LOG" "$REPO_ROOT/bitstreams/archive.sh" \
        "${ARCHIVE_LABEL:-auto}" --log "$LOG" \
      || echo "WARNING: auto-archive FAILED - snapshot by hand before the next build!" >&2
fi
