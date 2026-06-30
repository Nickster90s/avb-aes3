#!/bin/bash
# Deterministic openXC7 build: ASLR off (setarch -R) so nextpnr's pointer-hashed
# placement is reproducible run-to-run, + PYTHONHASHSEED=0 for migen/floorplan.
# Without ASLR off, the SAME seed thrashes one run and places fine the next.
export CHIPDB=/home/lisp/FPGA/demo-projects/chipdb
export PRJXRAY_DB_DIR=/home/lisp/openxc7/openxc7/opt/nextpnr-xilinx/external/prjxray-db
export PYTHONHASHSEED=0
exec setarch -R python3 avb_soc.py --build --firmware firmware/firmware.bin "$@"
