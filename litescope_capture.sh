#!/usr/bin/env bash
# LiteScope capture over CH347 JTAG (jtagbone) for the block_fifo contradiction.
#
# Resolves: level pinned full (512) + push==pop==~48k + underrun==0, yet usbmon
# says host=46979. Impossible if consumer>producer — so one of {level, first,
# usbmon} lies. This captures the actual sys-clock waveforms to settle it.
#
# Prereqs:
#   - litescope bitstream loaded (python3 avb_soc.py --litescope --load, or openocd)
#   - USB plugged, stream enabled (firmware: aaf_pkt.enable=1), host playing audio
#   - csr.csv + analyzer.csv present (see step 0 below)
set -e
cd "$(dirname "$0")"

export OPENOCD=/home/lisp/openocd/src/openocd
CFG=/home/lisp/FPGA/Colorlight-FPGA-Projects/tools/ch347.cfg
CLI=/home/lisp/litex/litescope/litescope/software/litescope_cli.py
CSR=csr.csv
ACSV=analyzer.csv

# Step 0: ensure csr.csv exists (the running build predated csr_csv= in builder).
if [ ! -f "$CSR" ]; then
    echo ">> csr.csv missing; generating via soft elaboration into /tmp/csrgen ..."
    CHIPDB=/home/lisp/FPGA/demo-projects/chipdb \
    PRJXRAY_DB_DIR=/home/lisp/openxc7/openxc7/opt/nextpnr-xilinx/external/prjxray-db \
        python3 avb_soc.py --litescope --soft-only --output-dir /tmp/csrgen
    cp /tmp/csrgen/csr.csv "$CSR"
fi

# Step 1: start the JTAG->Wishbone bridge (litex_server) if not already running.
if ! pgrep -f "litex_server --jtag" >/dev/null; then
    echo ">> starting litex_server --jtag (CH347 -> :1234) ..."
    OPENOCD=$OPENOCD litex_server --jtag --jtag-config "$CFG" >/tmp/litex_server_jtag.log 2>&1 &
    sleep 4
    echo "   (log: /tmp/litex_server_jtag.log)"
fi

# Step 2: list signals (sanity) then capture.
echo ">> available signals:"
python3 "$CLI" --csv "$ACSV" --csr-csv "$CSR" -l 2>/dev/null || true

# Capture: trigger on the NCO consume tick (p_strobe rising), 1/4 pre-trigger
# offset so we see level + we/re both before and after a consume. 4096 deep
# = 82 us @50MHz = ~4 strobes -> exact strobe & push spacing on ONE time base.
echo ">> capturing (trigger: p_strobe rising) -> /tmp/litescope_fifo.vcd"
python3 "$CLI" --csv "$ACSV" --csr-csv "$CSR" \
    -r avbsoc_aafpacketizer_p_strobe \
    --offset 1024 --length 4096 \
    --dump /tmp/litescope_fifo.vcd

echo ">> done. View: gtkwave /tmp/litescope_fifo.vcd"
echo "   Decisive reads:"
echo "    - level waveform: flat 512? oscillating? near 0?  (is 'level' truthful)"
echo "    - p_strobe spacing in cycles: 1041.7=>48000Hz, 1041.5=>48007Hz (true media rate)"
echo "    - syncfifo_we spacing vs p_first: matches 46979 host? or higher (phantom first)"
echo "    - at p_strobe: is syncfifo_readable=1 (drains) or 0 (underrun/empty)"
