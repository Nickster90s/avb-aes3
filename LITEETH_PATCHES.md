# Required liteeth patches for this project

liteeth is patched in-place at /home/lisp/litex/liteeth/ (not vendored).
Re-apply these after any liteeth reinstall.

## 1. ptp.py — TSU addend multiplier (pre-existing)
Shift-add tree instead of `*` so nextpnr-xilinx can route the 52x30
multiply (the natural form infers an unrouteable DSP48 cascade). See
memory liteeth-tsu-dsp-workaround.

## 2. mac/core.py — TX skid buffer before the CDC (2026-05-26, P3.2)
Inserted `stream.Buffer(eth_phy_description(datapath_dw),
pipe_valid=True, pipe_ready=True)` in cd_tx before the with_sys_datapath
domain switch. Pipelines the CRC→CDC handshake. Backup: core.py.nen-bak.

RESULT: eth_tx_clk 112 -> 114 MHz with the USB block present — helps but
NOT enough (need 125 for gigabit RGMII). The TX valid/ready handshake is
a multi-stage combinational mesh (padding<->crc<->preamble<->last_be) and
the failure is routing/congestion-dominated: the USB UAC2 block scatters
the TX-datapath cells across the die (critical-path coords bounce
35,46 <-> 30,49 <-> 43,39). One buffer breaks one link only.

NEXT LEVER = FLOORPLAN, not more buffers: region-constrain the USB block
(usb_avb_subsystem instance) to a PBlock away from the LiteEth TX
datapath so the placer keeps the TX cells clustered (recovers the
without-USB 131 MHz at seed=8). nextpnr-xilinx region constraints via
the XDC / --xdc, or LiteX platform.add_platform_command region. Then
the buffer + clustering together should clear 125 MHz.
Alternative if floorplan insufficient: trim USB (fewer channels) or
pipeline more TX handshake links.
