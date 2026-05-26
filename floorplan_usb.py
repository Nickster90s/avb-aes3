#!/usr/bin/env python3
# nextpnr-xilinx --pre-place floorplan for the USB+AVB combined build.
# Goal: recover eth_tx_clk >= 125 MHz (gigabit RGMII) with the USB block present.
# See docs/phase3-bridge.md (P3.2).
#
# Background (measured, not guessed)
# ----------------------------------
# nextpnr grid: X 0..114, Y 0..156. ALL package I/O is on the LEFT edge
# (PAD bels only at grid X=0/1): eth TX pins X1,Y51-54; eth RX X1,Y55-72;
# ULPI X1,Y23-49.
#
# v1 (USB-only region, X>=60) made eth_tx WORSE (112 -> 104 MHz). The
# post-route critical path proved why: it is ENTIRELY the LiteEth TX
# datapath
#     preamble_inserter -> tx_crc_pipe -> txlastbe -> tx_cdc graycounter
#     -> storage_2 (the TX CDC FIFO)
# at 1.4 ns logic / 8.2 ns ROUTING. That cluster placed at X51..62, Y44..84,
# and the v1 USB boundary at X60 sliced right through it (a cluster cell sat
# at X62, inside the USB zone) — USB crowded the exact columns eth needed.
#
# v2 strategy (this file)
# -----------------------
# 1. Compact the eth TX datapath into a tight box (ETH_*), shrinking its
#    routing-dominated internal nets (the worst is a 2.3 ns graycounter->FIFO
#    net spanning ~15 rows). This is the direct fix for the critical path.
# 2. Confine the USB block to the right edge (USB_*), well clear of the eth
#    box, so it can never crowd those columns again.
# 3. A keep-out gap is left between the two boxes.
#
# constrainCellToRegion only PULLS the named cells in; it never evicts others.
# IOBs are not constrained (they keep their pinned LOC on the left edge); we
# only move logic. Placement/timing failures are loud — this cannot silently
# break the AVB stack.
#
# Tunable via env (comma lists "x0,y0,x1,y1"): NEXTPNR_USB_REGION,
# NEXTPNR_ETH_REGION. Set NEXTPNR_ETH_REGION="" to disable the eth box.

import os

# ---- USB block: right edge, clear of the eth TX cluster --------------------
USB_REGION = "usb_fp"
USB_PREFIX = "usb_avb_subsystem"          # clean + $flatten\ -escaped names
UX0, UY0, UX1, UY1 = 78, 0, 114, 156
_u = os.environ.get("NEXTPNR_USB_REGION", "").strip()
if _u:
    UX0, UY0, UX1, UY1 = (int(v) for v in _u.split(","))

# ---- eth TX datapath: compact box near the TX pins (X1,Y51) ----------------
# Matches the cells seen on the critical path. storage_2 is the TX CDC FIFO
# RAM (anchored forms 'storage_2.' / 'storage_2$' / 'storage_2/' avoid
# matching storage_20.. ). All are LiteEth TX-clock-domain logic.
ETH_REGION   = "eth_tx_fp"
ETH_SUBSTR   = ("txdatapath", "tx_crc_pipe", "tx_cdc")
ETH_FIFO_PFX = ("storage_2.", "storage_2$", "storage_2/", "\\storage_2$")
EX0, EY0, EX1, EY1 = 38, 44, 66, 88   # only used if explicitly enabled
# eth box OFF by default: it is HARMFUL — it hung the placer for 90 min (v2,
# too few SLICEM cols for the storage_2 LUTRAM) and, when it did complete,
# dragged eth_tx_clk to 94 MHz. The winning config is USB-floorplan-only +
# a good seed. Enable only for experiments: NEXTPNR_ETH_REGION="38,44,66,88".
_e = os.environ.get("NEXTPNR_ETH_REGION", "").strip()
_eth_on = _e != ""
if _eth_on:
    EX0, EY0, EX1, EY1 = (int(v) for v in _e.split(","))


def _matches_eth(name):
    if any(s in name for s in ETH_SUBSTR):
        return True
    return any(p in name for p in ETH_FIFO_PFX)


ctx.createRectangularRegion(USB_REGION, UX0, UY0, UX1, UY1)
if _eth_on:
    ctx.createRectangularRegion(ETH_REGION, EX0, EY0, EX1, EY1)

nu = ne = 0
for cname, cell in ctx.cells:
    if USB_PREFIX in cname:
        ctx.constrainCellToRegion(cname, USB_REGION)
        nu += 1
    elif _eth_on and _matches_eth(cname):
        ctx.constrainCellToRegion(cname, ETH_REGION)
        ne += 1

print("[floorplan_usb] USB: %d cells -> %s (X %d..%d, Y %d..%d)"
      % (nu, USB_REGION, UX0, UX1, UY0, UY1))
if _eth_on:
    print("[floorplan_usb] eth TX: %d cells -> %s (X %d..%d, Y %d..%d)"
          % (ne, ETH_REGION, EX0, EX1, EY0, EY1))
if nu == 0:
    print("[floorplan_usb] WARNING: 0 USB cells matched — name changed? no-op.")
if _eth_on and ne == 0:
    print("[floorplan_usb] WARNING: 0 eth TX cells matched — check substrings.")
