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

# ---- USB block: LEFT edge, close to the ULPI pins at X=1, Y=23-49 ----------
# (Opposite of the original v2 region which pushed USB to X>=78.) Rationale:
# patch #3 (TX-only sys-datapath) made gigabit eth_tx robust WITHOUT a
# floorplan, so the v2 "USB to the right half" recipe is no longer needed —
# AND that distance from the ULPI pins re-introduced placement-marginal HS
# ULPI sampling once the CRF extractor + MCRI2STx were added (USB enumerated
# only as full-speed, error -71 during HS chirp).
# Goal here: keep USB cells within ~40 columns of X=1 so the 60 MHz ULPI
# input setup time is satisfied regardless of synthesis non-determinism.
# Eth TX cluster (X=51-62) is well clear of X<=40.
USB_REGION = "usb_fp"
USB_PREFIX = "usb_avb_subsystem"          # clean + $flatten\ -escaped names
UX0, UY0, UX1, UY1 = 0, 0, 45, 156
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

# ---------------------------------------------------------------------------
# NET-CONNECTIVITY matching (the fix, 2026-05-28).
#
# Matching cells by name (`USB_PREFIX in cname`) catches only ~2% of the
# USB subsystem (388 / 18019 cells). yosys+ABC rename the bulk of the
# combinational logic to anonymous forms like `$auto$alumacc.cc:...` and
# `$abc$...$lut$aiger...` that DON'T carry the `usb_avb_subsystem`
# hierarchy — so the floorplan was leaving ~95% of USB cells (incl. the
# cd_usb-critical get_descriptor compare) to float wherever the placer
# liked. That's why USB enumeration was a per-build lottery and why the
# get_descriptor path spilled to X74 and dropped cd_usb to 54.5 MHz.
#
# But the NETS keep their hierarchy names (1584 / 22860 match the
# prefix). And `net.driver.cell` + `net.users[i].cell` reach the actual
# cells — including the anonymous `$auto$`/`$abc aiger` ones connected to
# USB nets. So: walk every usb-named net, pull its driver + all user
# cells into the region. This catches the floating soup by connectivity
# instead of by name.
#
# IOBs / already-located cells can't be region-constrained; constrain in
# a try/except and skip failures (the ULPI pins keep their pinned LOC).
# ---------------------------------------------------------------------------
_constrained = set()

# Only region-constrain FABRIC LOGIC. BRAM / DSP / IOB / clock primitives
# live in fixed device columns; pinning them into a narrow rectangle makes
# the analytic placer thrash trying to legalise (the same 90-min hang the
# eth-box LUTRAM caused). The cd_usb critical path is the get_descriptor
# LUT/carry compare — fabric logic — so constraining LUT/FF/CARRY is what
# matters; the 3 USB BRAMs + 11 ULPI IOBs float to their natural columns.
_CONSTRAIN_TYPES = ("SLICE_LUTX", "SLICE_FFX", "CARRY4", "SELMUX2")
def _constrainable(cell):
    t = getattr(cell, "type", "") or ""
    return any(t.startswith(p) for p in _CONSTRAIN_TYPES)

def _pull(cell, region):
    """Constrain a fabric-logic cell into a region; dedup + skip the rest."""
    if cell is None or not _constrainable(cell):
        return 0
    nm = cell.name
    if nm in _constrained:
        return 0
    try:
        ctx.constrainCellToRegion(nm, region)
        _constrained.add(nm)
        return 1
    except Exception:
        return 0   # already-located / packed — leave it be

nu = ne = 0

# Pass 1 — direct name match (fast path for the cells that DO carry the
# prefix: named FFs, ROM-derived ABC luts).
for cname, cell in ctx.cells:
    if USB_PREFIX in cname:
        nu += _pull(cell, USB_REGION)
    elif _eth_on and _matches_eth(cname):
        if cname not in _constrained:
            try:
                ctx.constrainCellToRegion(cname, ETH_REGION)
                _constrained.add(cname)
                ne += 1
            except Exception:
                pass

# Pass 2 — net connectivity: pull driver + users of every usb-named net.
# This is what catches the anonymous combinational soup.
nnets = 0
for nname, net in ctx.nets:
    if USB_PREFIX not in nname:
        continue
    nnets += 1
    drv = getattr(net, "driver", None)
    if drv is not None:
        nu += _pull(getattr(drv, "cell", None), USB_REGION)
    for u in getattr(net, "users", []):
        nu += _pull(getattr(u, "cell", None), USB_REGION)

print("[floorplan_usb] USB: %d cells (via %d nets + name match) -> %s (X %d..%d, Y %d..%d)"
      % (nu, nnets, USB_REGION, UX0, UX1, UY0, UY1))
if _eth_on:
    print("[floorplan_usb] eth TX: %d cells -> %s (X %d..%d, Y %d..%d)"
          % (ne, ETH_REGION, EX0, EX1, EY0, EY1))
if nu == 0:
    print("[floorplan_usb] WARNING: 0 USB cells matched — prefix changed? no-op.")
