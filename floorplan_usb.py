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

# PHASE-2 dual-USB: if a 2nd subsystem (usb_avb_subsystem2) is present in the
# netlist, split the left edge — USB1 lower half, USB2 upper half — so each
# ULPI's cells cluster near their (bank-34, left-edge) pins and the placer
# doesn't scatter 2x USB logic across the die (which stalls the analytic placer).
# NOTE 'usb_avb_subsystem' is a substring of 'usb_avb_subsystem2', so USB1's
# matches below must EXCLUDE USB2_PREFIX.
USB2_REGION = "usb2_fp"
USB2_PREFIX = "usb_avb_subsystem2"
_HAS_USB2 = any(USB2_PREFIX in c for c, _ in ctx.cells)
if _HAS_USB2 and not _u:
    UY1 = 76                                  # USB1 -> lower half
U2X0, U2Y0, U2X1, U2Y1 = 0, 80, 45, 156       # USB2 -> upper half

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
if _HAS_USB2:
    ctx.createRectangularRegion(USB2_REGION, U2X0, U2Y0, U2X1, U2Y1)
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
    if USB_PREFIX in cname and USB2_PREFIX not in cname:   # USB1 only (excl USB2)
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
    if USB_PREFIX not in nname or USB2_PREFIX in nname:    # USB1 only (excl USB2)
        continue
    nnets += 1
    drv = getattr(net, "driver", None)
    if drv is not None:
        nu += _pull(getattr(drv, "cell", None), USB_REGION)
    for u in getattr(net, "users", []):
        nu += _pull(getattr(u, "cell", None), USB_REGION)

# ---- Pass 1+2 for USB2 (Backup), pulled into its own upper-half region --------
nu2 = 0
if _HAS_USB2:
    for cname, cell in ctx.cells:
        if USB2_PREFIX in cname:
            nu2 += _pull(cell, USB2_REGION)
    for nname, net in ctx.nets:
        if USB2_PREFIX not in nname:
            continue
        drv = getattr(net, "driver", None)
        if drv is not None:
            nu2 += _pull(getattr(drv, "cell", None), USB2_REGION)
        for u in getattr(net, "users", []):
            nu2 += _pull(getattr(u, "cell", None), USB2_REGION)
    print("[floorplan] USB2 region %d,%d..%d,%d  cells=%d"
          % (U2X0, U2Y0, U2X1, U2Y1, nu2))

# ---- cfgflash SPIMaster: BIG region right of the USB box --------------------
# Keeps the SPIMaster cells out of the marginal USB zone (X<=45) AND gives the
# better-Fmax placements (61-64 MHz vs 54 with it floating). Big region (not the
# tight X95-114 box that thrashed) legalises fast.
CFG_REGION = "cfgflash_fp"
CFG_PREFIX = "cfgflash"
ctx.createRectangularRegion(CFG_REGION, 50, 0, 114, 156)
ncf = 0
for cname, cell in ctx.cells:
    if CFG_PREFIX in cname:                  # NAMED SPIMaster cells ONLY — pulling
        ncf += _pull(cell, CFG_REGION)       # net-DRIVERS too dragged long-net cells
                                             # and thrashed the placer; named is local.
print("[floorplan_usb] cfgflash: %d cells -> %s (X 50..114)" % (ncf, CFG_REGION))

# ---- AAF packetizer: compact box in the clear right-center -----------------
# The sys-domain critical paths (pres-time t_next_value adder, ring rd/level,
# byte-build FSM) are ALL internal to the AAF packetizer and routing-dominated
# — left to float they scatter across the die and cap sys at ~40-45 MHz. Boxing
# them tight keeps those internal nets short so sys closes 50 deterministically,
# independent of the placer's seed. Clear of USB (X<=45) and the eth-TX pins.
# Same net-connectivity capture as USB (the bulk are anonymous $abc/$auto cells
# that only the NETS carry the 'aafpacketizer' hierarchy for). Tunable via
# NEXTPNR_AAF_REGION="x0,y0,x1,y1"; set "" to disable.
AAF_REGION = "aaf_fp"
AAF_PREFIX = "aafpacketizer"
AX0, AY0, AX1, AY1 = 72, 28, 104, 96
_a = os.environ.get("NEXTPNR_AAF_REGION", "").strip()   # default OFF: boxing the
na = 0
if _a != "":
    if "," in _a:
        AX0, AY0, AX1, AY1 = (int(v) for v in _a.split(","))
    ctx.createRectangularRegion(AAF_REGION, AX0, AY0, AX1, AY1)
    # SELECTIVE capture: unlike USB (self-contained), the AAF packetizer's nets
    # fan out to the MAC / CSR bus / MCR, so pulling net *users* dragged ~66% of
    # the design into the box (11896 cells — unplaceable). Pull only (a) cells
    # NAMED aafpacketizer and (b) the DRIVER of each aafpacketizer net (the cell
    # that PRODUCES the signal — i.e. AAF-internal logic, incl. the anonymous
    # $abc luts that drive AAF nets). Skip users (the external readers).
    for cname, cell in ctx.cells:
        if AAF_PREFIX in cname:
            na += _pull(cell, AAF_REGION)
    for nname, net in ctx.nets:
        if AAF_PREFIX not in nname:
            continue
        drv = getattr(net, "driver", None)
        if drv is not None:
            na += _pull(getattr(drv, "cell", None), AAF_REGION)
    print("[floorplan_usb] AAF: %d cells -> %s (X %d..%d, Y %d..%d)"
          % (na, AAF_REGION, AX0, AX1, AY0, AY1))

# ---- VexRiscv CPU: compact box (Phase-1 timing, 2026-07-08) -----------------
# After registering the CSR bridge (avb_soc.py add_csr_bridge override), the
# sys_clk critical path moved OFF the CSR decode ONTO the VexRiscv CPU's own
# datapath: dBus_cmd -> maccmap multiplier -> ALU -> IBusSimplePlugin_pending,
# ROUTING-dominated (12.2ns route / 2.8ns logic), sprawling X56-67 Y12-51. The
# CPU is the only big sys-domain block with no region, so it scatters and its
# internal nets run 1.5-1.8ns each. Box it compact to shorten them. The path's
# nets ALL carry the 'VexRiscv' hierarchy, so named-cell + net-DRIVER capture
# (drivers only — pulling users would drag every peripheral the CPU talks to)
# catches the anonymous $abc/maccmap/alumacc soup on the path. Plain LUT/FF/
# CARRY (no LUTRAM) so it won't hit the eth-box legalisation hang. Clear of USB
# (X<=45). Tunable/disable via NEXTPNR_CPU_REGION="" ; default ON.
CPU_REGION = "cpu_fp"
CPU_PREFIX = "VexRiscv"
CX0, CY0, CX1, CY1 = 46, 6, 78, 104
# DEFAULT OFF (2026-07-08): this box THRASHED the analytical placer (>160s, no
# routing). Root: the CPU dBus/iBus tie to the SRAM+ROM BRAMs (16+43 blocks) in
# fixed device columns; constraining the CPU LUT/FF/CARRY to X46-78 while its
# BRAMs sit elsewhere makes legalisation fight. FOLLOW-UP to make it work: size
# the region to span the BRAM columns the CPU uses (or leave BRAMs out and widen
# Y so the CPU logic hugs its own BRAM rows). Enable to experiment:
# NEXTPNR_CPU_REGION="46,6,78,104" (or tuned coords).
_cpu = os.environ.get("NEXTPNR_CPU_REGION", "").strip()
ncpu = 0
if _cpu != "":
    if "," in _cpu:
        CX0, CY0, CX1, CY1 = (int(v) for v in _cpu.split(","))
    ctx.createRectangularRegion(CPU_REGION, CX0, CY0, CX1, CY1)
    for cname, cell in ctx.cells:
        if CPU_PREFIX in cname:
            ncpu += _pull(cell, CPU_REGION)
    for nname, net in ctx.nets:
        if CPU_PREFIX not in nname:
            continue
        drv = getattr(net, "driver", None)
        if drv is not None:
            ncpu += _pull(getattr(drv, "cell", None), CPU_REGION)
    print("[floorplan_usb] CPU: %d cells -> %s (X %d..%d, Y %d..%d)"
          % (ncpu, CPU_REGION, CX0, CX1, CY0, CY1))

print("[floorplan_usb] USB: %d cells (via %d nets + name match) -> %s (X %d..%d, Y %d..%d)"
      % (nu, nnets, USB_REGION, UX0, UX1, UY0, UY1))
if _eth_on:
    print("[floorplan_usb] eth TX: %d cells -> %s (X %d..%d, Y %d..%d)"
          % (ne, ETH_REGION, EX0, EX1, EY0, EY1))
if nu == 0:
    print("[floorplan_usb] WARNING: 0 USB cells matched — prefix changed? no-op.")
