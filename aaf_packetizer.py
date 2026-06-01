#!/usr/bin/env python3
#
# Gateware AAF TX packetizer — moves the USB→AVB audio path out of firmware.
# Copyright 2025-2026 Nick (nick.eventslight@gmail.com)
# SPDX-License-Identifier: Apache-2.0
#
# This is the TX-side counterpart to avtp_extractor.py / crf_extractor.py:
# it keeps the CPU out of the per-sample audio path entirely. The CPU only
# writes the stream binding (dst_mac, stream_id, vlan, src_mac) once at ACMP
# CONNECT time and flips `enable`; it never touches a sample again.
#
# Data path (all sys domain):
#
#   usb_avb_subsystem sample handshake          (USB host clock, ~48 kHz)
#        │  (lo/hi/readable/pop — same signals firmware's usb_aaf_drain reads)
#        ▼
#   8-channel frame assembler  ──►  block_fifo (256-bit, elastic rate buffer)
#        │
#        ▼  read paced by mcr.sample_strobe   ◄── THE MEDIA CLOCK
#   pay[fill][blk]  (ping-pong, 6 blocks/packet)
#        │  every 6 strobes → send_req
#        ▼
#   builder FSM ──► frame_ram (59×32) ──► stream.Endpoint(32) ──► TX mux ──► MAC
#        ▲
#        └── presentation_time = (sec·1e9 + ns + offset) mod 2^32   (from TSU)
#
# MEDIA CLOCK / CRF NOTE
# ----------------------
# Egress is paced by `mcr.sample_strobe`, the same NCO strobe MCRI2STx uses.
# Firmware's PI servo (mcr.c) tunes the NCO increment from CRF timestamps
# whenever the selected clock source is CRF (cs=1) and CRF is locked — so in
# that regime the strobe rate, and therefore the AAF stream rate AND the
# presentation-time cadence, are the recovered CRF media clock by construction.
# When CRF is not locked / cs=0, the NCO free-runs at local 48 kHz. No extra
# logic is needed in the packetizer: "rate from CRF when locked+cs=1" falls
# out of pacing on the NCO, exactly like the I2S DAC.

from functools import reduce
from operator import or_

from migen import *
from migen.genlib.fifo import SyncFIFO
from litex.gen import LiteXModule
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import CSRStorage, CSRStatus

from liteeth.common import eth_phy_description


# 1_000_000_000 as set-bit positions, for a DSP-free constant multiply.
# (Same lesson as the LiteEth TSU addend*1e9 shift-add workaround — a `*`
#  here infers an unroutable DSP cascade on nextpnr-xilinx.)
_NS_PER_SEC_BITS = [29, 28, 27, 25, 24, 23, 20, 19, 17, 15, 14, 11, 9]


class TXFrameArbiter(LiteXModule):
    """Frame-atomic N:1 stream arbiter for the LiteEth MAC core sink.

    Priority = list order (index 0 highest — give that to the firmware SRAM
    reader so gPTP / AVDECC / MSRP are never delayed; the AAF talker waits a
    few µs at most, dwarfed by the 2 ms presentation offset). A frame in
    flight is never interrupted: once a source is granted, the grant holds
    until that source asserts valid & ready & last.
    """
    def __init__(self, sources, dw=32):
        self.source = source = stream.Endpoint(eth_phy_description(dw))
        n = len(sources)

        sel  = Signal(max=max(2, n))
        busy = Signal()

        # High while source index 0 (the firmware/control-plane path) holds the
        # grant — used to gate the gPTP TX-timestamp latch.
        self.firmware_granted = Signal()
        self.comb += self.firmware_granted.eq(busy & (sel == 0))

        # Granted source drives the output; backpressure routes to it alone.
        for i, s in enumerate(sources):
            self.comb += If(busy & (sel == i),
                source.valid.eq(s.valid),
                source.data.eq(s.data),
                source.last.eq(s.last),
                source.last_be.eq(s.last_be),
                source.error.eq(s.error),
                s.ready.eq(source.ready),
            )

        # Priority encoder: iterate high→low so index 0 wins when valid.
        nextsel = Signal(max=max(2, n))
        anyv    = Signal()
        self.comb += anyv.eq(reduce(or_, [s.valid for s in sources]))
        for i in reversed(range(n)):
            self.comb += If(sources[i].valid, nextsel.eq(i))

        self.sync += [
            If(~busy,
                If(anyv,
                    sel.eq(nextsel),
                    busy.eq(1),
                ),
            ).Elif(source.valid & source.ready & source.last,
                busy.eq(0),
            ),
        ]


def _mul_1e9_lo32(x):
    """Low 32 bits of x * 1_000_000_000, as a shift-add tree (no multiplier).

    Only the low 32 bits of `x` matter mod 2^32, but we let the slice handle
    that — yosys prunes the unused high bits."""
    return reduce(lambda a, b: a + b, [(x << s) for s in _NS_PER_SEC_BITS])


class AAFPacketizer(LiteXModule):
    """Build + transmit AVTP-AAF (32-bit INT) frames from the USB sample
    stream, paced by the MCR NCO. Produces a LiteEth phy-description source
    to be muxed onto the MAC core sink.

    Parameters
    ----------
    mcr  : MCRNco          — provides `sample_strobe` (the media clock tick).
    tsu  : LiteEthTSU      — provides live `seconds` / `nanoseconds`.
    usb_* : the sample handshake exported by usb_avb_subsystem (sys domain):
        usb_sample_lo[8:32] = 24-bit audio, MSB-aligned in a 32-bit sample
        usb_sample_hi[0:3]  = channel index, usb_sample_hi[3] = first marker
        usb_readable        = FIFO head valid
        self.usb_pop        = OUTPUT: one-cycle pop strobe (mux into the
                              wrapper's sample_pop ONLY when self.enable.storage)
    channels           : audio channels per AAF frame (Milan default 8).
    samples_per_packet : AAF blocks per packet (Milan AAF @48k = 6).
    """
    def __init__(self, mcr, tsu, *, usb_sample_lo, usb_sample_hi,
                 usb_readable, channels=8, samples_per_packet=6,
                 fifo_depth=64):
        dw = 32
        ch_bits  = max(1, log2_int(channels, need_pow2=False))
        blk_bits = max(1, log2_int(samples_per_packet, need_pow2=False))

        # ---- Frame geometry (computed once, Python-side) ----
        HDR_LEN  = 42                                 # 14 eth + 4 vlan + 24 avtp
        PAY_LEN  = samples_per_packet * channels * 4  # 6*8*4 = 192
        TOTAL    = HDR_LEN + PAY_LEN                   # 234
        rem      = TOTAL % 4
        N_WORDS  = (TOTAL + 3) // 4                    # 59
        LAST_IDX = N_WORDS - 1
        LAST_BE  = 0xF if rem == 0 else ((1 << rem) - 1)

        # AAF header scalar fields (depend on channels/spp).
        nsr_ch   = (5 << 12) | (channels & 0x3FF)     # nsr=48k | channels
        data_len = PAY_LEN                            # bytes of stream payload

        # ---- Output stream (to be muxed onto mac.core.sink) ----
        self.source = source = stream.Endpoint(eth_phy_description(dw))

        # ---- Pop strobe back to the USB wrapper (gated by enable upstream) ----
        self.usb_pop = Signal()

        # ---- CSRs: binding (firmware writes once at CONNECT) ----
        self.enable        = CSRStorage(1,  description="1 = gateware sources the AAF stream (CPU out of the audio path).")
        self.src_mac_hi    = CSRStorage(16, description="Source MAC [47:32] (FPGA MAC).")
        self.src_mac_lo    = CSRStorage(32, description="Source MAC [31:0].")
        self.dst_mac_hi    = CSRStorage(16, description="Dest MAC [47:32] (SRP/ACMP learned multicast).")
        self.dst_mac_lo    = CSRStorage(32, description="Dest MAC [31:0].")
        self.stream_id_hi  = CSRStorage(32, description="AVTP stream_id [63:32] (byte0 = bits[31:24]).")
        self.stream_id_lo  = CSRStorage(32, description="AVTP stream_id [31:0].")
        self.vlan_tci      = CSRStorage(16, reset=(3 << 13) | 2,
                             description="802.1Q TCI = (pcp<<13)|vid. Class A default pcp=3, vid=2.")
        self.pres_offset   = CSRStorage(32, reset=2_000_000,
                             description="presentation_time offset (ns) added to gPTP now. Milan AAF = 2 ms.")

        # ---- CSRs: status (read-only diagnostics) ----
        self.packet_count   = CSRStatus(32, description="AAF frames transmitted.")
        self.underrun_count = CSRStatus(32, description="Media-clock ticks where block_fifo was empty (silence inserted).")
        self.overrun_count  = CSRStatus(32, description="send_req arriving while builder busy (packet skipped — should stay 0).")
        self.fifo_level     = CSRStatus(blk_bits + 8, description="block_fifo occupancy (blocks).")
        # Flow-control diagnostics (soft ILA): measure block production vs
        # consumption to resolve the FIFO-overflow contradiction. Rate them
        # over a precise interval from the firmware `a` command.
        self.dbg_block_push = CSRStatus(32, description="blocks PRODUCED (block_fifo.we) — assembler frame rate.")
        self.dbg_block_pop  = CSRStatus(32, description="blocks CONSUMED (block_fifo.re) — packetizer strobe rate.")
        self.dbg_first      = CSRStatus(32, description="`first` markers consumed (do_pop & first) — host frame boundaries.")
        # Min/max of block_fifo.level since last reset — resolves "stuck full" vs
        # "oscillating full↔empty" (aggregate counters can't). Write dbg_level_rst
        # to start a fresh window.
        self.dbg_level_min  = CSRStatus(16, description="min block_fifo.level since reset.")
        self.dbg_level_max  = CSRStatus(16, description="max block_fifo.level since reset.")
        self.dbg_level_rst  = CSRStorage(1, description="write 1 → restart min/max window at current level.")
        # THE two numbers that break the level/strobe/host contradiction:
        #  - dbg_raw_strobe: mcr.sample_strobe counted UNGATED (no en/primed gate),
        #    i.e. the true NCO/consumer demand rate. Distinguishes 48000 (1041.7
        #    cyc) from 48007 independent of FIFO state.
        #  - dbg_usb_samp: actual samples DRAINED from the cd_usb→sys bridge
        #    (usb_readable & do_pop) = true USB producer rate. /8 = frame rate,
        #    directly comparable to usbmon (46979) and to dbg_first. If
        #    dbg_first > dbg_usb_samp/8 then `first` is glitching (phantom push).
        self.dbg_raw_strobe = CSRStatus(32, description="mcr.sample_strobe UNGATED — true NCO consumer rate.")
        self.dbg_usb_samp   = CSRStatus(32, description="samples drained from USB bridge (usb_readable & do_pop) — true producer rate.")

        # MAC error lane is always 0 for our generated frames.
        self.comb += source.error.eq(0)

        # =========================================================
        # 1) USB ingress -> 8-channel frame assembler -> SRC ring
        # =========================================================
        # Async sample-rate converter (FIX (b), 2026-06-01). The USB host writes
        # whole 8ch frames into a BRAM ring at its own crystal rate; the gPTP NCO
        # strobe pulls ONE interpolated frame per tick (= the AVB media clock). A
        # Q1.31 phase accumulator (`src_step` = f_in/f_out) + per-channel linear
        # interpolation resamples host-rate -> gPTP-rate. `src_step` is servo'd IN
        # FIRMWARE from the ring level (level -> step) — the piece the earlier WIP
        # (6797563) lacked: at fixed step=1.0 the gPTP consumer out-paced the host
        # crystal and drained the ring. The block_fifo approach it replaces could
        # never centre because USB feedback is decoupled from block_fifo level by
        # the wrapper's free-running producer (root cause, 2026-06-01). Here the
        # OUTPUT rate is gPTP and does NOT feed back to the host, so no runaway;
        # USB feedback is pinned nominal (0x60000).
        assert (fifo_depth & (fifo_depth - 1)) == 0, "SRC ring depth must be power of 2"
        log2depth = log2_int(fifo_depth)
        mem = Memory(channels * 32, fifo_depth)   # BRAM (sync reads), not LUT-RAM
        self.specials += mem
        wp  = mem.get_port(write_capable=True)
        rp0 = mem.get_port()
        rp1 = mem.get_port()
        self.specials += wp, rp0, rp1

        self.src_step = CSRStorage(32, reset=1 << 31,
            description="SRC resampling ratio f_in/f_out in Q1.31 (1<<31 = 1.0); firmware servo'd from ring level.")
        self.fifo_depth = fifo_depth

        wr     = Signal(32)
        rd_int = Signal(32)
        frac   = Signal(31)
        # SIGNED occupancy: if the consumer ever reaches the producer an unsigned
        # wr-rd_int underflows to a huge value that masquerades as "full"; signed,
        # it goes negative -> have2 false -> consumer stalls and waits.
        level  = Signal((33, True))
        self.comb += level.eq(wr - rd_int)
        self.block_level = Signal(max=fifo_depth + 1)
        self.comb += self.block_level.eq(level)        # legacy port for avb_soc
        # unsigned, range-clamped copy for the level CSR + min/max tracker
        level_u = Signal(max=fifo_depth + 1)
        self.comb += If(level < 0, level_u.eq(0)).Elif(level > fifo_depth, level_u.eq(fifo_depth)).Else(level_u.eq(level))

        rd0 = Signal(log2depth); rd1 = Signal(log2depth)
        self.comb += [rd0.eq(rd_int[0:log2depth]), rd1.eq((rd_int + 1)[0:log2depth]),
                      rp0.adr.eq(rd0), rp1.adr.eq(rd1)]

        # Min/max level tracker (resolves stuck-full vs oscillating vs centred).
        _lvl_min = Signal(max=fifo_depth + 1, reset=fifo_depth)
        _lvl_max = Signal(max=fifo_depth + 1, reset=0)
        self.sync += [
            If(self.dbg_level_rst.re,
                _lvl_min.eq(level_u), _lvl_max.eq(level_u),
            ).Else(
                If(level_u < _lvl_min, _lvl_min.eq(level_u)),
                If(level_u > _lvl_max, _lvl_max.eq(level_u)),
            ),
        ]
        self.comb += [self.dbg_level_min.status.eq(_lvl_min),
                      self.dbg_level_max.status.eq(_lvl_max)]

        cur  = Array([Signal(32) for _ in range(channels)])
        have = Signal()
        ch    = usb_sample_hi[0:ch_bits]
        first = usb_sample_hi[3]
        # 24-bit audio MSB-aligned into a 32-bit sample (= firmware v & 0xFFFFFF00).
        samp32 = Cat(Signal(8), usb_sample_lo[8:32])   # [0:8]=0, [8:32]=audio

        need_push = first & have
        en = self.enable.storage
        # ALWAYS drain the wrapper bridge — do NOT stall on ring-full. The
        # cd_usb->sys bridge is a 2nd buffer in series with this ring; stalling
        # do_pop when the ring fills lets the bridge accumulate, making it a
        # second integrator. Two cascaded integrators + the proportional
        # src_step servo = a relaxation limit-cycle (ring rode full with deep
        # dips, on-HW 2026-06-01). Draining unconditionally keeps the bridge
        # near-empty (pure CDC latency, not an integrator) so only the ring
        # integrates -> the P servo is first-order stable. On ring-full we DROP
        # the just-completed frame (don't write) instead of back-pressuring;
        # once the servo centres (~level 286) the ring never nears full, so
        # drops happen only during the startup transient.
        have_space = Signal()
        self.comb += have_space.eq(level < (fifo_depth - 2))
        do_pop  = usb_readable
        ring_wr = Signal()
        self.comb += [
            self.usb_pop.eq(do_pop),
            ring_wr.eq(en & do_pop & need_push & have_space),
            wp.adr.eq(wr[0:log2depth]),
            wp.dat_w.eq(Cat(*cur)),    # the just-completed frame (cur updates same edge)
            wp.we.eq(ring_wr),
        ]
        self.sync += [
            If(ring_wr, wr.eq(wr + 1)),
            If(do_pop,
                If(need_push, *[cur[i].eq(0) for i in range(channels)]),
                cur[ch].eq(samp32),
                have.eq(1),
            ),
        ]

        # ---- producer-side probe taps (kept from soft-ILA; harmless) ----
        self.p_usb_readable = Signal(); self.p_do_pop = Signal()
        self.p_first = Signal(); self.p_need_push = Signal()
        self.comb += [self.p_usb_readable.eq(usb_readable), self.p_do_pop.eq(do_pop),
                      self.p_first.eq(first), self.p_need_push.eq(need_push)]

        # =========================================================
        # 2) Media-clock-paced SRC read -> pay ping-pong buffer
        # =========================================================
        pay      = Array([Signal(channels * 32) for _ in range(16)])
        fill_buf = Signal()
        send_buf = Signal()
        blk_idx  = Signal(blk_bits)
        send_req = Signal()

        # Prime the ring to centre before consuming (equal jitter headroom).
        primed = Signal()
        _center = fifo_depth // 2
        self.sync += [If(~en, primed.eq(0)).Elif(level >= _center, primed.eq(1))]
        strobe = Signal()
        self.comb += strobe.eq(mcr.sample_strobe & en & primed)

        # Pipelined (2-stage) linear interpolation between the two adjacent ring
        # frames at `frac`. rd_int/frac are stable ~1042 sys cycles between
        # strobes, so the read+interp pipeline is always settled when a strobe
        # samples `interp`; the constant latency is harmless.
        f0_r = Signal(channels * 32); f1_r = Signal(channels * 32); frac_r = Signal(31)
        self.sync += [f0_r.eq(rp0.dat_r), f1_r.eq(rp1.dat_r), frac_r.eq(frac)]
        interp = Signal(channels * 32)
        for c in range(channels):
            s0 = Signal((32, True)); s1 = Signal((32, True))
            self.comb += [s0.eq(f0_r[c*32:(c+1)*32]), s1.eq(f1_r[c*32:(c+1)*32])]
            delta = Signal((33, True)); self.comb += delta.eq(s1 - s0)
            prod  = Signal((64, True)); self.comb += prod.eq(delta * frac_r)   # Q0.31
            outc  = Signal((32, True)); self.comb += outc.eq(s0 + (prod >> 31))
            self.sync += interp[c*32:(c+1)*32].eq(outc)
        have2 = Signal(); self.comb += have2.eq(level >= 2)   # 2 frames -> interp valid

        underruns = Signal(32)
        self.comb += [self.underrun_count.status.eq(underruns),
                      self.fifo_level.status.eq(level_u)]
        # Phase advance: acc = frac + src_step; integer part advances rd_int (0..2).
        acc = Signal(33)
        self.comb += acc.eq(frac + self.src_step.storage)

        # ---- consumer-side probe taps ----
        self.p_primed = Signal(); self.p_strobe = Signal()
        self.comb += [self.p_primed.eq(primed), self.p_strobe.eq(strobe)]

        # Soft-ILA counters: production (ring_wr), consumption (strobe), firsts,
        # raw ungated NCO tick, and true USB samples drained.
        _push_cnt = Signal(32); _pop_cnt = Signal(32); _first_cnt = Signal(32)
        _rawstr_cnt = Signal(32); _usbsamp_cnt = Signal(32)
        self.sync += [
            If(ring_wr,               _push_cnt.eq(_push_cnt + 1)),
            If(strobe,                _pop_cnt.eq(_pop_cnt + 1)),
            If(do_pop & first,        _first_cnt.eq(_first_cnt + 1)),
            If(mcr.sample_strobe,     _rawstr_cnt.eq(_rawstr_cnt + 1)),
            If(usb_readable & do_pop, _usbsamp_cnt.eq(_usbsamp_cnt + 1)),
        ]
        self.comb += [
            self.dbg_block_push.status.eq(_push_cnt),
            self.dbg_block_pop.status.eq(_pop_cnt),
            self.dbg_first.status.eq(_first_cnt),
            self.dbg_raw_strobe.status.eq(_rawstr_cnt),
            self.dbg_usb_samp.status.eq(_usbsamp_cnt),
        ]

        self.sync += [
            send_req.eq(0),
            If(strobe,
                If(have2,
                    # Data present: interpolate AND advance the read phase. The
                    # advance is gated by have2 so rd_int can NEVER overtake wr.
                    pay[Cat(blk_idx, fill_buf)].eq(interp),
                    frac.eq(acc[0:31]),
                    rd_int.eq(rd_int + acc[31:33]),    # advance by integer part (0..2)
                ).Else(
                    # Underrun: emit silence, HOLD the read phase.
                    pay[Cat(blk_idx, fill_buf)].eq(0),
                    underruns.eq(underruns + 1),
                ),
                If(blk_idx == (samples_per_packet - 1),
                    blk_idx.eq(0),
                    send_buf.eq(fill_buf),
                    fill_buf.eq(~fill_buf),
                    send_req.eq(1),
                ).Else(
                    blk_idx.eq(blk_idx + 1),
                ),
            ),
        ]

        # 3) Header byte vector (LSB index = first byte on the wire)
        # =========================================================
        src_mac = Cat(self.src_mac_lo.storage, self.src_mac_hi.storage)   # [0:48], byte0 = [40:48]
        dst_mac = Cat(self.dst_mac_lo.storage, self.dst_mac_hi.storage)
        sid     = Cat(self.stream_id_lo.storage, self.stream_id_hi.storage)  # [0:64], byte0 = [56:64]
        tci     = self.vlan_tci.storage

        seq  = Signal(8)
        pres = Signal(32)

        def mac_byte(sig, i):   # i=0 is the wire-first (MSB) byte of a 48-bit MAC
            hi = 48 - i * 8
            return sig[hi - 8:hi]

        def sid_byte(i):        # i=0 is the wire-first (MSB) byte of a 64-bit id
            hi = 64 - i * 8
            return sid[hi - 8:hi]

        hb = [
            mac_byte(dst_mac, 0), mac_byte(dst_mac, 1), mac_byte(dst_mac, 2),
            mac_byte(dst_mac, 3), mac_byte(dst_mac, 4), mac_byte(dst_mac, 5),   # 0..5
            mac_byte(src_mac, 0), mac_byte(src_mac, 1), mac_byte(src_mac, 2),
            mac_byte(src_mac, 3), mac_byte(src_mac, 4), mac_byte(src_mac, 5),   # 6..11
            Constant(0x81, 8), Constant(0x00, 8),                               # 12,13 TPID
            tci[8:16], tci[0:8],                                                # 14,15 VLAN TCI
            Constant(0x22, 8), Constant(0xF0, 8),                              # 16,17 ethertype 0x22F0
            Constant(0x02, 8),                                                 # 18 subtype = AAF
            Constant(0x81, 8),                                                 # 19 sv=1, tv=1
            seq,                                                                # 20 sequence_num
            Constant(0x00, 8),                                                 # 21 reserved | tu=0
            sid_byte(0), sid_byte(1), sid_byte(2), sid_byte(3),
            sid_byte(4), sid_byte(5), sid_byte(6), sid_byte(7),                # 22..29 stream_id
            pres[24:32], pres[16:24], pres[8:16], pres[0:8],                   # 30..33 avtp_timestamp be32
            Constant(0x02, 8),                                                 # 34 format = INT_32BIT
            Constant((nsr_ch >> 8) & 0xFF, 8),                                # 35 nsr|channels hi
            Constant(nsr_ch & 0xFF, 8),                                        # 36 nsr|channels lo
            Constant(0x20, 8),                                                 # 37 bit_depth = 32
            Constant((data_len >> 8) & 0xFF, 8),                              # 38 stream_data_length hi
            Constant(data_len & 0xFF, 8),                                      # 39 stream_data_length lo
            Constant(0x00, 8),                                                 # 40 sp=0, evt=0
            Constant(0x00, 8),                                                 # 41 reserved
        ]
        assert len(hb) == HDR_LEN
        header = Array(hb)

        # =========================================================
        # 4) Builder FSM: bytes → frame_ram, then stream frame_ram → source
        # =========================================================
        frame_ram = Array([Signal(32) for _ in range(N_WORDS)])
        byte_idx  = Signal(max=TOTAL + 1)
        wacc      = Signal(24)            # holds lanes 0..2 of the in-progress word
        rd_idx    = Signal(max=N_WORDS)
        pkt_count = Signal(32)
        overruns  = Signal(32)
        self.comb += [
            self.packet_count.status.eq(pkt_count),
            self.overrun_count.status.eq(overruns),
        ]

        # Current byte value: header for idx<42, else payload (big-endian sample).
        cur_byte = Signal(8)
        pi   = byte_idx - HDR_LEN                 # payload byte offset (valid when >=0)
        n    = pi[2:]                             # sample number within packet (0..47)
        k    = pi[0:2]                            # byte within sample (0..3)
        p_blk = n[3:]                             # block 0..5
        p_ch  = n[0:3]                            # channel 0..7
        blk_word = pay[Cat(p_blk[0:3], send_buf)]            # 256-bit block
        p_samp   = (blk_word >> (p_ch * 32))[0:32]           # selected channel sample
        p_byte   = (p_samp >> ((3 - k) * 8))[0:8]            # big-endian byte (put_be32)
        self.comb += If(byte_idx < HDR_LEN,
            cur_byte.eq(header[byte_idx[0:6]]),
        ).Else(
            cur_byte.eq(p_byte),
        )

        lane = byte_idx[0:2]
        widx = byte_idx[2:]

        fsm = FSM(reset_state="IDLE")
        self.submodules.fsm = fsm
        fsm.act("IDLE",
            If(send_req,
                NextValue(pres, (_mul_1e9_lo32(tsu.seconds)
                                 + tsu.nanoseconds
                                 + self.pres_offset.storage)[0:32]),
                NextValue(byte_idx, 0),
                NextState("BUILD"),
            ),
        )
        # BUILD: one byte/cycle into wacc; commit a word every 4th byte and on
        # the final (possibly partial) byte. 234 cycles ≈ 4.7 µs << 125 µs.
        commit_full = (lane == 3)
        is_last     = (byte_idx == (TOTAL - 1))
        fsm.act("BUILD",
            Case(lane, {
                0: NextValue(wacc[0:8],   cur_byte),
                1: NextValue(wacc[8:16],  cur_byte),
                2: NextValue(wacc[16:24], cur_byte),
            }),
            If(commit_full,
                NextValue(frame_ram[widx], Cat(wacc, cur_byte)),
            ),
            If(is_last,
                # Final word (rem=2 → lanes 0,1 valid). wacc[0:8]=byte232,
                # cur_byte=byte233; zero-pad the rest.
                NextValue(frame_ram[LAST_IDX],
                          Cat(wacc[0:8], cur_byte, Constant(0, 32 - rem * 8))
                          if rem else Cat(wacc, cur_byte)),
                NextValue(rd_idx, 0),
                NextState("STREAM"),
            ).Else(
                NextValue(byte_idx, byte_idx + 1),
            ),
        )
        fsm.act("STREAM",
            source.valid.eq(1),
            source.data.eq(frame_ram[rd_idx]),
            source.last.eq(rd_idx == LAST_IDX),
            If(rd_idx == LAST_IDX,
                source.last_be.eq(LAST_BE),
            ).Else(
                source.last_be.eq(0xF),
            ),
            If(source.ready,
                If(source.last,
                    NextValue(pkt_count, pkt_count + 1),
                    NextValue(seq, seq + 1),
                    NextState("IDLE"),
                ).Else(
                    NextValue(rd_idx, rd_idx + 1),
                ),
            ),
        )
        # Safety: a send_req while not IDLE means the builder fell behind
        # (should never happen — build+stream ≪ 6 strobe periods). Count it.
        self.sync += If(send_req & ~fsm.ongoing("IDLE"), overruns.eq(overruns + 1))
