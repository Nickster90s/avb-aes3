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
                 fifo_depth=64, streams=1):
        dw = 32
        ch_bits  = max(1, log2_int(channels, need_pow2=False))
        blk_bits = max(1, log2_int(samples_per_packet, need_pow2=False))

        # TIME-MUX geometry: `streams` AAF frames of `channels` each share one
        # media clock and one USB ingress. The host delivers BLOCK_CH interleaved
        # channels per media frame (ch0..ch47 for 6x8); the ring stores them in
        # arrival order and the reader strides out each stream's 8-ch slice.
        # Frame s sample (row r, slice-ch c) is at ring offset r*BLOCK_CH+s*ch+c;
        # the reader advances +1 within a slice and +ROW_JUMP at the row boundary
        # (proven in sims/sim_stride_timemux.py).
        self.streams = streams
        BLOCK_CH      = streams * channels                 # 48 for 6x8
        ROW_JUMP      = BLOCK_CH - channels + 1             # 41: ch7 -> next row's slice
        FRAME_SAMPLES = channels * samples_per_packet       # 48 samples in one AAF frame
        BLOCK_SAMPLES = BLOCK_CH * samples_per_packet        # 288 ring samples per block
        # samp_hi channel-index field width; `first` marker sits just above it.
        first_bit     = max(1, (BLOCK_CH - 1).bit_length()) # 6 for 48ch, 3 for 8ch
        st_bits       = max(1, log2_int(streams, need_pow2=False))

        # ---- Frame geometry (computed once, Python-side) ----
        HDR_LEN  = 42                                 # 14 eth + 4 vlan + 24 avtp
        PAY_LEN  = samples_per_packet * channels * 4  # 6*8*4 = 192
        TOTAL    = HDR_LEN + PAY_LEN                   # 234
        rem      = TOTAL % 4
        N_WORDS  = (TOTAL + 3) // 4                    # 59
        LAST_IDX = N_WORDS - 1
        # last_be is a ONE-HOT of the LAST VALID BYTE (LiteEth mac/sram.py:238-245:
        # 1 byte->0b0001, 2->0b0010, 3->0b0100, 4->0b1000), NOT a byte mask.
        # We had ((1<<rem)-1) = 0x3 for the 234-byte frame (rem=2) where LiteEth's
        # TX last_be stage + 32->8 converter expect 0x2 -> wrong final byte count
        # -> WRONG FRAME LENGTH -> BAD FCS -> every AAF frame dropped on the wire
        # as an RX error (firmware frames were fine because the SRAM reader sets
        # this correctly). THE bug that kept AAF off the wire end-to-end.
        LAST_BE  = (1 << 3) if rem == 0 else (1 << (rem - 1))

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
        # TIME-MUX: dst_mac + stream_id are PER-STREAM (indirect-addressed to keep
        # the CSR count low — see [[csr-mux-explodes-sys-clk]]). Firmware writes
        # ctx_select=s, then dst_mac_hi/lo + stream_id_hi/lo; the gateware latches
        # each {hi,lo} into context[s] on the _lo write strobe. streams=1 reduces
        # to the original single binding (ctx_select always 0).
        self.ctx_select    = CSRStorage(max(1, st_bits),
                             description="Per-stream binding context index (0..streams-1) for the writes below.")
        self.stream_id_hi  = CSRStorage(32, description="AVTP stream_id [63:32] for context ctx_select (latched on stream_id_lo write).")
        self.stream_id_lo  = CSRStorage(32, description="AVTP stream_id [31:0] for context ctx_select (write LAST -> latches the pair).")
        self.vlan_tci      = CSRStorage(16, reset=(3 << 13) | 2,
                             description="802.1Q TCI = (pcp<<13)|vid. Class A default pcp=3, vid=2.")
        self.pres_offset   = CSRStorage(32, reset=2_000_000,
                             description="presentation_time offset (ns) added to gPTP now. Milan AAF = 2 ms.")
        # REMOVED (2026-06-23): pres_base CSR. It fed the (now-deleted) pres-ramp
        # dilation, so it was dead; worse, on openXC7 the value mcr.c wrote into it
        # (gbase ~= the NCO increment) was ending up in the pres in place of
        # pres_offset -> cold-start avtp_ts = gPTP + ~increment (4123363) not +2ms.
        # The two adjacent 32-bit CSRs (pres_offset @0x24, pres_base @0x28) aliased.
        # Deleting pres_base removes the increment from the design entirely.

        # ---- CSRs: status (read-only diagnostics) ----
        self.packet_count   = CSRStatus(32, description="AAF frames transmitted.")
        self.underrun_count = CSRStatus(32, description="Media-clock ticks where block_fifo was empty (silence inserted).")
        self.overrun_count  = CSRStatus(32, description="send_req arriving while builder busy (packet skipped — should stay 0).")
        self.fifo_level     = CSRStatus(blk_bits + 8, description="block_fifo occupancy (blocks).")
        # Timestamp instrument: the ACTUAL emitted avtp_timestamp (pres) and the
        # live gPTP-ns(low32) latched at the SAME packet emit. effective offset =
        # dbg_last_pres - dbg_emit_gptp should hold ~= pres_offset (2ms); if it
        # drifts/goes negative the pres ramp is wrong (chasing AxC Late-Timestamp).
        self.dbg_last_pres  = CSRStatus(32, description="Last emitted AAF avtp_timestamp (gateware pres).")
        self.dbg_emit_gptp  = CSRStatus(32, description="gPTP ns low32 at that emit (pres - this = effective offset).")
        _dbg_pres_r = Signal(32)
        _dbg_gptp_r = Signal(32)
        self.comb += [self.dbg_last_pres.status.eq(_dbg_pres_r),
                      self.dbg_emit_gptp.status.eq(_dbg_gptp_r)]
        # NOTE: the soft-ILA debug CSRs (dbg_block_push/pop/first, level min/max,
        # raw_strobe, usb_samp, pres, frame_addr/data) were REMOVED 2026-06-12.
        # They had done their diagnostic job, and the large AAF CSR bank was the
        # sys_clk critical path: the CPU->CSR-decode routing (csrbank10) capped sys
        # at 43.97 MHz, and sub-50 setup violations corrupted the AAF builder's
        # high-bit channel paths (ch1/3/5/6 = wrong MSBs, low bytes clean). Trimming
        # the bank shrinks that decode mux. See [[csr-mux-explodes-sys-clk]].

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
        # SAMPLE ring (32-bit entries) in BRAM — holds the raw interleaved stream
        # ch0,ch1,...,ch7,ch0,... read ONE SAMPLE AT A TIME by the byte builder.
        # No 256-bit frame is ever assembled in LUTs/FFs, so none of the wide
        # variable muxes / wide shift registers that openXC7 mis-synthesised
        # (pink noise, ch1/3/5/6, ch2/4) exist anymore — only BRAM is wide. depth
        # = 8x the old frame depth (same buffering time). Read pipeline verified in
        # /tmp/sim_sring.py.
        # SIX de-interleaved 8-ch rings (one per AAF stream), NOT one 48-ch ring with
        # a strided read. The 48ch USB frame is DEMUXED by channel_nr -> bundle=nr>>
        # ch_bits, within=nr&(channels-1); each ring is then written + read EXACTLY
        # like the proven 1x8ch path (contiguous, pow2 depth, no stride). This
        # replicates the working 8ch datapath x6 instead of the fragile shared-ring
        # strided read (which produced garbage + dropped streams on HW). Sim:
        # sims/sim_six_ring_demux.py. depth = per-ring (8ch) buffering, next pow2.
        _need_depth = fifo_depth * channels        # per-ring samples of buffering
        SRING_DEPTH = 1 << max(1, (_need_depth - 1).bit_length())
        log2depth   = log2_int(SRING_DEPTH)
        # 36-bit entries: the 32 sample bits are spread to AVOID the RAMB36 parity
        # positions (8,17,26,35), which nextpnr-xilinx drops when packing a 32-bit
        # BRAM. Dummies land on parity; real data survives. See pack/unpack below.
        mems = [Memory(36, SRING_DEPTH) for _ in range(streams)]
        wps  = []
        rps  = []
        for _m in mems:
            self.specials += _m
            _wp = _m.get_port(write_capable=True)
            _rp = _m.get_port()
            self.specials += _wp, _rp
            wps.append(_wp)
            rps.append(_rp)

        # Retained for CSR-layout / firmware compat — UNUSED now. The host is
        # rate-slaved by USB async feedback (tracks our NCO rate), so this is a
        # bit-exact passthrough: no resampler, no per-channel multiplier.
        self.src_step = CSRStorage(32, reset=1 << 31,
            description="(unused) legacy SRC ratio; bit-exact passthrough now.")
        self.fifo_depth = fifo_depth

        wr     = Signal(32)                         # sample write pointer
        rd     = Signal(32)                         # sample read/consumed pointer
        # SIGNED sample occupancy (negative if consumer overtakes producer).
        level  = Signal((34, True))
        self.comb += level.eq(wr - rd)
        # block_level for the wrapper PI servo — SAME scale as the old frame level
        # (samples >> 3 = frames), so the servo CENTER/KP/clamps carry over with no
        # change to the wrapper or the .v.
        # Scale the ring fill to 0..128 (servo CENTER=64) regardless of BLOCK_CH:
        # SRING_DEPTH >> _bl_sh == 128, so a half-full ring reads ~64. (8ch: shift
        # 5 == the old >>5; 48ch ring 32768 -> shift 8.)
        _bl_sh = log2depth - 7
        # Full-ring scale 0..128 (= SRING_DEPTH>>_bl_sh). This was CAPPED at
        # fifo_depth (64) = HALF the ring, which (a) hid the upper half from the
        # diagnostics and (b) made err=CENTRE-level ALWAYS >=0 in the .v feedback
        # loop -> the servo could only push the host FASTER, never slower -> NO
        # authority to pull a full ring back down (and no symmetric center for the
        # new integral). CENTRE=64 is now true mid; un-cap to the full 0..128.
        _bl_max = SRING_DEPTH >> _bl_sh                # 128 = full ring (any depth)
        self.block_level = Signal(max=_bl_max + 1)
        self.comb += If(level < 0,
            self.block_level.eq(0),
        ).Elif((level >> _bl_sh) > _bl_max,
            self.block_level.eq(_bl_max),
        ).Else(
            self.block_level.eq(level >> _bl_sh),   # ring samples -> 0..128, CENTER=mid
        )
        level_u = Signal(max=_bl_max + 1)
        self.comb += level_u.eq(self.block_level)   # fifo_level CSR = frame-equiv
        # rp.adr is driven by the byte builder's fetch pointer (rdf), below.

        # CHANNEL-ADDRESSED write (ADAT BundleDemultiplexer style): place every USB
        # sample at frame_base + channel_nr, NOT sequentially. The decoder labels
        # each sample with its explicit channel (0..BLOCK_CH-1) plus a `first`(ch0)
        # marker; frame_base advances ONE media-frame (BLOCK_CH) per `first`. The old
        # SEQUENTIAL write (wr+=1) inferred channel from POSITION, so a single dropped
        # sample (ring full) shifted the write phase and ROTATED ALL channels
        # permanently — the 48ch "corrupt after a while". 8ch survived only because
        # that rotation was a constant offset fixed once by chan_rot. With explicit
        # addressing a drop leaves ONE stale channel slot (a tiny per-channel glitch)
        # and the frame grid self-aligns every `first`, so drift heals within one
        # media-frame. Register the bridge output first (stable, timing-clean).
        do_pop = usb_readable
        samp_lo_r = Signal(32)
        samp_hi_r = Signal(first_bit + 1)   # [0:first_bit]=channel(0..BLOCK_CH-1), [first_bit]=first
        samp_vld  = Signal()
        self.sync += [
            samp_vld.eq(do_pop),
            If(do_pop,
                samp_lo_r.eq(usb_sample_lo),
                samp_hi_r.eq(usb_sample_hi[0:first_bit + 1]),
            ),
        ]
        first   = samp_hi_r[first_bit]              # 48ch-frame ch0 marker (channel_nr==0)
        chan_nr = samp_hi_r[0:first_bit]            # explicit channel 0..BLOCK_CH-1
        ch_bits = max(1, log2_int(channels))        # 3 for 8ch
        bundle  = chan_nr[ch_bits:]                 # which ring   (channel_nr >> 3)
        within  = chan_nr[0:ch_bits]                # ch in bundle (channel_nr & 7)
        samp32  = samp_lo_r                         # true 32-bit, no truncation
        en = self.enable.storage
        started   = Signal()                        # high once the first 48ch ch0 seen
        # frame_base = per-ring base of the current 8-sample frame; advances by
        # `channels` per `first` (one 48ch USB frame fills one 8-sample frame in EVERY
        # ring). within addresses the channel inside that frame -> a dropped sample is
        # one stale slot in one ring, never a rotation.
        frame_base = Signal(32)
        new_base   = Signal(32)
        self.comb += new_base.eq(frame_base + channels)
        base_now   = Signal(32)
        self.comb += base_now.eq(Mux(samp_vld & first, new_base, frame_base))
        wr_addr    = Signal(32)
        self.comb += wr_addr.eq(base_now + within)
        rd_anchor  = Signal(32)
        have_space = Signal()
        self.comb += have_space.eq(level < (SRING_DEPTH - 2))
        do_write = Signal()
        self.comb += [
            self.usb_pop.eq(do_pop),                # always drain the bridge FIFO
            do_write.eq(en & samp_vld & (started | first) & have_space),
        ]
        packed = Signal(36)                          # data spread around parity (8,17,26,35)
        self.comb += packed.eq(Cat(samp32[0:8],  Constant(0, 1),
                                   samp32[8:16], Constant(0, 1),
                                   samp32[16:24],Constant(0, 1),
                                   samp32[24:32],Constant(0, 1)))
        # DEMUX: drive all rings; only ring[bundle] takes the write this cycle.
        for _s in range(streams):
            self.comb += [
                wps[_s].adr.eq(wr_addr[0:log2depth]),
                wps[_s].dat_w.eq(packed),
                wps[_s].we.eq(do_write & (bundle == _s)),
            ]
        # wr (for level/servo) = frame_base: all rings filled up to here. Advance one
        # 8-sample frame per `first` REGARDLESS of have_space so the grid self-aligns.
        self.comb += wr.eq(frame_base)
        self.sync += [
            If(~en, started.eq(0)).Elif(samp_vld & first, started.eq(1)),
            If(samp_vld & first & (started | first), frame_base.eq(new_base)),
            If(en & ~started & samp_vld & first, rd_anchor.eq(new_base)),
        ]

        # =========================================================
        # 2) Media-clock-paced SRC read -> pay ping-pong buffer
        # =========================================================
        # 2) Media-clock PACING: one packet every samples_per_packet (=6) media
        #    strobes. The byte builder (section 4) drains 48 samples from the ring
        #    per packet and advances `rd` there — no per-strobe frame read, no wide
        #    packet accumulator. Nothing here is wider than a counter.
        blk_idx  = Signal(blk_bits)
        send_req = Signal()
        # Prime to half-full (in SAMPLES) before consuming real audio; until then
        # the builder emits silence so the listener (AxC) can still lock.
        primed  = Signal()
        _center = SRING_DEPTH // 2                    # half the ring, in samples
        # HYSTERESIS: prime at half-full, but UN-PRIME on underrun (level < one
        # packet of samples). Without the un-prime, when the ring drains (USB
        # stopped / host not feeding), the reader kept advancing and WRAPPED through
        # stale ring data forever -> continuous noise on the live channels even with
        # NO USB input (the smoking-gun symptom). Un-priming makes the builder emit
        # silence (samp_hold=0, rd holds) until the ring refills past center.
        self.sync += [
            If(~en, primed.eq(0)
            ).Elif(level >= _center, primed.eq(1)
            ).Elif(level < FRAME_SAMPLES, primed.eq(0)),
        ]
        strobe = Signal()
        self.comb += strobe.eq(mcr.sample_strobe & en)
        underruns = Signal(32)
        self.comb += [self.underrun_count.status.eq(underruns),
                      self.fifo_level.status.eq(level_u)]
        self.sync += [
            send_req.eq(0),
            # underrun = media tick spent emitting silence (ring below the prime
            # floor). This counter was DEAD — declared + read into the CSR but
            # NEVER incremented — so every silence glitch was invisible
            # (underrun_count stuck at 0). Counts ticks, so it is severity-weighted;
            # a nonzero baseline at startup (initial prime) is expected, watch the
            # DELTA after audio is flowing.
            If(strobe & ~primed, underruns.eq(underruns + 1)),
            If(strobe,
                If(blk_idx == (samples_per_packet - 1),
                    blk_idx.eq(0),
                    send_req.eq(1),
                ).Else(
                    blk_idx.eq(blk_idx + 1),
                ),
            ),
        ]

        # 3) Header byte vector (LSB index = first byte on the wire)
        # =========================================================
        # Per-stream binding contexts (indirect-written above). dst_mac + stream_id
        # are selected by `stream_idx` (the frame being built); src_mac/tci/pres are
        # shared. Array indexing -> a clean Case mux (NOT a barrel shift), safe on
        # openXC7. seq is per-stream.
        sid_arr  = Array([Signal(64) for _ in range(streams)])
        dmac_arr = Array([Signal(48) for _ in range(streams)])
        seq_arr  = Array([Signal(8)  for _ in range(streams)])
        ctx_sel  = self.ctx_select.storage
        self.sync += [
            If(self.stream_id_lo.re,
               sid_arr[ctx_sel].eq(Cat(self.stream_id_lo.storage, self.stream_id_hi.storage))),
            If(self.dst_mac_lo.re,
               dmac_arr[ctx_sel].eq(Cat(self.dst_mac_lo.storage, self.dst_mac_hi.storage))),
        ]

        stream_idx = Signal(max=streams) if streams > 1 else Signal()
        src_mac = Cat(self.src_mac_lo.storage, self.src_mac_hi.storage)   # [0:48], byte0 = [40:48]
        dst_mac = dmac_arr[stream_idx]                                    # selected context
        sid     = sid_arr[stream_idx]                                     # [0:64], byte0 = [56:64]
        tci     = self.vlan_tci.storage

        seq  = seq_arr[stream_idx]                                        # per-stream sequence_num
        pres = Signal(32)

        # ---- Deterministic CRF-dilated presentation-time ramp (gst-avtp model) ----
        # Instead of re-sampling gPTP every packet (which carries strobe->latch
        # jitter and a possible 1-second glitch at the TSU seconds/ns wrap), we
        # anchor the gPTP time ONCE and advance avtp_ts by a FIXED per-packet
        # period, DILATED to the CRF media-clock rate via the MCR servo
        # increment. The result is a perfectly smooth avtp_ts that tracks
        # Auvitran's crystal — exactly what its media-clock PLL needs to hold
        # lock. Mirrors gstavtpaafpay.c: launch_ns = anchor + samples*1e9/rate,
        # then dilation_correct().  First-order dilation (servo deviation is tiny):
        #   period_ns = P0 * base/inc  ~=  P0 - (inc-base)*(P0/base)
        # accumulated with _PRES_F fractional bits so there is no rounding drift.
        _PRES_F  = 16
        _P0_ns   = int(round(samples_per_packet * 1_000_000_000 / 48000))  # 125000 @48k/6
        _base    = mcr.base_increment
        _Kfix    = int(round((_P0_ns / _base) * (1 << _PRES_F)))           # ns per inc-unit, Q_F
        pres_acc  = Signal(32 + _PRES_F)
        anchored  = Signal()
        anchor_ns = Signal(32)
        self.comb += anchor_ns.eq((_mul_1e9_lo32(tsu.seconds)
                                   + tsu.nanoseconds
                                   + self.pres_offset.storage)[0:32])
        # PURE RE-ANCHOR: pres = gPTP_now + offset EVERY packet, registered ONE cycle
        # to keep the *1e9 shift-add tree off the critical path (anchor_ns_r ~20ns
        # stale = negligible). Re-sampling gPTP every packet pins eff_offset =
        # pres_offset (+2ms) by construction; do_emit is media-clock-paced so avtp_ts
        # spacing stays smooth and the AxC recovers a clean rate.
        #
        # DELETED (2026-06-23): the dilated-ramp + PLL machinery (dinc = mcr.increment
        # - pres_base, step_scaled, pres_err, pres_corr). It was already unused (Python
        # `_ = (...)` discard), BUT it was still SYNTHESIZED, and on openXC7 the dead
        # increment-math merged into the live pres tree -> at cold start the pres leaked
        # ~the NCO increment (HW: eff_offset = gw_pres-gw_gptp read 4123363 = `inc`, not
        # the 2,000,000 ns pres_offset; a manual re-anchor "trick" cleared it). Removing
        # the dead increment-using logic removes the only path the increment could leak.
        anchor_ns_r = Signal(32)
        self.sync += anchor_ns_r.eq(anchor_ns)              # register the *1e9 tree output ONLY
        new_acc = Signal(32 + _PRES_F)
        self.comb += new_acc.eq(Cat(Constant(0, _PRES_F), anchor_ns_r))
        _ = anchored  # (latched in the do_emit block below)

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
        # Ring-full write-drops: a USB sample arrives while ~have_space, so the
        # channel-addressed write is SUPPRESSED (line ~358) -> ONE stale channel
        # slot = an audible per-channel click. This was UNCOUNTED — overrun_count
        # only caught builder-busy skips, so a ring hitting the full rail was
        # invisible. Count it and fold into overrun_count.
        write_drops = Signal(32)
        self.sync += If(en & samp_vld & (started | first) & ~have_space,
                        write_drops.eq(write_drops + 1))
        self.comb += [
            self.packet_count.status.eq(pkt_count),
            self.overrun_count.status.eq(overruns + write_drops),
        ]

        # Current byte: header (idx<42), else the current sample's bytes (big-endian,
        # MSB first) from a 32-bit samp_hold register. samp_hold is fed one sample at
        # a time by the BRAM ring read pipeline in BUILD (rdf leads, latched at the
        # sample's last byte) — verified in /tmp/sim_sring.py. NOTHING here is wider
        # than 32 bits, so no wide LUT/FF structure for openXC7 to mis-synthesise
        # (the per-channel muxes AND the wide shift registers are both gone).
        cur_byte   = Signal(8)
        samp_hold  = Signal(32)
        rdf        = Signal(32)              # strided ring fetch address (combinational)
        pkt_primed = Signal()                # `primed` latched at packet start
        # Strip parity dummies; the sample comes from the SELECTED stream's ring
        # (stream_idx picks which of the de-interleaved rings).
        samp_rd = Signal(32)
        rdat    = Signal(36)
        self.comb += rdat.eq(Array([rps[_s].dat_r for _s in range(streams)])[stream_idx])
        self.comb += samp_rd.eq(Cat(rdat[0:8], rdat[9:17], rdat[18:26], rdat[27:35]))
        pi = Signal(max=TOTAL + 1)
        self.comb += pi.eq(byte_idx - HDR_LEN)         # payload byte offset (k = pi[0:2])
        self.comb += If(byte_idx < HDR_LEN,
            cur_byte.eq(header[byte_idx[0:6]]),
        ).Else(
            Case(pi[0:2], {
                0: cur_byte.eq(samp_hold[24:32]),
                1: cur_byte.eq(samp_hold[16:24]),
                2: cur_byte.eq(samp_hold[8:16]),
                3: cur_byte.eq(samp_hold[0:8]),
            }),
        )

        lane = byte_idx[0:2]
        widx = byte_idx[2:]

        # CONTIGUOUS read: `sc` = sample index 0..FRAME_SAMPLES-1, PREFETCHED on the
        # ring address; samp_hold lags rp.dat_r by one (BUILD pipeline). Each stream
        # reads its OWN 8-ch ring straight out (rd + sc) — no stride, no row math (the
        # de-interleave already happened at the demux write). All rings share the
        # address; stream_idx selects the data (samp_rd above).
        sc = Signal(max=FRAME_SAMPLES + 2)
        self.comb += rdf.eq(rd + sc)
        for _s in range(streams):
            self.comb += rps[_s].adr.eq(rdf[0:log2depth])
        aligned_once = Signal()   # read anchored to the first block (anchor-once align)
        fsm = FSM(reset_state="IDLE")
        self.submodules.fsm = fsm
        fsm.act("IDLE",
            If(send_req,
                NextValue(byte_idx, 0),
                NextValue(sc, 0),                 # prefetch sample 0 of frame 0
                NextValue(stream_idx, 0),         # first stream of the block
                NextValue(pkt_primed, primed),    # whole block is silence OR audio
                If(~aligned_once, NextValue(rd, rd_anchor)),  # first block: anchor the read
                NextState("BUILD"),
            ),
        )
        # Presentation-time accumulator: latch on the IDLE->BUILD transition.
        # Anchor on the first packet (and re-anchor whenever the talker is
        # disabled, so a restarted stream gets a fresh anchor), then advance by
        # one dilated packet period each packet.
        do_emit = Signal()
        self.comb += do_emit.eq(send_req & fsm.ongoing("IDLE"))
        self.sync += [
            If(~en,
                anchored.eq(0),
                aligned_once.eq(0),               # re-anchor the read on re-enable
            ).Elif(do_emit,
                pres_acc.eq(new_acc),
                pres.eq(new_acc[_PRES_F:_PRES_F + 32]),
                anchored.eq(1),
                aligned_once.eq(1),
                # Timestamp instrument: capture the emitted pres + the raw gPTP now.
                _dbg_pres_r.eq(new_acc[_PRES_F:_PRES_F + 32]),
                _dbg_gptp_r.eq((_mul_1e9_lo32(tsu.seconds) + tsu.nanoseconds)[0:32]),
            ),
        ]
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
            # BRAM ring read pipeline (verified sim_sring.py), overlapped with the
            # header so the first payload sample is ready at byte 42:
            #   byte 0: rp.adr=rdf=rd this cycle -> bump rdf so dat_r prefetches rd+1
            #   byte 1: latch samp_hold = ring[rd]
            #   payload last byte (pi&3==3): latch next sample, advance rd + rdf
            # A silence packet (~pkt_primed) holds samp_hold=0 and does NOT advance
            # rd, so the ring fills until primed; 48 samples/packet keeps ch-align.
            If(pkt_primed,
                If(byte_idx == 0, NextValue(sc, 1)),         # prefetch sample 1 during header
                If(byte_idx == 1, NextValue(samp_hold, samp_rd)),   # latch sample 0
                If((byte_idx >= HDR_LEN) & (pi[0:2] == 3),
                    NextValue(samp_hold, samp_rd),           # latch the prefetched sample
                    NextValue(sc, sc + 1),                   # strided fetch advances via rdf(sc)
                ),
            ).Else(
                NextValue(samp_hold, 0),
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
                    NextValue(seq, seq + 1),                 # per-stream (seq = seq_arr[stream_idx])
                    If(stream_idx != (streams - 1),
                        # More frames in this block: build the next stream's slice
                        # from the SAME 288-sample block (rd unchanged).
                        NextValue(stream_idx, stream_idx + 1),
                        NextValue(byte_idx, 0),
                        NextValue(sc, 0),
                        NextState("BUILD"),
                    ).Else(
                        # Block done: every ring advances FRAME_SAMPLES (6 frames x
                        # 8ch); all rings share `rd` (read in lockstep). Audio only.
                        If(pkt_primed, NextValue(rd, rd + FRAME_SAMPLES)),
                        NextState("IDLE"),
                    ),
                ).Else(
                    NextValue(rd_idx, rd_idx + 1),
                ),
            ),
        )
        # Safety: a send_req while not IDLE means the builder fell behind
        # (should never happen — build+stream ≪ 6 strobe periods). Count it.
        self.sync += If(send_req & ~fsm.ongoing("IDLE"), overruns.eq(overruns + 1))
