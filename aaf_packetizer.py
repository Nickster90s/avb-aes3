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

        # MAC error lane is always 0 for our generated frames.
        self.comb += source.error.eq(0)

        # =========================================================
        # 1) USB ingress → 8-channel frame assembler → block_fifo
        # =========================================================
        block_fifo = SyncFIFO(width=channels * 32, depth=fifo_depth)
        self.submodules.block_fifo = block_fifo

        # Live block-FIFO occupancy (0..fifo_depth), exposed for the USB
        # feedback flow-control loop in avb_soc (drives the host to keep this
        # FIFO centred). self.fifo_depth lets the loop compute the midpoint.
        self.block_level = Signal(max=fifo_depth + 1)
        self.fifo_depth  = fifo_depth
        self.comb += self.block_level.eq(block_fifo.level)

        cur  = Array([Signal(32) for _ in range(channels)])
        have = Signal()

        ch    = usb_sample_hi[0:ch_bits]
        first = usb_sample_hi[3]
        # 24-bit audio MSB-aligned into a 32-bit sample (= firmware v & 0xFFFFFF00).
        samp32 = Cat(Signal(8), usb_sample_lo[8:32])   # [0:8]=0, [8:32]=audio

        # A `first` sample closes the previous frame: push it, then start fresh.
        need_push = first & have
        # Only consume from the wrapper when we either don't need to push, or
        # the block_fifo can accept the pushed block — otherwise stall (the
        # sample waits in the wrapper's FIFO; its own overflow counter covers
        # true loss). Gated by enable so firmware keeps the path when disabled.
        do_pop = self.enable.storage & usb_readable & (~need_push | block_fifo.writable)
        self.comb += self.usb_pop.eq(do_pop)

        self.comb += [
            block_fifo.din.eq(Cat(*cur)),
            block_fifo.we.eq(do_pop & need_push),
        ]
        self.sync += [
            If(do_pop,
                If(need_push,
                    # Start a new frame: zero all, drop this sample in its lane.
                    *[cur[i].eq(0) for i in range(channels)],
                ),
                cur[ch].eq(samp32),
                have.eq(1),
            ),
        ]

        # =========================================================
        # 2) Media-clock-paced read: block_fifo → pay ping-pong buffer
        # =========================================================
        # pay addressed as fill*8+blk (depth 16 keeps the address a clean
        # Cat(blk[3], buf[1]); slots 6,7 of each half are unused).
        pay      = Array([Signal(channels * 32) for _ in range(16)])
        fill_buf = Signal()
        send_buf = Signal()
        blk_idx  = Signal(blk_bits)
        send_req = Signal()

        # Egress runs only when the gateware owns the stream. The NCO strobe
        # free-runs regardless of enable, so gating here keeps us off the wire
        # (FSM stays IDLE, source.valid=0) while firmware holds the path.
        strobe = Signal()
        self.comb += strobe.eq(mcr.sample_strobe & self.enable.storage)
        underruns = Signal(32)
        self.comb += [
            block_fifo.re.eq(strobe & block_fifo.readable),
            self.underrun_count.status.eq(underruns),
            self.fifo_level.status.eq(block_fifo.level),
        ]
        self.sync += [
            send_req.eq(0),
            If(strobe,
                # One block per media-clock tick: real data or silence.
                If(block_fifo.readable,
                    pay[Cat(blk_idx, fill_buf)].eq(block_fifo.dout),
                ).Else(
                    pay[Cat(blk_idx, fill_buf)].eq(0),
                    underruns.eq(underruns + 1),
                ),
                If(blk_idx == (samples_per_packet - 1),
                    blk_idx.eq(0),
                    send_buf.eq(fill_buf),     # hand the just-filled half to the builder
                    fill_buf.eq(~fill_buf),
                    send_req.eq(1),
                ).Else(
                    blk_idx.eq(blk_idx + 1),
                ),
            ),
        ]
        # blk index within pay address is 3 bits (0..5) regardless of blk_bits.
        # Re-do the address with a fixed 3-bit blk field for the *fill* path.
        # (Cat above uses blk_idx width = blk_bits = 3 for spp=6, so this is
        #  already 3 bits; assert to catch a non-default spp.)
        assert (samples_per_packet - 1) < 8, "pay address assumes blk fits in 3 bits"

        # =========================================================
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
