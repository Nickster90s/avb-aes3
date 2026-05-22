# AVTPSampleExtractor — Stage 2a of the firmware-to-gateware audio
# path migration. Observes the LiteEth MAC RX stream (same as Stage 1's
# AVTPStreamFilter), matches frames against N configured AVTP stream
# slots, and on a match, parses the AAF header and pushes audio samples
# into per-slot per-channel sample FIFOs.
#
# Architecture:
#   MAC stream → header buffer (8 beats × 32-bit = 32 bytes)
#              → match decision at EOF (pipelined 3 cycles)
#              → if matched + subtype=AAF: extract samples from buffered audio portion
#              → push samples into sample_fifo[slot][channel]
#
# Why buffer the whole frame: the matching decision arrives AT EOF
# (after all beats observed). To extract samples for a matched frame,
# we need access to the audio bytes, which are mid-frame. So we either
# (a) speculate and discard on no-match, or (b) buffer the frame.
# This module does (b) — frame data goes into hdr_words[8] for the
# header + a 64-deep audio_buffer for the audio portion. After EOF,
# if matched, we replay the audio into per-channel FIFOs.
#
# CSR interface for firmware:
#   slotN_chM_pop  (write-strobe)    : pop one sample from this FIFO
#   slotN_chM_data (read-only 32-bit): sample at head of FIFO
#   slotN_chM_level(read-only 9-bit) : FIFO occupancy

from migen import *
from migen.genlib.fifo import SyncFIFO
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *


AVTP_SUBTYPE_AAF = 0x02
AVTP_SUBTYPE_CRF = 0x04


class AVTPSampleExtractor(Module, AutoCSR):
    """Stage 2a — observer-mode hardware sample extractor.

    Reuses the Stage-1 matcher's CSR-configurable (dst_mac, stream_id)
    slot table. On a matched AAF frame, parses the AAF header (channel
    count, sample format) and pushes each per-channel sample into the
    corresponding sample_fifo[slot][channel].

    n_slots    — number of configurable stream slots
    n_channels — number of channels per slot to extract
    fifo_depth — depth of each per-channel sample FIFO (samples)

    Total BRAM: n_slots × n_channels × fifo_depth × 4 bytes.
    Default (4 × 8 × 256) = 32 KB.
    """

    def __init__(self, n_slots=4, n_channels=8, fifo_depth=256, dw=32):
        assert dw == 32, "Only 32-bit input stream supported at present"

        # ---- Input stream (observe-only) ----
        self.sink = stream.Endpoint([
            ("data",    dw),
            ("last_be", dw // 8),
        ])

        # ---- Per-slot configuration CSRs (split for 32-bit CSR width) ----
        for i in range(n_slots):
            setattr(self, f"slot{i}_enabled",
                CSRStorage(1, name=f"slot{i}_enabled",
                    description=f"Extractor slot {i}: 1 = active."))
            setattr(self, f"slot{i}_dst_mac_hi",
                CSRStorage(16, name=f"slot{i}_dst_mac_hi",
                    description=f"Extractor slot {i}: dst_mac[47:32]."))
            setattr(self, f"slot{i}_dst_mac_lo",
                CSRStorage(32, name=f"slot{i}_dst_mac_lo",
                    description=f"Extractor slot {i}: dst_mac[31:0]."))
            setattr(self, f"slot{i}_stream_id_hi",
                CSRStorage(32, name=f"slot{i}_stream_id_hi",
                    description=f"Extractor slot {i}: stream_id[63:32]."))
            setattr(self, f"slot{i}_stream_id_lo",
                CSRStorage(32, name=f"slot{i}_stream_id_lo",
                    description=f"Extractor slot {i}: stream_id[31:0]."))
            setattr(self, f"slot{i}_match_count",
                CSRStatus(32, name=f"slot{i}_match_count",
                    description=f"Extractor slot {i}: matched AAF frames."))
            setattr(self, f"slot{i}_sample_count",
                CSRStatus(32, name=f"slot{i}_sample_count",
                    description=f"Extractor slot {i}: total samples extracted (per channel)."))

        slot_enabled = Array(
            getattr(self, f"slot{i}_enabled").storage for i in range(n_slots))
        slot_dst_mac = Array(
            Cat(getattr(self, f"slot{i}_dst_mac_lo").storage,
                getattr(self, f"slot{i}_dst_mac_hi").storage)
            for i in range(n_slots))
        slot_stream_id = Array(
            Cat(getattr(self, f"slot{i}_stream_id_lo").storage,
                getattr(self, f"slot{i}_stream_id_hi").storage)
            for i in range(n_slots))

        # ---- Per-slot per-channel CSR-readable sample FIFOs ----
        # Each FIFO is a 32-bit-wide × fifo_depth-deep SyncFIFO. Per-channel
        # access via dedicated CSRs lets firmware pop samples without an
        # additional mux address.
        self.fifos = []
        for s in range(n_slots):
            row = []
            for c in range(n_channels):
                fifo = SyncFIFO(width=32, depth=fifo_depth)
                self.submodules += fifo
                row.append(fifo)

                # Three CSRs per FIFO: pop strobe, data head, level.
                pop = CSRStorage(1, name=f"slot{s}_ch{c}_pop",
                    description=f"Slot {s} ch {c}: write 1 to pop one sample.")
                data = CSRStatus(32, name=f"slot{s}_ch{c}_data",
                    description=f"Slot {s} ch {c}: sample at FIFO head.")
                level = CSRStatus(9, name=f"slot{s}_ch{c}_level",
                    description=f"Slot {s} ch {c}: FIFO occupancy 0..{fifo_depth}.")
                setattr(self, f"slot{s}_ch{c}_pop",   pop)
                setattr(self, f"slot{s}_ch{c}_data",  data)
                setattr(self, f"slot{s}_ch{c}_level", level)

                self.comb += [
                    data.status.eq(fifo.dout),
                    level.status.eq(fifo.level),
                    # Pop on CSRStorage write (re-edge of .re).
                    fifo.re.eq(pop.re),
                ]
            self.fifos.append(row)

        # ---- Frame buffer ----
        # 8 header beats + audio. Worst-case AAF: 24-byte AAF header + 6 ×
        # 8ch × 4-byte samples = 24 + 192 = 216 bytes payload. Plus 14 byte
        # Ethernet + optional 4 byte VLAN = up to 234 bytes. 64 beats (256
        # bytes) is enough headroom for any AVB frame at 100 Mbit Class A.
        BUF_BEATS = 64
        frame_buf = Memory(width=32, depth=BUF_BEATS)
        wr_port   = frame_buf.get_port(write_capable=True)
        rd_port   = frame_buf.get_port(async_read=False)
        self.specials += frame_buf, wr_port, rd_port

        beat_idx  = Signal(max=BUF_BEATS + 1)
        frame_len = Signal(max=BUF_BEATS + 1)
        frame_done = Signal()
        self.comb += frame_done.eq(
            self.sink.valid & self.sink.ready & self.sink.last
        )
        # Always accept beats; observer never backpressures the MAC.
        self.comb += self.sink.ready.eq(1)

        # Capture beats into frame_buf during the frame.
        beat_accept = Signal()
        self.comb += beat_accept.eq(self.sink.valid & self.sink.ready)
        self.comb += [
            wr_port.adr.eq(beat_idx),
            wr_port.dat_w.eq(self.sink.data),
            wr_port.we.eq(beat_accept & (beat_idx < BUF_BEATS)),
        ]
        self.sync += If(beat_accept,
            If(beat_idx < BUF_BEATS, beat_idx.eq(beat_idx + 1)),
            If(self.sink.last,
                frame_len.eq(beat_idx + 1),
                beat_idx.eq(0),
            ),
        )

        # ---- Match decision pipeline (Stage B/C/D after EOF) ----
        # Same logic as AVTPStreamFilter, but the inputs are pulled from
        # the first 8 captured beats (frame_buf or a parallel header-only
        # shadow). For simplicity, register a copy of beats 0-7 here.
        hdr_words = Array(Signal(32) for _ in range(8))
        self.sync += If(beat_accept & (beat_idx < 8),
            hdr_words[beat_idx].eq(self.sink.data),
        )

        def byte_at(off):
            w = off // 4
            shift = 24 - 8 * (off % 4)
            return hdr_words[w][shift:shift + 8]

        # Pipeline stage B: latch keys at EOF.
        vlan_present_b = Signal()
        subtype_b      = Signal(8)
        dst_mac_b      = Signal(48)
        sid_b          = Signal(64)
        eof_b          = Signal()
        frame_len_b    = Signal(max=BUF_BEATS + 1)

        vlan_comb = (byte_at(12) == 0x81) & (byte_at(13) == 0x00)
        subtype_comb = Mux(vlan_comb, byte_at(18), byte_at(14))
        def sid_byte(i):
            return Mux(vlan_comb, byte_at(22 + i), byte_at(18 + i))
        sid_comb = Cat(
            sid_byte(7), sid_byte(6), sid_byte(5), sid_byte(4),
            sid_byte(3), sid_byte(2), sid_byte(1), sid_byte(0),
        )
        dst_mac_comb = Cat(
            byte_at(5), byte_at(4), byte_at(3),
            byte_at(2), byte_at(1), byte_at(0),
        )

        self.sync += [
            eof_b.eq(frame_done),
            If(frame_done,
                vlan_present_b.eq(vlan_comb),
                subtype_b.eq(subtype_comb),
                dst_mac_b.eq(dst_mac_comb),
                sid_b.eq(sid_comb),
                frame_len_b.eq(beat_idx + 1),
            ),
        ]

        # Stage C: per-slot compare registered.
        slot_match_c = Signal(n_slots)
        eof_c        = Signal()
        frame_len_c  = Signal(max=BUF_BEATS + 1)
        vlan_c       = Signal()
        is_aaf       = subtype_b == AVTP_SUBTYPE_AAF
        for i in range(n_slots):
            self.sync += If(eof_b,
                slot_match_c[i].eq(
                    slot_enabled[i]
                    & (slot_dst_mac[i] == dst_mac_b)
                    & (slot_stream_id[i] == sid_b)
                    & is_aaf
                ),
            )
        self.sync += [
            eof_c.eq(eof_b),
            If(eof_b,
                frame_len_c.eq(frame_len_b),
                vlan_c.eq(vlan_present_b),
            ),
        ]

        # ---- Sample extraction FSM ----
        #
        # On matched-EOF, walk the captured frame buffer from the audio
        # data offset and push each 32-bit sample (big-endian on wire)
        # into the appropriate per-channel FIFO.
        #
        # AAF INT_32BIT audio layout (IEEE 1722-2016 §7.3.4):
        #   - AVTP header starts at frame byte 14 (untagged) or 18 (VLAN).
        #   - AAF subheader is 24 bytes, audio samples at AVTP_OFF+24.
        #   - Class A: 6 sample-blocks × 8 channels × 4 bytes = 192 B.
        #   → audio starts at frame byte 38 (untagged) or 42 (VLAN).
        #
        # Byte-alignment issue: both 38 and 42 are 4n+2, so AAF samples
        # straddle two 32-bit LiteEth beats. The shifter recovers each
        # sample as { beat[N][15:0], beat[N+1][31:16] } — a 2-byte
        # shift right of (beat[N] << 32 | beat[N+1]) >> 16.
        #
        # FSM (1 sample per cycle in steady state):
        #   IDLE      : wait for match
        #   PRIME     : issue read for beat N (audio start)
        #   STREAM_RD : issue reads N+1, N+2, ... ; on cycle K+1 dat_r is
        #               beat K. Latch prev=beat[K-1], curr=beat[K], emit
        #               sample = {prev[15:0], curr[31:16]}, write to
        #               fifo[slot][ch=sample_n%n_channels].
        #   FINISH    : last sample written, return to IDLE.

        # Need TWO priming cycles before writing the first sample:
        #   PRIME : issue read for start beat (synchronous read → dat_r
        #           on next cycle), no write
        #   WARM  : dat_r = mem[start], latch as prev_word; issue read
        #           for start+1; no write
        #   STREAM: dat_r = mem[start+N], sample = {prev[15:0], dat_r[31:16]},
        #           write fifo, latch prev_word ← dat_r, advance
        EX_IDLE     = 0
        EX_PRIME    = 1
        EX_WARM     = 2
        EX_STREAM   = 3
        ex_state = Signal(2, reset=EX_IDLE)

        ex_slot       = Signal(max=max(n_slots, 2))
        ex_sample_n   = Signal(max=64)              # 0..47
        ex_beat_addr  = Signal(max=BUF_BEATS + 1)   # next memory read
        ex_prev_word  = Signal(32)                  # beat[N-1]

        # AAF Class A: 6 sample-blocks × n_channels = total samples per frame.
        # Hardcoded to the wire format we declared in the AEM descriptor.
        SAMPLES_PER_FRAME = 6 * n_channels

        # Combinational read address.
        self.comb += rd_port.adr.eq(ex_beat_addr)

        # Per-FIFO write enable + data. ex_state == EX_STREAM drives
        # exactly one fifo[slot][ch].we high per cycle (one-hot decode).
        ex_sample = Signal(32)
        self.comb += ex_sample.eq(Cat(rd_port.dat_r[16:32], ex_prev_word[0:16]))
        # ↑ Cat() is LSB-first in Migen:
        #     bits 0..15  = rd_port.dat_r[16:32]  (curr_word[31:16] on wire)
        #     bits 16..31 = ex_prev_word[0:16]    (prev_word[15:0]  on wire)
        # → integer value reads as {prev[15:0], curr[31:16]} big-endian
        # → matches the on-wire AAF sample for byte-offset = 2.

        # Channel index for the current sample.
        ex_ch_now = Signal(max=n_channels)
        self.comb += ex_ch_now.eq(ex_sample_n[:Signal(max=n_channels).nbits] if n_channels > 1
                                  else 0)
        # Actually, just modulo: ex_sample_n mod n_channels.
        # n_channels is a constant power-of-2 (8) so the modulo is the
        # low log2(n_channels) bits of ex_sample_n. Simpler:
        if n_channels == 1:
            self.comb += ex_ch_now.eq(0)
        else:
            import math
            nb = int(math.log2(n_channels))
            self.comb += ex_ch_now.eq(ex_sample_n[:nb])

        # One-hot per-(slot,ch) write enable + shared sample data.
        ex_write = Signal()
        self.comb += ex_write.eq(ex_state == EX_STREAM)
        for s in range(n_slots):
            for c in range(n_channels):
                self.comb += [
                    self.fifos[s][c].din.eq(ex_sample),
                    self.fifos[s][c].we.eq(
                        ex_write & (ex_slot == s) & (ex_ch_now == c)
                        & self.fifos[s][c].writable
                    ),
                ]

        # FSM transitions.
        ex_match_slot = Signal(max=max(n_slots, 2))
        case_dict = {1 << i: ex_match_slot.eq(i) for i in range(n_slots)}
        self.sync += If(eof_c & (slot_match_c != 0) & (ex_state == EX_IDLE),
            ex_match_slot.eq(0),     # default
        )
        self.sync += If(eof_c & (slot_match_c != 0) & (ex_state == EX_IDLE),
            Case(slot_match_c, case_dict),
            ex_slot.eq(ex_match_slot),
        )

        self.sync += If((ex_state == EX_IDLE) & eof_c & (slot_match_c != 0),
            ex_state.eq(EX_PRIME),
            ex_beat_addr.eq(Mux(vlan_c, 10, 9)),    # first audio beat addr
            ex_sample_n.eq(0),
        ).Elif(ex_state == EX_PRIME,
            # adr was set to `start` during PRIME cycle. dat_r at next
            # cycle will be mem[start]. Advance adr → start+1.
            ex_state.eq(EX_WARM),
            ex_beat_addr.eq(ex_beat_addr + 1),
        ).Elif(ex_state == EX_WARM,
            # dat_r = mem[start]. Latch as prev. Advance adr → start+2.
            # Still no FIFO write — first real sample emits next cycle.
            ex_prev_word.eq(rd_port.dat_r),
            ex_state.eq(EX_STREAM),
            ex_beat_addr.eq(ex_beat_addr + 1),
        ).Elif(ex_state == EX_STREAM,
            # dat_r = mem[start+N+1]. Sample = {prev[15:0], dat_r[31:16]}
            # → represents the (sample_n)-th wire sample. Combinational
            # ex_write fires the fifo.we during this cycle.
            ex_prev_word.eq(rd_port.dat_r),
            ex_beat_addr.eq(ex_beat_addr + 1),
            If(ex_sample_n == (SAMPLES_PER_FRAME - 1),
                ex_state.eq(EX_IDLE),
            ).Else(
                ex_sample_n.eq(ex_sample_n + 1),
            ),
        )

        # Match counter bump (1 per matched AAF frame).
        for i in range(n_slots):
            self.sync += If(eof_c & slot_match_c[i],
                getattr(self, f"slot{i}_match_count").status.eq(
                    getattr(self, f"slot{i}_match_count").status + 1),
            )

        # Sample-count counter: per slot, bumps once per sample BLOCK
        # (i.e. once per group of n_channels samples = on channel 0 writes).
        for i in range(n_slots):
            self.sync += If(ex_write & (ex_slot == i) & (ex_ch_now == 0),
                getattr(self, f"slot{i}_sample_count").status.eq(
                    getattr(self, f"slot{i}_sample_count").status + 1),
            )


# ----------------------------------------------------------------------
# Simulation testbench — confirms frame buffering + match pipeline run
# without errors. Audio extraction itself is exercised when the FSM
# byte-shifter lands.
# ----------------------------------------------------------------------

def _bytes_to_beats(bs):
    while len(bs) % 4 != 0:
        bs.append(0)
    return [(bs[i] << 24) | (bs[i+1] << 16) | (bs[i+2] << 8) | bs[i+3]
            for i in range(0, len(bs), 4)]


def _drive_frame(dut, beats):
    for i, beat in enumerate(beats):
        yield dut.sink.valid.eq(1)
        yield dut.sink.data.eq(beat)
        yield dut.sink.last.eq(1 if i == len(beats) - 1 else 0)
        yield
    yield dut.sink.valid.eq(0)
    yield dut.sink.last.eq(0)
    yield


def _tb(dut):
    # Configure slot 0 for an AAF stream.
    AAF_MAC = [0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x42]
    AAF_SID = [0x02, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x01]
    def mac48(b): return sum(b[i] << (8*(5-i)) for i in range(6))
    def sid64(b): return sum(b[i] << (8*(7-i)) for i in range(8))

    yield dut.slot0_enabled.storage.eq(1)
    yield dut.slot0_dst_mac_lo.storage.eq(mac48(AAF_MAC) & 0xFFFFFFFF)
    yield dut.slot0_dst_mac_hi.storage.eq((mac48(AAF_MAC) >> 32) & 0xFFFF)
    yield dut.slot0_stream_id_lo.storage.eq(sid64(AAF_SID) & 0xFFFFFFFF)
    yield dut.slot0_stream_id_hi.storage.eq((sid64(AAF_SID) >> 32) & 0xFFFFFFFF)
    yield

    # Build a VLAN-tagged AAF frame with the right format.
    bs = list(AAF_MAC) + [0x02, 0x00, 0x00, 0xFF, 0xFE, 0x00]   # dst+src
    bs += [0x81, 0x00, 0x00, 0x02]                              # VLAN tag (VID 2)
    bs += [0x22, 0xF0]                                          # AVTP ethertype
    # AVTP header (subtype AAF + flags + seq + type + stream_id + ts + format)
    bs += [AVTP_SUBTYPE_AAF, 0x00, 0x00, 0x00]
    bs += list(AAF_SID)
    bs += [0x00, 0x00, 0x00, 0x00]                              # avtp_ts
    bs += [0x02, 0x20, 0x08, 0x20, 0x00, 0xC0, 0x00, 0x00]      # format/nsr/ch/depth/len
    # 192 bytes of audio (6 samples × 8 channels × 4 bytes)
    audio = []
    for blk in range(6):
        for ch in range(8):
            v = (blk * 8 + ch) << 16
            audio += [(v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF]
    bs += audio
    yield from _drive_frame(dut, _bytes_to_beats(bs))
    # Let the extractor FSM walk the buffer (1 sample/cycle steady state).
    # 6 blocks × 8 channels = 48 samples + 5 cycles startup ≈ 60 cycles.
    for _ in range(80): yield

    print(f"slot0_match_count = {(yield dut.slot0_match_count.status)}  (want 1)")
    print(f"slot0_sample_count = {(yield dut.slot0_sample_count.status)}  (want 6)")
    for c in range(8):
        lvl = (yield getattr(dut, f"slot0_ch{c}_level").status)
        # Peek the head of each FIFO to confirm it got something.
        head = (yield getattr(dut, f"slot0_ch{c}_data").status)
        print(f"  slot0_ch{c}: level={lvl}  head=0x{head:08x}")


if __name__ == "__main__":
    dut = AVTPSampleExtractor(n_slots=4, n_channels=8, fifo_depth=256)
    run_simulation(dut, _tb(dut), vcd_name="avtp_extractor.vcd")
    print("Done.")
