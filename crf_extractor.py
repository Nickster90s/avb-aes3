# CRFTimestampExtractor — hardware Media-Clock-Recovery front end.
#
# Problem it solves: CRF (Clock Reference Format) timestamps were parsed in
# firmware off the shared 2-slot MAC RX path. On a busy AVB network that RX
# overflows (writer_errors flood) and CRF frames get dropped before the CPU
# sees them → the firmware PI servo is starved → media clock never locks
# ("CRF patched but unlocked"). gPTP survives because it's lower-contention,
# but the stream-multicast CRF loses the race.
#
# This module snoops the LiteEth MAC RX byte stream in parallel (observe-only,
# never backpressures), and for each CRF AVTPDU matching the bound stream_id
# it captures:
#     - avtp_ts   : the first 64-bit gPTP timestamp in the CRF PDU
#     - local_ts  : the TSU time latched at the frame's first beat (arrival)
# and pushes the pair into a small FIFO the CPU drains via CSRs. The PI servo
# stays in firmware (keeps the tuning flexibility), but is now fed from this
# FIFO instead of competing for a MAC RX slot — so CRF delivery no longer
# depends on CPU RX servicing under flood. The NCO is already gateware
# (MCRNco in avb_soc.py); this completes the firmware→gateware split for the
# media-clock path. Mirrors the AVTPSampleExtractor (AAF) matching style.
#
# CRF AVTPDU layout (IEEE 1722-2016 §10.2), AVTP starts at frame byte
#   14 (untagged) / 18 (VLAN); CRF header is 20 bytes; timestamps follow:
#     +0   subtype (0x04 = CRF)
#     +4..11  stream_id (8, big-endian)
#     +12..15 pull(3)|base_frequency(29)
#     +16..17 crf_data_length
#     +18..19 timestamp_interval
#     +20..27 first timestamp (8, big-endian gPTP ns)

from migen import *
from migen.genlib.fifo import SyncFIFO
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *


AVTP_SUBTYPE_CRF = 0x04


class CRFTimestampExtractor(Module, AutoCSR):
    """Observe-mode CRF (avtp_ts, local_ts) extractor feeding a CSR FIFO.

    fifo_depth — number of (avtp_ts, local_ts) pairs buffered for the CPU.
                 CRF is ~100 Hz, the servo drains every main-loop pass, so a
                 shallow FIFO is plenty; 16 covers a burst / slow loop pass.
    """

    HDR_BEATS = 16   # 64 bytes — covers VLAN CRF first-timestamp (byte 45)

    def __init__(self, fifo_depth=16, dw=32):
        assert dw == 32, "Only 32-bit input stream supported"

        # ---- Observe-only input stream (from mac.core.source) ----
        self.sink = stream.Endpoint([("data", dw), ("last_be", dw // 8)])

        # ---- TSU time input (wire to tsu.timestamp: Cat(ns[32], sec[48])) ----
        self.tsu_ts = Signal(80)

        # ---- Config CSRs ----
        self.enabled = CSRStorage(1, description="1 = extract CRF for the configured stream_id.")
        self.stream_id_hi = CSRStorage(32, description="Bound CRF stream_id[63:32].")
        self.stream_id_lo = CSRStorage(32, description="Bound CRF stream_id[31:0].")

        # ---- FIFO read CSRs (drain interface) ----
        # Read avtp_hi/lo + local_sec/ns (the current head), then strobe pop.
        self.level    = CSRStatus(max(1, (fifo_depth).bit_length()),
            description="CRF timestamp FIFO occupancy.")
        self.pop      = CSRStorage(1, description="Write 1 to pop one (avtp_ts, local_ts) pair.")
        self.avtp_hi  = CSRStatus(32, description="Head avtp_ts[63:32].")
        self.avtp_lo  = CSRStatus(32, description="Head avtp_ts[31:0].")
        self.local_sec= CSRStatus(32, description="Head local_ts seconds[31:0].")
        self.local_ns = CSRStatus(32, description="Head local_ts nanoseconds.")

        # ---- Diagnostics ----
        self.match_count   = CSRStatus(32, description="Matched CRF PDUs extracted.")
        self.overflow_count= CSRStatus(32, description="CRF pairs dropped (FIFO full).")
        self.eof_count     = CSRStatus(32, description="Total frames observed at EOF.")
        self.diag_last_sid_hi = CSRStatus(32, description="Last EOF stream_id[63:32].")
        self.diag_last_sid_lo = CSRStatus(32, description="Last EOF stream_id[31:0].")
        self.diag_last_subtype= CSRStatus(8,  description="Last EOF AVTP subtype.")

        cfg_sid = Cat(self.stream_id_lo.storage, self.stream_id_hi.storage)  # [63:0]

        # ---- Frame header capture (first HDR_BEATS beats) ----
        self.comb += self.sink.ready.eq(1)              # observer never stalls MAC
        beat_accept = Signal()
        self.comb += beat_accept.eq(self.sink.valid & self.sink.ready)
        frame_done = Signal()
        self.comb += frame_done.eq(self.sink.valid & self.sink.ready & self.sink.last)

        beat_idx = Signal(max=self.HDR_BEATS + 1)
        hdr_words = Array(Signal(32) for _ in range(self.HDR_BEATS))
        self.sync += If(beat_accept,
            If(beat_idx < self.HDR_BEATS, hdr_words[beat_idx].eq(self.sink.data)),
            If(self.sink.last, beat_idx.eq(0)).Else(
                If(beat_idx < self.HDR_BEATS, beat_idx.eq(beat_idx + 1))),
        )

        # Latch TSU time at the FIRST beat of every frame = arrival time.
        local_ts_cap = Signal(80)
        self.sync += If(beat_accept & (beat_idx == 0), local_ts_cap.eq(self.tsu_ts))

        # byte_at: LiteEth mac.core.source carries frame byte 0 at the LSB of
        # the 32-bit word (shift = 8*(off%4)). Same convention as the AAF
        # extractor — see avtp_extractor.byte_at note.
        def byte_at(off):
            w = off // 4
            shift = 8 * (off % 4)
            return hdr_words[w][shift:shift + 8]

        vlan = (byte_at(12) == 0x81) & (byte_at(13) == 0x00)
        subtype = Mux(vlan, byte_at(18), byte_at(14))            # AVTP byte 0

        def sid_byte(i):   # stream_id byte i (0=MSB), AVTP+4
            return Mux(vlan, byte_at(22 + i), byte_at(18 + i))
        sid = Cat(*[sid_byte(7 - i) for i in range(8)])          # big-endian → [63:0]

        def ts_byte(i):    # first CRF timestamp byte i (0=MSB), AVTP+20
            return Mux(vlan, byte_at(38 + i), byte_at(34 + i))
        avtp_ts = Cat(*[ts_byte(7 - i) for i in range(8)])       # big-endian → [63:0]

        # Match + extract one cycle after EOF, when hdr_words (incl. the last
        # beat) is fully latched. Inter-frame gap guarantees no new frame has
        # overwritten hdr_words / local_ts_cap by then.
        eof_b = Signal()
        self.sync += eof_b.eq(frame_done)

        matched = Signal()
        self.comb += matched.eq(
            eof_b & self.enabled.storage
            & (subtype == AVTP_SUBTYPE_CRF) & (sid == cfg_sid))

        # ---- FIFO: 128-bit {avtp_ts[64], local_sec[32], local_ns[32]} ----
        fifo = SyncFIFO(width=128, depth=fifo_depth)
        self.submodules += fifo
        local_sec = local_ts_cap[32:64]      # lower 32 of 48-bit seconds
        local_ns  = local_ts_cap[0:32]
        self.comb += [
            fifo.din.eq(Cat(local_ns, local_sec, avtp_ts)),
            fifo.we.eq(matched & fifo.writable),
            fifo.re.eq(self.pop.re),
            self.level.status.eq(fifo.level),
            self.avtp_lo.status.eq(fifo.dout[64:96]),
            self.avtp_hi.status.eq(fifo.dout[96:128]),
            self.local_sec.status.eq(fifo.dout[32:64]),
            self.local_ns.status.eq(fifo.dout[0:32]),
        ]

        # Counters / diagnostics.
        self.sync += [
            If(frame_done, self.eof_count.status.eq(self.eof_count.status + 1)),
            If(matched & fifo.writable,
               self.match_count.status.eq(self.match_count.status + 1)),
            If(matched & ~fifo.writable,
               self.overflow_count.status.eq(self.overflow_count.status + 1)),
            If(eof_b,
               self.diag_last_sid_hi.status.eq(sid[32:64]),
               self.diag_last_sid_lo.status.eq(sid[0:32]),
               self.diag_last_subtype.status.eq(subtype)),
        ]


# ---------------------------------------------------------------------------
# Sim: drive a VLAN-tagged CRF frame and confirm one (avtp_ts, local) pair.
# ---------------------------------------------------------------------------
def _bytes_to_beats(bs):
    while len(bs) % 4: bs.append(0)
    return [bs[i] | (bs[i+1] << 8) | (bs[i+2] << 16) | (bs[i+3] << 24)
            for i in range(0, len(bs), 4)]


def _tb(dut):
    SID = [0x00, 0x0e, 0x55, 0x09, 0x06, 0xe9, 0x00, 0x04]
    def sid64(b): return sum(b[i] << (8*(7-i)) for i in range(8))
    yield dut.enabled.storage.eq(1)
    yield dut.stream_id_lo.storage.eq(sid64(SID) & 0xFFFFFFFF)
    yield dut.stream_id_hi.storage.eq((sid64(SID) >> 32) & 0xFFFFFFFF)
    yield dut.tsu_ts.eq((123 << 32) | 456789)   # sec=123, ns=456789
    yield

    AVTP_TS = 0x0000_0064_1122_3344
    bs  = [0x91,0xe0,0xf0,0x00,0x7a,0x3f, 0x00,0x0e,0x55,0x09,0x06,0xe9]  # dst+src
    bs += [0x81,0x00,0x00,0x02, 0x22,0xf0]                                # VLAN + AVTP et
    bs += [AVTP_SUBTYPE_CRF, 0x00, 0x00, 0x00]                            # CRF subtype+seq+type
    bs += SID                                                            # stream_id
    bs += [0x00,0x00,0xbb,0x80]                                          # pull|base_freq
    bs += [0x00,0x08, 0x00,0x01]                                        # crf_data_len=8, interval=1
    bs += [(AVTP_TS >> (8*(7-i))) & 0xFF for i in range(8)]             # first timestamp
    for i, beat in enumerate(_bytes_to_beats(bs)):
        yield dut.sink.valid.eq(1); yield dut.sink.data.eq(beat)
        yield dut.sink.last.eq(1 if i == len(_bytes_to_beats(bs)) - 1 else 0)
        yield
    yield dut.sink.valid.eq(0); yield dut.sink.last.eq(0)
    for _ in range(4): yield

    lvl = (yield dut.level.status)
    mc  = (yield dut.match_count.status)
    ahi = (yield dut.avtp_hi.status); alo = (yield dut.avtp_lo.status)
    sec = (yield dut.local_sec.status); ns = (yield dut.local_ns.status)
    print(f"level={lvl} (want 1)  match_count={mc} (want 1)")
    print(f"avtp_ts=0x{ahi:08x}{alo:08x} (want 0x{AVTP_TS:016x})")
    print(f"local sec={sec} (want 123)  ns={ns} (want 456789)")


if __name__ == "__main__":
    dut = CRFTimestampExtractor()
    run_simulation(dut, _tb(dut), vcd_name="crf_extractor.vcd")
    print("Done.")
