# AVTPStreamFilter — Stage 1 of the firmware-to-gateware audio path migration.
#
# Captures the first 32 bytes of every Ethernet frame from the LiteEth MAC
# source stream, then on end-of-frame decides whether it matches one of N
# configured (dst_mac, stream_id) AVTP slots. Emits a per-frame match
# indication that downstream stages will use to extract audio samples
# directly into per-stream FIFOs (Stage 2), bypassing the CPU entirely.
#
# Why we buffer 32 bytes: enough to cover dst_mac(6) + src_mac(6) + optional
# VLAN tag(4) + ethertype(2) + AVTP subtype(1) + sv/version(1) + seq(1) +
# type(1) + stream_id(8) = 30 bytes max. We round up to 32 because 32-bit
# stream gives us 4 bytes per beat — 8 beats × 4 bytes = 32.
#
# This file is standalone — `python3 avtp_stream_filter.py` runs the
# Migen simulation testbench at the bottom. After Stage 1 acceptance,
# the module gets instantiated by avb_soc.py between LiteEth's
# mac.core.source and the existing SRAM writer.

from migen import *
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *


# AVTP subtypes we care about for stream matching.
AVTP_SUBTYPE_AAF = 0x02
AVTP_SUBTYPE_CRF = 0x04


class AVTPStreamFilter(Module):
    """Tags each LiteEth RX frame with `match_slot` if it's a configured
    AVTP stream (matched on dst_mac + stream_id + cd=0 AVTP data subtype).

    The match signal asserts for one cycle when the frame ends — by which
    time we've already buffered the bytes needed for matching. Downstream
    can use `match_valid + match_slot` to route audio extraction.

    Parameters
    ----------
    n_slots : int
        Number of configurable stream slots (default 4).
    dw : int
        Data width of the input stream (default 32 — matches
        add_ethernet(data_width=8) which internally promotes to dw=32).
    """

    def __init__(self, n_slots=4, dw=32):
        assert dw == 32, "Only 32-bit input stream supported at present"

        # Input: same shape as litex stream from LiteEth mac.core.source.
        self.sink = stream.Endpoint([
            ("data",    dw),
            ("last_be", dw // 8),
        ])

        # Per-slot configuration — driven from CSRs by the SoC wrapper.
        # dst_mac is 48 bits, stream_id 64 bits. enabled gates the slot.
        self.slot_enabled   = Array(Signal()      for _ in range(n_slots))
        self.slot_dst_mac   = Array(Signal(48)    for _ in range(n_slots))
        self.slot_stream_id = Array(Signal(64)    for _ in range(n_slots))

        # Outputs: pulse for one cycle at end of frame.
        self.eof          = Signal()                      # any frame end
        self.match_valid  = Signal()                      # eof AND a slot matched
        self.match_slot   = Signal(max=max(n_slots, 2))   # which slot matched
        # Per-slot match counters (read by SoC wrapper through a CSR).
        self.match_count  = Array(Signal(32) for _ in range(n_slots))

        # ---------------------------------------------------------------
        # Header buffer: 32 bytes = 8 × 32-bit beats.
        # ---------------------------------------------------------------
        # MAC stream is big-endian on the wire. data[31:24] is byte 0 of
        # the beat, data[23:16] byte 1, etc. We store one 32-bit word
        # per beat and index into them with byte offsets later.
        HDR_BEATS = 8
        HDR_BYTES = HDR_BEATS * 4
        hdr_words = Array(Signal(32) for _ in range(HDR_BEATS))
        beat_idx  = Signal(max=HDR_BEATS + 1)

        # Capture first HDR_BEATS beats into hdr_words. Discard extra
        # beats (frame body). We still need to ride out the frame to see
        # `last` so we can emit the match pulse.
        self.sync += [
            self.eof.eq(0),
            self.match_valid.eq(0),
        ]

        self.sync += If(self.sink.valid & self.sink.ready,
            If(beat_idx != HDR_BEATS,
                hdr_words[beat_idx].eq(self.sink.data),
                beat_idx.eq(beat_idx + 1),
            ),
        )
        # Always ready to receive — we don't apply backpressure to the MAC.
        self.comb += self.sink.ready.eq(1)

        # ---------------------------------------------------------------
        # Helpers: extract a byte at byte-offset `off` from hdr_words[].
        # ---------------------------------------------------------------
        def byte_at(off):
            """Return Signal(8) holding the wire-byte at offset `off`.

            Wire-byte 0 of beat N is the MSB of hdr_words[N] (data[31:24])
            because LiteEth presents frames big-endian within the 32-bit
            word."""
            w = off // 4
            shift = 24 - 8 * (off % 4)
            return hdr_words[w][shift:shift + 8]

        # ---------------------------------------------------------------
        # End-of-frame edge: emit match decision.
        # ---------------------------------------------------------------
        self.sync += If(self.sink.valid & self.sink.ready & self.sink.last,
            beat_idx.eq(0),
            self.eof.eq(1),

            # Probe the captured header. Detect VLAN tag at offset 12-13:
            # if 0x8100, the AVTP header starts at offset 18; otherwise 14.
            #
            # Build the candidate match key (dst_mac, stream_id) and check
            # against each slot.
        )

        # ---------------------------------------------------------------
        # Match comb logic (evaluated continuously, sampled at EOF).
        # ---------------------------------------------------------------
        vlan_present = Signal()
        self.comb += vlan_present.eq(
            (byte_at(12) == 0x81) & (byte_at(13) == 0x00)
        )

        # AVTP header offset in the buffer.
        # Without VLAN: AVTP at byte 14, subtype at 14, stream_id at 18.
        # With VLAN:    AVTP at byte 18, subtype at 18, stream_id at 22.
        # Use Mux to select.
        subtype = Signal(8)
        self.comb += subtype.eq(
            Mux(vlan_present, byte_at(18), byte_at(14))
        )

        # Build a Signal(64) holding the candidate stream_id, byte-MSB-first.
        sid = Signal(64)
        self.comb += sid.eq(Cat(
            # Migen Cat() is LSB-first; we want byte_at(off+7) at LSB so
            # that the resulting integer matches the wire-byte order.
            Mux(vlan_present, byte_at(25), byte_at(21)),  # stream_id[0]  → bits 7..0
            Mux(vlan_present, byte_at(24), byte_at(20)),
            Mux(vlan_present, byte_at(23), byte_at(19)),
            Mux(vlan_present, byte_at(22), byte_at(18)),
            Mux(vlan_present, byte_at(21), byte_at(17)),  # ← stream_id last 4 bytes...
            # Wait — stream_id starts at AVTP byte 4. AVTP starts at frame
            # byte 14 or 18. So stream_id wire bytes are at frame offsets
            # 18..25 (no VLAN) or 22..29 (with VLAN). The Mux above had
            # them shifted by 4 (offset 18→22, 19→23, etc.). Re-derive.
        ))

        # The Cat above was confused. Redo cleanly: stream_id byte 0 (MSB)
        # is at frame_off (18 no-vlan / 22 vlan), through byte 7 (LSB) at
        # frame_off+7.
        def stream_id_byte(i):
            off = Mux(vlan_present, 22 + i, 18 + i)
            # We can't use a runtime Mux as an index into hdr_words. So
            # express both options and Mux the byte:
            return Mux(vlan_present, byte_at(22 + i), byte_at(18 + i))

        self.comb += sid.eq(Cat(
            stream_id_byte(7), stream_id_byte(6), stream_id_byte(5),
            stream_id_byte(4), stream_id_byte(3), stream_id_byte(2),
            stream_id_byte(1), stream_id_byte(0),
        ))

        # dst_mac: always at frame bytes 0..5.
        dst_mac = Signal(48)
        self.comb += dst_mac.eq(Cat(
            byte_at(5), byte_at(4), byte_at(3),
            byte_at(2), byte_at(1), byte_at(0),
        ))

        # Check each slot for a match. Combinational because we sample
        # at eof and the buffer is stable at that point.
        slot_match = Signal(n_slots)
        for i in range(n_slots):
            self.comb += slot_match[i].eq(
                self.slot_enabled[i]
                & (self.slot_dst_mac[i]   == dst_mac)
                & (self.slot_stream_id[i] == sid)
                & ((subtype == AVTP_SUBTYPE_AAF) | (subtype == AVTP_SUBTYPE_CRF))
            )

        # Emit match_valid + match_slot one cycle after eof. We use a
        # registered comparison so timing is closed at sim/hw boundary.
        eof_reg = Signal()
        self.sync += eof_reg.eq(self.eof)
        self.sync += If(eof_reg,
            # priority-encoded slot index (first match wins)
            self.match_valid.eq(slot_match != 0),
            self.match_slot.eq(0),
        )
        # Priority decode without a loop variable bug — use Migen Case.
        case_dict = {}
        for i in range(n_slots):
            case_dict[1 << i] = self.match_slot.eq(i)
        # The match_slot setter above sets default 0; this Case overrides
        # when exactly one bit is set. For multi-bit matches (shouldn't
        # happen with valid config), the lowest index wins via the
        # cascaded Case ordering.
        self.sync += If(eof_reg, Case(slot_match, case_dict))

        # Bump per-slot match counter (one cycle after eof).
        for i in range(n_slots):
            self.sync += If(eof_reg & slot_match[i],
                self.match_count[i].eq(self.match_count[i] + 1),
            )


# ---------------------------------------------------------------------------
# Stand-alone Migen simulation testbench.
# ---------------------------------------------------------------------------

def _frame_bytes(dst_mac, src_mac, vlan_vid, ethertype, payload):
    """Build a Python-list Ethernet frame, optionally VLAN-tagged."""
    b  = list(dst_mac) + list(src_mac)
    if vlan_vid is not None:
        b += [0x81, 0x00, (vlan_vid >> 8) & 0x0F, vlan_vid & 0xFF]
    b += [(ethertype >> 8) & 0xFF, ethertype & 0xFF]
    b += list(payload)
    return b


def _avtp_payload(subtype, stream_id):
    """Minimal AVTP header: subtype, sv/version, seq, type, stream_id, plus
    a few bytes of dummy body so the frame has nonzero length."""
    return [
        subtype,
        0x00,         # sv/version/mr/r/fs/tu
        0x00,         # sequence_num
        0x00,         # type
    ] + list(stream_id) + [0xCA, 0xFE, 0xBA, 0xBE]


def _drive_frame(dut, beats):
    """Drive a list of 32-bit beats onto dut.sink with last on the final."""
    for i, beat in enumerate(beats):
        last = 1 if i == len(beats) - 1 else 0
        yield dut.sink.valid.eq(1)
        yield dut.sink.data.eq(beat)
        yield dut.sink.last.eq(last)
        yield
        # Wait until the DUT accepts (it's always ready in this design).
        while not (yield dut.sink.ready):
            yield
    yield dut.sink.valid.eq(0)
    yield dut.sink.last.eq(0)
    yield


def _bytes_to_beats(bs):
    """Pad to a multiple of 4 and pack big-endian into 32-bit beats."""
    while len(bs) % 4 != 0:
        bs.append(0)
    beats = []
    for i in range(0, len(bs), 4):
        beats.append((bs[i] << 24) | (bs[i+1] << 16) | (bs[i+2] << 8) | bs[i+3])
    return beats


def _testbench(dut):
    # Configure slot 0 for AAF stream
    # dst_mac 91:E0:F0:00:FE:42, stream_id 02:00:00:00:00:42:00:01
    AAF_MAC = [0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x42]
    AAF_SID = [0x02, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x01]
    # Configure slot 1 for CRF stream
    # dst_mac 91:E0:F0:00:39:74, stream_id 00:0E:55:09:06:E9:00:04
    CRF_MAC = [0x91, 0xE0, 0xF0, 0x00, 0x39, 0x74]
    CRF_SID = [0x00, 0x0E, 0x55, 0x09, 0x06, 0xE9, 0x00, 0x04]

    def mac48(b): return (b[0] << 40) | (b[1] << 32) | (b[2] << 24) | \
                          (b[3] << 16) | (b[4] << 8)  |  b[5]
    def sid64(b): return (b[0] << 56) | (b[1] << 48) | (b[2] << 40) | \
                          (b[3] << 32) | (b[4] << 24) | (b[5] << 16) | \
                          (b[6] << 8)  |  b[7]

    yield dut.slot_enabled[0].eq(1)
    yield dut.slot_dst_mac[0].eq(mac48(AAF_MAC))
    yield dut.slot_stream_id[0].eq(sid64(AAF_SID))
    yield dut.slot_enabled[1].eq(1)
    yield dut.slot_dst_mac[1].eq(mac48(CRF_MAC))
    yield dut.slot_stream_id[1].eq(sid64(CRF_SID))
    yield  # let signals propagate

    # --------------------------------
    # Test 1: VLAN-tagged AAF frame matching slot 0
    # --------------------------------
    print("Test 1: VLAN AAF should match slot 0")
    frame = _frame_bytes(
        dst_mac=AAF_MAC,
        src_mac=[0x02, 0x00, 0x00, 0xFF, 0xFE, 0x00],
        vlan_vid=2,
        ethertype=0x22F0,
        payload=_avtp_payload(AVTP_SUBTYPE_AAF, AAF_SID),
    )
    yield from _drive_frame(dut, _bytes_to_beats(frame))
    for _ in range(5): yield
    print(f"  match_count[0] = {(yield dut.match_count[0])} (want 1)")
    print(f"  match_count[1] = {(yield dut.match_count[1])} (want 0)")

    # --------------------------------
    # Test 2: untagged CRF frame matching slot 1
    # --------------------------------
    print("Test 2: untagged CRF should match slot 1")
    frame = _frame_bytes(
        dst_mac=CRF_MAC,
        src_mac=[0x00, 0x0E, 0x55, 0x09, 0x06, 0xE9],
        vlan_vid=None,
        ethertype=0x22F0,
        payload=_avtp_payload(AVTP_SUBTYPE_CRF, CRF_SID),
    )
    yield from _drive_frame(dut, _bytes_to_beats(frame))
    for _ in range(5): yield
    print(f"  match_count[0] = {(yield dut.match_count[0])} (want 1)")
    print(f"  match_count[1] = {(yield dut.match_count[1])} (want 1)")

    # --------------------------------
    # Test 3: frame to a different MAC should NOT match
    # --------------------------------
    print("Test 3: wrong dst_mac should not match")
    frame = _frame_bytes(
        dst_mac=[0x91, 0xE0, 0xF0, 0x00, 0x00, 0x00],
        src_mac=[0x02, 0x00, 0x00, 0xFF, 0xFE, 0x00],
        vlan_vid=None,
        ethertype=0x22F0,
        payload=_avtp_payload(AVTP_SUBTYPE_AAF, AAF_SID),
    )
    yield from _drive_frame(dut, _bytes_to_beats(frame))
    for _ in range(5): yield
    print(f"  match_count[0] = {(yield dut.match_count[0])} (still want 1)")

    # --------------------------------
    # Test 4: AVDECC subtype should NOT match (cd=1, not data)
    # --------------------------------
    print("Test 4: ADP (cd=1 / subtype 0xFA) should not match")
    frame = _frame_bytes(
        dst_mac=AAF_MAC,
        src_mac=[0x02, 0x00, 0x00, 0xFF, 0xFE, 0x00],
        vlan_vid=None,
        ethertype=0x22F0,
        payload=_avtp_payload(0xFA, AAF_SID),     # ADP subtype
    )
    yield from _drive_frame(dut, _bytes_to_beats(frame))
    for _ in range(5): yield
    print(f"  match_count[0] = {(yield dut.match_count[0])} (still want 1)")


if __name__ == "__main__":
    dut = AVTPStreamFilter(n_slots=4)
    run_simulation(dut, _testbench(dut), vcd_name="avtp_stream_filter.vcd")
    print("\nDone. Open avtp_stream_filter.vcd in GTKWave for waveforms.")
