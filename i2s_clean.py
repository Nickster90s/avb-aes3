#!/usr/bin/env python3
# Clean I2S transmitter for the local PCM5102A monitor (v4 — clean cd_audio clock).
#
# Root cause of "the DAC doesn't accept what we send": our board has NO MCLK pin
# (only BCK/LRCK/DIN), so the PCM5102A runs in no-MCLK mode and derives its
# system clock from BCK with an internal PLL — which needs a CLEAN, regular BCK.
# Earlier versions derived BCK from the NCO phase accumulator (±1 sys-cycle edge
# jitter + the DAC mis-counts BCK-per-LRCK near the jittery transition) → the DAC
# mis-frames → a left-only signal leaks onto both channels (bleed) / mono.
#
# v4 (exactly the ADAT approach): run the whole serializer in cd_audio (12.288 MHz
# = 256*48 kHz, a clean PLL output). BCK = audio/4 = 3.072 MHz, LRCK = audio/256 =
# 48 kHz, derived as pure integer divisions of cd_audio → ALWAYS exactly 32 BCK /
# channel, zero edge jitter. Samples cross sys→audio through an AsyncFIFO; the
# tiny NCO-vs-cd_audio rate difference is absorbed by repeating the last frame on
# underrun (inaudible for a monitor). Instantiate under DomainRenamer("audio").
#
# I2S Philips, MSB first, 1-bit delay, LRCK low = left. 32-BCK slot per channel:
#   bit0 = delay 0 ; bits 1..24 = data MSB..LSB ; bits 25..31 = pad 0
from migen import *
from litex.gen import LiteXModule
from litex.soc.interconnect.csr import CSRStatus


class CleanI2STx(LiteXModule):
    def __init__(self):
        self.bck   = Signal()
        self.lrck  = Signal()
        self.dout  = Signal()
        self.audio_l     = Signal(24)
        self.audio_r     = Signal(24)
        self.audio_valid = Signal()
        self.audio_ready = Signal()

        self._underruns = CSRStatus(32, description="Frames repeated because the I2S FIFO was empty.")
        underruns = Signal(32)
        self.comb += self._underruns.status.eq(underruns)

        # 256 cd_audio cycles per stereo frame.
        cnt = Signal(8)
        self.sync += cnt.eq(cnt + 1)
        self.comb += [
            self.bck .eq(cnt[1]),   # audio/4  = 3.072 MHz
            self.lrck.eq(cnt[7]),   # audio/256 = 48 kHz, low = left
        ]

        shift_l = Signal(24)
        shift_r = Signal(24)
        sr      = Signal(32)

        def word(d):
            # sr[31] sent first: [31]=delay0, [30:7]=data MSB-first, [6:0]=pad0
            return Cat(C(0, 7), d, C(0, 1))

        # Latch the next frame's L+R at a stable point near the end of the R slot
        # (after R's data, before the next L), and pop the FIFO there — so both
        # channels of a frame come from the SAME FIFO entry (no inter-channel skew).
        # On underrun, keep the previous shift_l/shift_r → repeat last frame.
        prep = Signal()
        self.comb += prep.eq(cnt == 250)
        self.comb += self.audio_ready.eq(prep & self.audio_valid)
        self.sync += If(prep,
            If(self.audio_valid,
                shift_l.eq(self.audio_l),
                shift_r.eq(self.audio_r),
            ).Else(
                underruns.eq(underruns + 1),
            )
        )

        bck_fall = Signal()
        self.comb += bck_fall.eq(cnt[0:2] == 0)   # cnt % 4 == 0
        self.sync += If(cnt == 0,                  # LEFT slot start
            sr.eq(Cat(C(0, 1), word(shift_l)[0:31])),
            self.dout.eq(0),
        ).Elif(cnt == 128,                          # RIGHT slot start
            sr.eq(Cat(C(0, 1), word(shift_r)[0:31])),
            self.dout.eq(0),
        ).Elif(bck_fall,
            self.dout.eq(sr[31]),
            sr.eq(Cat(C(0, 1), sr[0:31])),
        )


# ---------------------------------------------------------------------------
# Self-checking sim: clock the serializer (single domain = cd_audio), drive the
# FIFO interface, sample (lrck, dout) at BCK rising as the DAC does, decode, and
# verify (a) bit-exact L/R, (b) LEFT-ONLY → R decodes to 0 (no bleed).
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    from migen.sim import run_simulation

    def run_case(name, Ls, Rs):
        dut = CleanI2STx()
        res = {}
        def tb():
            idx = 0
            yield dut.audio_l.eq(Ls[0]); yield dut.audio_r.eq(Rs[0])
            yield dut.audio_valid.eq(1)
            prev = 0; cap = []
            for t in range(256 * 12):       # ~12 frames
                yield
                if (yield dut.audio_ready):
                    idx = min(idx + 1, len(Ls) - 1)
                    yield dut.audio_l.eq(Ls[idx]); yield dut.audio_r.eq(Rs[idx])
                b = (yield dut.bck)
                if b and not prev:
                    cap.append(((yield dut.lrck), (yield dut.dout)))
                prev = b
            runs = []; cl = None; run = []
            for lrck, bit in cap:
                if lrck != cl:
                    if run: runs.append((cl, run))
                    cl = lrck; run = [bit]
                else: run.append(bit)
            if run: runs.append((cl, run))
            dec = []
            for lrck, bits in runs:
                if len(bits) < 25: continue
                v = 0
                for x in bits[1:25]: v = (v << 1) | x
                dec.append(("L" if lrck == 0 else "R", v))
            res['dec'] = dec
        run_simulation(dut, tb(), vcd_name=None)
        dec = res['dec']
        L = [v for c, v in dec if c == "L"]; R = [v for c, v in dec if c == "R"]
        print(f"[{name}] decoded {len(dec)}  L[3:6]={[hex(x) for x in L[3:6]]}  R[3:6]={[hex(x) for x in R[3:6]]}")
        return L, R

    TONE = [0x7FFFFF, 0x123456, 0xABCDEF, 0x800000, 0x555555, 0x000001]
    print("=== A: identical L=R (bit-exact, no swap) ===")
    L, R = run_case("A", TONE, TONE)
    okA = all(v in TONE for v in L[2:6]) and all(v in TONE for v in R[2:6])
    print("  both bit-exact:", okA)
    print("=== B: LEFT-ONLY (L=tone, R=0) → R MUST be 0 (bleed test) ===")
    L, R = run_case("B", TONE, [0]*len(TONE))
    bleed = [v for v in R[2:8] if v != 0]
    okB = (not bleed) and any(v != 0 for v in L[2:6])
    print(f"  R bleed samples: {[hex(x) for x in bleed]}   L has signal + R silent: {okB}")
    print("\nPASS" if okA and okB else "\nFAIL")
