#!/usr/bin/env python3

#
# AVB AES3 SoC for Colorlight i9plus v6.1
#
# LiteX SoC with:
# - VexRiscv CPU + SRAM
# - LiteEth RGMII MAC (Wishbone interface for firmware raw frame access)
# - LiteEthTSU (PTP Timestamping Unit) with CSRs for gPTP firmware
# - I2S TX → PCM5102A DAC
# - MCR NCO (firmware PI servo on CRF stream)
# - AVTPSampleExtractor (gateware-side stream matching + sample extraction, Stage 2a)
# - UART (PMOD)
#
# AES3 RX/TX gateware modules were removed 2026-05-21 — they were
# dormant (firmware path moved to I2S DAC) but still consumed enough
# slices to block AVTPSampleExtractor timing closure. Verilog files
# preserved in rtl/_aes3_backup/.
#
# Build:
#   source /home/lisp/FPGA/env.sh
#   python3 avb_soc.py --build --firmware firmware/firmware.bin
#

import os
import argparse

from migen import *
from litex.gen import *

from litex.build.io import DDROutput
from litex.build.generic_platform import Pins, IOStandard, Subsignal

from litex_boards.platforms import colorlight_i9plus

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser
from litex.soc.interconnect.csr import CSRStatus, CSRStorage, AutoCSR

from liteeth.phy.s7rgmii import LiteEthPHYRGMII
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "rtl"))
from rgmii_var_delay import LiteEthPHYRGMIIVar
from liteeth.mac import LiteEthMAC
from liteeth.core.ptp import LiteEthTSU

# Stage 1 of firmware-to-gateware audio path migration. Observes the
# LiteEth MAC RX stream and per-frame matches (dst_mac, stream_id)
# against CSR-configured slots; 3-stage pipelined for timing closure.
from avtp_extractor import AVTPSampleExtractor
from crf_extractor  import CRFTimestampExtractor

from migen.genlib.fifo import SyncFIFO, AsyncFIFO

# CRG -------------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst       = Signal()
        self.cd_sys    = ClockDomain()
        self.cd_idelay = ClockDomain()

        # Audio clock domain: 12.288 MHz (256 * 48 kHz).
        self.cd_audio  = ClockDomain()

        # IDELAY reference clock (200 MHz).
        self.cd_idelay = ClockDomain()

        # Clk/Rst.
        clk25 = platform.request("clk25")

        # Single PLL for sys_clk, IDELAY, and audio clock.
        # The standalone audio MMCM never locked under openXC7 (whether driven
        # from clk25 or sys_clk), so we add the audio clock as a third PLLE2
        # output instead. PLLE2 only takes integer dividers — with a 1% margin
        # the solver finds a VCO that yields ~12.288 MHz, which is well within
        # the PCM5102A's internal-PLL lock range.
        self.audio_pll = self.pll = pll = S7PLL(speedgrade=-1)
        pll.vco_margin = 0.1
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk25, 25e6)
        pll.create_clkout(self.cd_sys,    sys_clk_freq)
        pll.create_clkout(self.cd_idelay, 200e6)
        pll.create_clkout(self.cd_audio,  12.288e6, margin=1e-2)

        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin)
        platform.add_false_path_constraints(self.cd_sys.clk, self.cd_audio.clk)

        self.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

        # USB clock domain: 60 MHz recovered from the USB3300 ULPI CLK
        # output, re-emitted at phase 0 (the proven-good ULPI sampling
        # phase from avb-usb-host's eye-scan). Separate PLL since ulpi_clk
        # is async to clk25. VCO kept >=800 MHz by S7PLL's solver.
        self.cd_usb = ClockDomain()
        ulpi_clk = platform.request("ulpi_clock")
        self.usb_pll = usb_pll = S7PLL(speedgrade=-1)
        self.comb += usb_pll.reset.eq(self.rst)
        usb_pll.register_clkin(ulpi_clk, 60e6)
        usb_pll.create_clkout(self.cd_usb, 60e6, phase=0)
        platform.add_false_path_constraints(self.cd_sys.clk, self.cd_usb.clk)

# TSU CSR Wrapper --------------------------------------------------------------------------------------

class TSUWithCSRs(LiteXModule):
    """Wraps LiteEthTSU with CSR registers for firmware access."""
    def __init__(self, clk_freq):
        self.tsu = tsu = LiteEthTSU(clk_freq)

        # Timestamp output (80-bit: 48-bit seconds + 32-bit nanoseconds).
        self.timestamp = Signal(80)
        self.comb += self.timestamp.eq(Cat(tsu.nanoseconds, tsu.seconds))

        # --- Read-only status CSRs ---

        # Current time (read atomically: read seconds_hi first, which latches all).
        self._seconds_hi    = CSRStatus(16, description="PTP seconds [47:32] — read first to latch.")
        self._seconds_lo    = CSRStatus(32, description="PTP seconds [31:0].")
        self._nanoseconds   = CSRStatus(32, description="PTP nanoseconds.")

        # Latched seconds/nanoseconds for atomic read.
        seconds_latch    = Signal(48)
        nanoseconds_latch = Signal(32)
        latch_pulse      = Signal()

        # Latch on read of seconds_hi (active for 1 cycle on CSR read strobe).
        self.comb += latch_pulse.eq(self._seconds_hi.we)
        self.sync += If(latch_pulse,
            seconds_latch.eq(tsu.seconds),
            nanoseconds_latch.eq(tsu.nanoseconds),
        )
        self.comb += [
            self._seconds_hi.status.eq(seconds_latch[32:48]),
            self._seconds_lo.status.eq(seconds_latch[0:32]),
            self._nanoseconds.status.eq(nanoseconds_latch),
        ]

        # RX/TX captured timestamps.
        self._rx_ts_seconds  = CSRStatus(48, description="RX timestamp seconds.")
        self._rx_ts_nsec     = CSRStatus(32, description="RX timestamp nanoseconds.")
        self._tx_ts_seconds  = CSRStatus(48, description="TX timestamp seconds.")
        self._tx_ts_nsec     = CSRStatus(32, description="TX timestamp nanoseconds.")

        self.comb += [
            self._rx_ts_seconds.status.eq(tsu.rx_ts[32:80]),
            self._rx_ts_nsec.status.eq(tsu.rx_ts[0:32]),
            self._tx_ts_seconds.status.eq(tsu.tx_ts[32:80]),
            self._tx_ts_nsec.status.eq(tsu.tx_ts[0:32]),
        ]

        # --- Writable control CSRs ---

        # Addend (frequency word for tick accumulation).
        # Full 52-bit resolution: (addend << 20) | addend_frac. 1 LSB of
        # addend_frac changes ns/cycle by 1e9 / 2^52 ≈ 2.22e-7, i.e.
        # ~11.1 ns/sec rate adjustment per LSB at 50 MHz — sub-ppm tuning.
        # Routability of the 52x30 multiplier is fixed by patching
        # liteeth/core/ptp.py to use a shift-add tree instead of the `*`
        # operator (the natural form makes yosys infer a DSP48 cascade
        # that nextpnr-xilinx fails to route on XC7A50T).
        self._addend      = CSRStorage(32, description="TSU addend integer part.")
        self._addend_frac = CSRStorage(20, description="TSU addend fractional part (20-bit).")
        self.comb += [
            tsu.addend.eq(self._addend.storage),
            tsu.addend_frac.eq(self._addend_frac.storage),
        ]

        # Offset correction (signed, applied once then cleared by TSU).
        self._offset_hi = CSRStorage(17, description="Offset correction [48:32] (signed).")
        self._offset_lo = CSRStorage(32, description="Offset correction [31:0].")
        self._offset_apply = CSRStorage(1, description="Write 1 to apply offset correction.")

        # Apply offset on write strobe.
        # tsu.offset is 81-bit signed; CSRs hold 49 bits (lo 32 + hi 17).
        # The hi field's top bit (bit 16) is the sign bit at combined bit 48 —
        # replicate it into bits 49..80 so a negative firmware value (e.g.
        # -145000ns) doesn't get zero-extended into a huge positive offset.
        sign_bit = self._offset_hi.storage[16]
        self.sync += If(self._offset_apply.re,
            tsu.offset.eq(Cat(
                self._offset_lo.storage,                # bits  0..31
                self._offset_hi.storage,                # bits 32..48
                Replicate(sign_bit, 81 - 49),           # bits 49..80
            )),
        )

        # Coarse step (set time directly).
        self._step_seconds = CSRStorage(48, description="Step: target seconds.")
        self._step_nsec    = CSRStorage(32, description="Step: target nanoseconds.")
        self._step_apply   = CSRStorage(1,  description="Write 1 to step to target time.")

        self.comb += [
            tsu.step_target.eq(Cat(self._step_nsec.storage, self._step_seconds.storage)),
        ]
        self.sync += If(self._step_apply.re,
            tsu.step.eq(1),
        ).Else(
            tsu.step.eq(0),
        )

# MCR NCO -----------------------------------------------------------------------------------------------

class MCRNco(LiteXModule):
    """Numerically-controlled oscillator that emits a sample-rate strobe.

    fs = (increment * sys_clk_freq) / 2**32

    Firmware sets `increment` from a PI servo on CRF timestamps (mcr.c).
    A 33-bit add captures the carry-out as the sample strobe — that's the
    standard NCO trick: every time the 32-bit phase wraps, one sample tick
    has elapsed in fs-time.

    `sample_count` is a free-running count of strobes; firmware reads it to
    detect lock (delta_count over a known interval == expected sample count).
    """
    def __init__(self, sys_clk_freq, fs=48000):
        self.sample_strobe = Signal()

        # Default increment ≈ fs / sys_clk_freq, scaled to 32 bits.
        default_inc = int(round(fs * (1 << 32) / sys_clk_freq))

        self._increment = CSRStorage(32, reset=default_inc,
            description="NCO phase increment per sys_clk. fs = (inc*sys_clk_freq)/2^32.")
        self._sample_count = CSRStatus(32,
            description="Free-running fs sample count (debug).")
        self._phase = CSRStatus(32,
            description="Current NCO phase (debug).")

        phase        = Signal(32)
        next_phase   = Signal(33)
        sample_count = Signal(32)

        self.comb += next_phase.eq(phase + self._increment.storage)
        self.sync += [
            phase.eq(next_phase[:32]),
            self.sample_strobe.eq(next_phase[32]),
            If(self.sample_strobe,
                sample_count.eq(sample_count + 1)),
        ]
        self.comb += [
            self._sample_count.status.eq(sample_count),
            self._phase.status.eq(phase),
        ]


# I2S Pin Extension ------------------------------------------------------------------------------------

def i2s_io():
    return [
        # SODIMM pin 46 → FPGA ball U7 → PCM5102A BCK
        ("i2s_bck",  0, Pins("U7"), IOStandard("LVCMOS33")),
        # SODIMM pin 48 → FPGA ball U6 → PCM5102A LRCK / LCK
        ("i2s_lrck", 0, Pins("U6"), IOStandard("LVCMOS33")),
        # SODIMM pin 50 → FPGA ball U5 → PCM5102A DIN
        ("i2s_dout", 0, Pins("U5"), IOStandard("LVCMOS33")),
    ]

def ulpi_io():
    # USB3300 ULPI breakout on the P2 header. CLK=T4 (MRCC, clock-capable);
    # RST active-HIGH (USB3300 pin 9) so handled as plain Pins, driven low
    # by the gateware to release. Data is a bidirectional bus (TSTriple in
    # the SoC). Verified free vs eth/eth_clocks/sdram on this platform.
    # See avb-usb-host docs/phase3-bridge.md + memory ulpi-twisted-pair-wiring.
    return [
        ("ulpi_clock", 0, Pins("T4"), IOStandard("LVCMOS33")),
        ("ulpi", 0,
            Subsignal("dir",  Pins("T3"),  IOStandard("LVCMOS33")),
            Subsignal("nxt",  Pins("U2"),  IOStandard("LVCMOS33")),
            Subsignal("stp",  Pins("U3"),  IOStandard("LVCMOS33")),
            Subsignal("rst",  Pins("R2"),  IOStandard("LVCMOS33")),
            Subsignal("data", Pins("V2 V3 W1 W2 Y1 AA1 AB1 Y2"),
                      IOStandard("LVCMOS33")),
        ),
    ]

# AVB SoC ----------------------------------------------------------------------------------------------

class AVBSoC(SoCCore):
    def __init__(self, sys_clk_freq=int(50e6), **kwargs):
        platform = colorlight_i9plus.Platform(toolchain="openxc7")

        # UART via CH347 on Ext-Board (TXD1/RXD1 routed to FPGA).
        platform.add_extension([
            ("serial", 0,
                Subsignal("tx", Pins("R3")),   # FPGA TX → CH347 RXD1
                Subsignal("rx", Pins("M3")),   # FPGA RX ← CH347 TXD1
                IOStandard("LVCMOS33"))
        ])

        # USB UAC2 ULPI (P2 header) — must be added before the CRG, which
        # requests ulpi_clock for the cd_usb PLL.
        platform.add_extension(ulpi_io())

        # CRG.
        self.crg = _CRG(platform, sys_clk_freq)

        # SoCCore (VexRiscv + SRAM).
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident         = "AVB-AES3 SoC on Colorlight i9+",
            ident_version = True,
            **kwargs
        )

        # Ethernet PHY (RGMII, PHY1 / U9). Cable lands on U9 on this board —
        # confirmed by MDIO power-down test (addr 0 = U5/PHY0 unlinked, addr 1
        # = U9/PHY1 with link to MOTU AVB switch at 1000-FD).
        # B50612D needs ≥10 ms of reset after supply stabilizes; 1_000_000
        # cycles ≈ 20 ms at 50 MHz.
        # TX: revert to "as-it-was-when-Auvitran-crashed" — tx_delay=0,
        # no TX nibble swap (stock LiteEth ODDR), no PHY-side TXC delay.
        # That config sent frames out (the malformed AVDECC frames that
        # tripped Hive's parser), so TX path was at least intermittently
        # working.  Marginal but functional, and most importantly: real.
        # RX: keep what works — rx_delay=0, IDDR Q1/Q2 swap, PHY-side
        # RXC delay (shadow_07 bit 8 set by firmware).
        self.ethphy = LiteEthPHYRGMII(
            clock_pads      = platform.request("eth_clocks", 1),
            pads            = platform.request("eth", 1),
            tx_delay        = 0,
            rx_delay        = 0,
            hw_reset_cycles = 1_000_000,
        )

        # Heartbeat counter in eth_rx domain (now L3 via LiteEth) — proves RXC alive.
        from migen.genlib.cdc import MultiReg
        self.eth_rx_heartbeat = CSRStatus(8, description="Counter ticking in eth_rx (PHY1 L3) domain.")
        rx_hb = Signal(32)
        self.sync.eth_rx += rx_hb.eq(rx_hb + 1)
        self.specials += MultiReg(rx_hb[24:32], self.eth_rx_heartbeat.status)

        # PTP Timestamping Unit.
        self.tsu = TSUWithCSRs(sys_clk_freq)

        # nrxslots=2 — confirmed (again, 2026-05-21 with fresh firmware
        # rebuild) that increasing to 4 silently kills TX: entity vanishes
        # from Hive, no ADP egress. Tested both seed=4 and seed=23. The
        # slot-RAM expansion appears to actually break the LiteEth TX path,
        # not just a stale-firmware artifact. Don't try this again without
        # patching LiteEthMAC itself. The AAF flood overrun has to be
        # solved another way (drop early, deferred processing, etc.).
        self.add_ethernet(
            phy            = self.ethphy,
            data_width     = 8,
            with_timestamp = False,
            nrxslots       = 2,
            ntxslots       = 2,
        )

        # Wire TSU latch signals to MAC frame events.
        #
        # add_ethernet(data_width=8) maps to LiteEthMAC dw=32 internally
        # (litex/soc/integration/soc.py:1919), so mac.core.source.data
        # carries 4 wire bytes per beat. A byte-counted ethertype filter
        # at this point is the wrong abstraction. Instead: latch the
        # TSU's instantaneous time on the FIRST BEAT of every frame and
        # push it into a small ring buffer that firmware pops in lock-
        # step with slot consumption. No filter — works for all frames;
        # firmware uses the popped value only when the slot is a PTP
        # frame, and discards it for AVTP/MSRP/etc.
        mac = self.ethmac
        sram_writer = mac.interface.sram.writer

        rx_active     = Signal()
        rx_first_beat = Signal()
        self.comb += rx_first_beat.eq(
            mac.core.source.valid & mac.core.source.ready & ~rx_active
        )
        self.sync += [
            If(mac.core.source.valid & mac.core.source.ready,
                If(~rx_active,
                    rx_active.eq(1),
                ),
                If(mac.core.source.last,
                    rx_active.eq(0),
                ),
            ),
        ]

        # Capture instantaneous TSU time on first beat (registered so
        # subsequent frames can't overwrite before commit).
        captured_rx_ts = Signal(80)
        self.sync += If(rx_first_beat, captured_rx_ts.eq(self.tsu.timestamp))

        # Keep the legacy _rx_ts CSRs working too (single-shot, last-frame).
        self.comb += self.tsu.tsu.rx_latch.eq(rx_first_beat)

        # RX timestamp ring: push when the SRAM writer commits a frame
        # to a slot (stat_fifo.sink.valid pulses for one cycle during
        # FSM TERMINATE state). This guarantees one ring entry per slot
        # firmware will see — drops/MTU-discards don't desync the ring.
        self.rx_ts_fifo = rx_ts_fifo = SyncFIFO(80, 8)

        commit_pulse = sram_writer.stat_fifo.sink.valid
        self.comb += [
            rx_ts_fifo.din.eq(captured_rx_ts),
            rx_ts_fifo.we.eq(commit_pulse & rx_ts_fifo.writable),
        ]

        # ------------------------------------------------------------
        # Stage 2a: AVTPSampleExtractor (observer of MAC RX stream).
        # Same matching pipeline as the Stage-1 filter but ALSO extracts
        # AAF audio samples into per-slot per-channel FIFOs that
        # firmware reads via CSR. 32 KB BRAM for 4×8×256 sample buffer.
        # Still observer mode — frames also reach the CPU SRAM writer
        # as before, but firmware no longer copies audio bytes (gateware
        # has already done it). Stage 2b will gate matched frames out
        # of the LiteEth SRAM path to eliminate the AAF flood entirely.
        # ------------------------------------------------------------
        # First-light shrink: 1 AAF slot. AAF stream on the wire is 8-channel
        # (Milan default) — the extractor FSM still parses all 8, but only
        # ch 0 and ch 1 (L+R) are stored in FIFOs and forwarded to the DAC.
        # CRF stays in firmware dispatcher (~100 Hz). The full 4 slots × 8 ch
        # fanout collapsed sys_clk to 60-70 MHz; grow back to multi-slot
        # once the end-to-end audio path is verified.
        self.submodules.avtp_extractor = avtp_extractor = AVTPSampleExtractor(
            n_slots=1, n_channels=2, wire_channels=8, fifo_depth=256)
        self.comb += [
            avtp_extractor.sink.valid.eq(mac.core.source.valid & mac.core.source.ready),
            avtp_extractor.sink.data.eq(mac.core.source.data),
            avtp_extractor.sink.last.eq(mac.core.source.last),
            avtp_extractor.sink.last_be.eq(mac.core.source.last_be),
        ]

        # Stage 2b: drop matched AAF frames before they raise the CPU event.
        # The extractor combinationally asserts match_at_eof on the same cycle
        # as the last beat of a matched frame; LiteEth's SRAM writer (patched)
        # then routes that frame into DISCARD instead of TERMINATE, so the
        # ev_pending interrupt never fires and the dispatch loop never wakes
        # for it. Audio bytes are already in the extractor FIFOs by then.
        self.comb += mac.interface.sram.writer.discard_in.eq(
            avtp_extractor.match_at_eof)

        # Hardware CRF timestamp extractor — snoops the same RX stream and, for
        # the bound CRF stream_id, captures (avtp_ts, local_rx_ts) pairs into a
        # CSR FIFO the firmware PI servo drains. Decouples media-clock recovery
        # from the congested 2-slot MAC RX path (CRF was being dropped under
        # network flood → "patched but unlocked"). Observe-only; the CRF frame
        # still reaches the CPU (firmware dispatch unchanged) — the servo just
        # reads timestamps from here instead of from the dropped frames.
        self.submodules.crf_ts = crf_ts = CRFTimestampExtractor(fifo_depth=16)
        self.comb += [
            crf_ts.sink.valid.eq(mac.core.source.valid & mac.core.source.ready),
            crf_ts.sink.data.eq(mac.core.source.data),
            crf_ts.sink.last.eq(mac.core.source.last),
            crf_ts.sink.last_be.eq(mac.core.source.last_be),
            crf_ts.tsu_ts.eq(self.tsu.timestamp),
        ]

        # CSRs: pop strobe + 80-bit popped value + level + overflow count.
        self.rx_ts_pop_lo  = CSRStatus(32, description="RX-ring popped nanoseconds.")
        self.rx_ts_pop_mid = CSRStatus(32, description="RX-ring popped seconds[31:0].")
        self.rx_ts_pop_hi  = CSRStatus(16, description="RX-ring popped seconds[47:32].")
        self.rx_ts_pop     = CSRStorage(1, description="Write 1 to pop next ring entry into pop_lo/mid/hi.")
        self.rx_ts_level   = CSRStatus(4, description="RX-ring fill level.")
        self.rx_ts_overflow_count = CSRStatus(32, description="Frames where ring was full at commit (lost ts).")
        self.rx_ts_commit_count   = CSRStatus(32, description="Total RX frames committed (sanity).")

        popped       = Signal(80)
        overflow_cnt = Signal(32)
        commit_cnt   = Signal(32)
        self.sync += [
            If(self.rx_ts_pop.re & rx_ts_fifo.readable,
                popped.eq(rx_ts_fifo.dout),
            ),
            If(commit_pulse,
                commit_cnt.eq(commit_cnt + 1),
                If(~rx_ts_fifo.writable,
                    overflow_cnt.eq(overflow_cnt + 1),
                ),
            ),
        ]
        self.comb += [
            rx_ts_fifo.re.eq(self.rx_ts_pop.re & rx_ts_fifo.readable),
            self.rx_ts_pop_lo.status.eq(popped[0:32]),
            self.rx_ts_pop_mid.status.eq(popped[32:64]),
            self.rx_ts_pop_hi.status.eq(popped[64:80]),
            self.rx_ts_level.status.eq(rx_ts_fifo.level),
            self.rx_ts_overflow_count.status.eq(overflow_cnt),
            self.rx_ts_commit_count.status.eq(commit_cnt),
        ]

        # TX timestamp: latch on first byte sent.
        tx_sof = Signal()
        tx_active = Signal()
        self.sync += [
            If(mac.core.sink.valid & mac.core.sink.ready,
                If(~tx_active,
                    tx_sof.eq(1),
                    tx_active.eq(1),
                ).Else(
                    tx_sof.eq(0),
                ),
                If(mac.core.sink.last,
                    tx_active.eq(0),
                ),
            ).Else(
                tx_sof.eq(0),
            ),
        ]
        self.comb += self.tsu.tsu.tx_latch.eq(tx_sof)

        # Media Clock Recovery NCO — driven by firmware PI servo on CRF.
        self.mcr = MCRNco(sys_clk_freq, fs=48000)

        # I2S DAC output (PCM5102A).
        platform.add_extension(i2s_io())
        i2s_bck_pad  = platform.request("i2s_bck")
        i2s_lrck_pad = platform.request("i2s_lrck")
        i2s_dout_pad = platform.request("i2s_dout")

        # I2S TX runs in the audio clock domain (12.288 MHz).
        # Source: firmware writes L/R CSRs then pulses _i2s_push — the push
        # signals an AsyncFIFO enqueue (sys_clk side). i2s_tx pulls one
        # sample per frame_start (48 kHz, audio_clk side). The FIFO bridges
        # the two clock domains AND absorbs the rate mismatch between MCR-
        # paced firmware push and audio_clk-paced I2S consume — without a
        # FIFO, pushes that land mid-frame are silently dropped by i2s_tx.
        i2s_bck  = Signal()
        i2s_lrck = Signal()
        i2s_dout = Signal()
        i2s_frame_start = Signal()

        self._i2s_audio_l = CSRStorage(24, description="I2S TX left channel (firmware write).")
        self._i2s_audio_r = CSRStorage(24, description="I2S TX right channel (firmware write).")
        self._i2s_push    = CSRStorage(1,  description="Write 1 to push L/R to I2S FIFO.")
        self._i2s_fifo_drops = CSRStatus(32, description="Pushes dropped because FIFO was full.")

        # 48-bit FIFO (24 L + 24 R), depth 32 samples = ~660 µs at 48 kHz.
        i2s_fifo = AsyncFIFO(width=48, depth=32)
        i2s_fifo = ClockDomainsRenamer({"write": "sys", "read": "audio"})(i2s_fifo)
        self.submodules.i2s_fifo = i2s_fifo

        # Write side (sys_clk): firmware pushes L||R when _i2s_push CSR
        # is written. Drop if FIFO full (would happen if MCR > audio_clk
        # rate; drop is preferable to overwriting in-flight samples).
        i2s_fifo_drops = Signal(32)
        push_pulse = self._i2s_push.re
        self.comb += [
            i2s_fifo.din.eq(Cat(self._i2s_audio_r.storage,
                                self._i2s_audio_l.storage)),  # R at bits[0:24], L at [24:48]
            i2s_fifo.we.eq(push_pulse & i2s_fifo.writable),
        ]
        self.sync += If(push_pulse & ~i2s_fifo.writable,
            i2s_fifo_drops.eq(i2s_fifo_drops + 1))
        self.comb += self._i2s_fifo_drops.status.eq(i2s_fifo_drops)

        # Read side (audio_clk): i2s_tx pulses frame_start at each 48 kHz
        # boundary. Use that pulse as the FIFO read enable. dout updates on
        # the next audio_clk edge, ready for i2s_tx's next frame_start.
        i2s_l_from_fifo = Signal(24)
        i2s_r_from_fifo = Signal(24)
        self.comb += [
            i2s_l_from_fifo.eq(i2s_fifo.dout[24:48]),
            i2s_r_from_fifo.eq(i2s_fifo.dout[0:24]),
            i2s_fifo.re.eq(i2s_frame_start & i2s_fifo.readable),
        ]

        self.specials += Instance("i2s_tx",
            i_clk         = ClockSignal("audio"),
            i_rst         = ResetSignal("audio"),
            i_audio_l     = i2s_l_from_fifo,
            i_audio_r     = i2s_r_from_fifo,
            i_audio_valid = i2s_fifo.readable,  # if FIFO empty, i2s_tx repeats last
            o_bck         = i2s_bck,
            o_lrck        = i2s_lrck,
            o_dout            = i2s_dout,
            o_frame_start_out = i2s_frame_start,
        )

        self.comb += [
            i2s_bck_pad.eq(i2s_bck),
            i2s_lrck_pad.eq(i2s_lrck),
            i2s_dout_pad.eq(i2s_dout),
        ]

        platform.add_source(os.path.join(os.path.dirname(__file__), "rtl", "i2s_tx.v"))

        # I2S diagnostics — readable from firmware.
        self._i2s_mmcm_locked = CSRStatus(1,  description="Audio MMCM locked (1=ok).")
        self._i2s_bck_count   = CSRStatus(32, description="BCK toggle counter (sys-clk view).")
        self._i2s_lrck_count  = CSRStatus(32, description="LRCK toggle counter (sys-clk view).")
        self.comb += self._i2s_mmcm_locked.status.eq(self.crg.audio_pll.locked)

        # CDC BCK/LRCK to sys clk (data, not clock) and count edges.
        bck_sync   = Signal(2)
        lrck_sync  = Signal(2)
        bck_count  = Signal(32)
        lrck_count = Signal(32)
        self.sync += [
            bck_sync.eq(Cat(i2s_bck, bck_sync[0])),
            lrck_sync.eq(Cat(i2s_lrck, lrck_sync[0])),
            If(bck_sync[0]  != bck_sync[1],  bck_count.eq(bck_count + 1)),
            If(lrck_sync[0] != lrck_sync[1], lrck_count.eq(lrck_count + 1)),
        ]
        self.comb += [
            self._i2s_bck_count.status.eq(bck_count),
            self._i2s_lrck_count.status.eq(lrck_count),
        ]

        # ---- USB UAC2 sink (Phase 3 / P3.2) ------------------------------
        # Drop-in Verilog from avb-usb-host (tag usb-hs-uac2-working):
        # ULPI link via ultraembedded ulpi_wrapper.v + LUNA USB device.
        # Goal of P3.2: enumerate as USB HS UAC2 INSIDE this AVB bitstream.
        # The decoded 8ch audio stream is exposed but, for now, just drained
        # (ready=1) and fed a nominal 48k feedback. P3.3 wires it to an
        # AsyncFIFO → AAF talker; P3.4 drives feedback from the MCR rate.
        _rtl = os.path.join(os.path.dirname(__file__), "rtl")
        platform.add_source(os.path.join(_rtl, "usb_avb_subsystem.v"))
        platform.add_source(os.path.join(_rtl, "ulpi_wrapper.v"))

        ulpi = platform.request("ulpi")
        ulpi_data_ts = TSTriple(8)
        self.specials += ulpi_data_ts.get_tristate(ulpi.data)

        self.usb_ch_payload = usb_ch_payload = Signal(24)
        self.usb_ch_channel = usb_ch_channel = Signal(3)
        self.usb_ch_valid   = usb_ch_valid   = Signal()
        self.usb_ch_first   = usb_ch_first   = Signal()
        self.usb_ch_last    = usb_ch_last    = Signal()
        usb_ch_ready        = Signal()
        usb_feedback        = Signal(32)

        self.specials += Instance("usb_avb_subsystem",
            i_clk       = ClockSignal("sys"),
            i_rst       = ResetSignal("sys"),
            i_usb_clk   = ClockSignal("usb"),
            i_ulpi_dir_i   = ulpi.dir,
            i_ulpi_nxt_i   = ulpi.nxt,
            i_ulpi_data_i  = ulpi_data_ts.i,
            o_ulpi_data_o  = ulpi_data_ts.o,
            o_ulpi_data_oe = ulpi_data_ts.oe,
            o_ulpi_stp_o   = ulpi.stp,
            o_ulpi_rst_o   = ulpi.rst,
            o_channel_stream_payload = usb_ch_payload,
            o_channel_stream_channel = usb_ch_channel,
            o_channel_stream_valid   = usb_ch_valid,
            o_channel_stream_first   = usb_ch_first,
            o_channel_stream_last    = usb_ch_last,
            i_channel_stream_ready   = usb_ch_ready,
            i_feedback_value         = usb_feedback,
        )
        # P3.2 temporary: drain the stream, nominal 48k feedback
        # (6.0 samples/microframe in 10.14 = 0x18000). Replaced in P3.3/P3.4.
        self.comb += [
            usb_ch_ready.eq(1),
            usb_feedback.eq(0x0001_8000),
        ]

        # LED.
        self.leds = LedChaser(
            pads         = platform.request_all("user_led"),
            sys_clk_freq = sys_clk_freq,
        )

# Build ------------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="AVB-AES3 SoC on Colorlight i9+")
    parser.add_argument("--build",        action="store_true", help="Build bitstream.")
    parser.add_argument("--load",         action="store_true", help="Load bitstream.")
    parser.add_argument("--seed", default=4, type=int, help="nextpnr P&R seed.")
    parser.add_argument("--floorplan", action="store_true",
        help="Inject floorplan_usb.py (--pre-place: confines USB to X>=78). "
             "OFF by default. Patch #3 (LITEETH_PATCHES.md, TX-only sys-datapath) "
             "made the floorplan unnecessary for eth_tx, and confining USB to "
             "X>=78 puts it ~77 columns away from the ULPI pins at X=1 — that "
             "long routing marginalises 60 MHz ULPI sampling and is the root "
             "cause of the build-to-build USB error -71 lottery. Only pass "
             "--floorplan to reproduce the pre-#3 timing-recovery behaviour.")
    parser.add_argument("--sys-clk-freq", default=50e6, type=float, help="System clock frequency.")
    parser.add_argument("--firmware",     default=None,        help="Custom firmware .bin to embed in ROM (replaces BIOS).")
    builder_args = parser.add_argument_group("builder")
    builder_args.add_argument("--output-dir", default=None, help="Output directory.")
    args = parser.parse_args()

    soc_kwargs = dict(
        sys_clk_freq             = int(args.sys_clk_freq),
        cpu_type                 = "vexriscv",
        cpu_variant              = "minimal",
        # 64 KB integrated SRAM holds .data/.bss/stack. The default 8 KB
        # is too small once AAF per-channel buffers (4 KB total) are in
        # play. Use SRAM rather than a separate main_ram region so all
        # firmware data lives in the proven-bootable scratchpad.
        integrated_sram_size     = 0x10000,  # 64 KB SRAM (RW data + stack)
        integrated_rom_size      = 0x18000,  # 96 KB ROM
        uart_name                = "serial",
        # 1 Mbaud + 64-byte HW FIFO. 115200/16 was the LiteX default and
        # at that rate periodic prints (gPTP dump + SRP poll + AVDECC etc)
        # stacked up faster than the main loop could drain the 128-byte
        # software ring, so each `s` command blocked for tens of ms. CH347
        # accepts up to ~9 Mbps over USB; 1 Mbaud is the sweet spot — 8.7×
        # less per-char wait, comfortable margin, every stty/picocom build
        # supports it. The bigger FIFO (16 → 64) also lets a full periodic
        # print fit without ever blocking the main loop.
        uart_baudrate            = 1_000_000,
        uart_fifo_depth          = 64,
    )
    if args.firmware:
        # Pre-load the firmware ourselves so SoCCore doesn't shrink the ROM
        # region down to the firmware size (which causes regions.ld to report
        # a tiny ROM on the next firmware-only build, masking real headroom).
        # Passing a list keeps the configured integrated_rom_size intact.
        from litex.soc.integration.common import get_mem_data
        soc_kwargs["integrated_rom_init"] = get_mem_data(
            args.firmware, endianness="little", data_width=32)

    soc = AVBSoC(**soc_kwargs)

    builder_kwargs = {}
    if args.output_dir:
        builder_kwargs["output_dir"] = args.output_dir

    builder = Builder(soc, **builder_kwargs)
    if args.build:
        # nextpnr-xilinx seed pinned to 4. Patch #3 (LITEETH_PATCHES.md,
        # TX-only sys-datapath) made eth_tx_clk robust across seeds (163 MHz
        # at seed 4 — well above the 125 MHz RGMII requirement), so the seed
        # is no longer a knife-edge timing knob — it's just pinned for build
        # reproducibility.
        #
        # The earlier "USB floorplan + seed sweep" recipe is SUPERSEDED by
        # patch #3 and is also actively harmful to USB: the floorplan
        # confines USB to X>=78 (right half), but the ULPI input pins are
        # at X=1 (left edge), so ULPI sampling has to traverse ~77 columns
        # of routing — that's why rebuilds fail USB enumeration with
        # error -71 (the working standalone has no such constraint and the
        # wrapper lands naturally near the ULPI pins). floorplan_usb.py is
        # kept for opt-in via --floorplan but is OFF by default now.
        if args.floorplan:
            fp = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "floorplan_usb.py")
            soc.platform.toolchain._pnr_opts += " --pre-place {} ".format(fp)
        builder.build(seed=args.seed)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
