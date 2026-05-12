#!/usr/bin/env python3

#
# AVB AES3 SoC for Colorlight i9plus v6.1
#
# LiteX SoC with:
# - VexRiscv CPU + SRAM
# - LiteEth RGMII MAC (Wishbone interface for firmware raw frame access)
# - LiteEthTSU (PTP Timestamping Unit) with CSRs for gPTP firmware
# - AES3 TX/RX with hardware FIFOs and CSR interface
# - UART (PMOD)
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

from migen.genlib.fifo import SyncFIFO

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

# Audio Clock Generator --------------------------------------------------------------------------------

class AudioClkGen(LiteXModule):
    """Generates audio-rate clock enables from the 12.288 MHz audio clock domain.

    Provides:
      - biphase_tick: 6.144 MHz enable (AES3 biphase symbol rate at 48 kHz)
      - fs_48k:       48 kHz sample rate pulse (every 256 audio clocks)
      - fs_96k:       96 kHz sample rate pulse (every 128 audio clocks)

    The 12.288 MHz clock is crossed into sys_clk domain via pulse synchronizers,
    so all outputs are in the sys_clk domain.
    """
    def __init__(self):
        # Outputs (sys_clk domain, active-high single-cycle pulses)
        self.biphase_tick = Signal()
        self.fs_48k       = Signal()
        self.fs_96k       = Signal()

        # --- Audio clock domain dividers ---
        # 12.288 MHz / 2 = 6.144 MHz biphase tick
        # 12.288 MHz / 128 = 96 kHz
        # 12.288 MHz / 256 = 48 kHz
        audio_div = Signal(8)  # 0-255 counter in audio domain

        # Toggle signals in audio domain (for CDC)
        biphase_toggle = Signal()
        fs_96k_toggle  = Signal()
        fs_48k_toggle  = Signal()

        self.sync.audio += [
            audio_div.eq(audio_div + 1),
            # Biphase: toggle every clock (div-by-2 = 6.144 MHz)
            biphase_toggle.eq(~biphase_toggle),
            # 96 kHz: toggle every 128 clocks
            If(audio_div[6:8] == 0,
                fs_96k_toggle.eq(~fs_96k_toggle),
            ),
            # 48 kHz: toggle every 256 clocks (on wrap)
            If(audio_div == 0,
                fs_48k_toggle.eq(~fs_48k_toggle),
            ),
        ]

        # --- Clock domain crossing (toggle-to-pulse) ---
        # Synchronize toggles from audio→sys domain, detect edges

        for toggle, output, name in [
            (biphase_toggle, self.biphase_tick, "biphase"),
            (fs_96k_toggle,  self.fs_96k,       "fs96k"),
            (fs_48k_toggle,  self.fs_48k,       "fs48k"),
        ]:
            # 2-stage synchronizer
            sync1 = Signal(name=f"{name}_sync1")
            sync2 = Signal(name=f"{name}_sync2")
            sync3 = Signal(name=f"{name}_sync3")
            self.sync += [
                sync1.eq(toggle),
                sync2.eq(sync1),
                sync3.eq(sync2),
            ]
            self.comb += output.eq(sync2 ^ sync3)

        # --- CSR status registers ---
        self._fs_48k_count = CSRStatus(32, description="48 kHz tick counter (for diagnostics).")
        fs_count = Signal(32)
        self.sync += If(self.fs_48k, fs_count.eq(fs_count + 1))
        self.comb += self._fs_48k_count.status.eq(fs_count)


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


# AES3 Pin Extension -----------------------------------------------------------------------------------

def i2s_io():
    return [
        # SODIMM pin 46 → FPGA ball U7 → PCM5102A BCK
        ("i2s_bck",  0, Pins("U7"), IOStandard("LVCMOS33")),
        # SODIMM pin 48 → FPGA ball U6 → PCM5102A LRCK / LCK
        ("i2s_lrck", 0, Pins("U6"), IOStandard("LVCMOS33")),
        # SODIMM pin 50 → FPGA ball U5 → PCM5102A DIN
        ("i2s_dout", 0, Pins("U5"), IOStandard("LVCMOS33")),
    ]

def aes3_io():
    """AES3 I/O pins on SODIMM connector."""
    return [
        ("aes3_out", 0, Pins("dimm:42"), IOStandard("LVCMOS33")),  # P5 — to RS-422 TX driver
        ("aes3_in",  0, Pins("dimm:44"), IOStandard("LVCMOS33")),  # T6 — from RS-422 RX receiver
    ]

# AES3 CSR Wrapper -------------------------------------------------------------------------------------

class AES3WithCSRs(LiteXModule):
    """Wraps AES3 TX and RX Verilog modules with CSRs and hardware FIFOs.

    Audio data path:
      AES3 RX → rx_fifo → CSR read (firmware) → AVTP ring buffer → network
      network → AVTP ring buffer → CSR write (firmware) → tx_fifo → AES3 TX
    """
    def __init__(self, platform, sys_clk_freq, pads_out, pads_in, biphase_tick=None):
        # --- AES3 RX (input from external source) ---

        # Signals from aes3_rx Verilog module
        rx_audio_l     = Signal(24)
        rx_audio_r     = Signal(24)
        rx_audio_valid = Signal()
        rx_locked      = Signal()
        rx_is_96k      = Signal()
        rx_error_count = Signal(4)
        rx_cs          = Signal(192)
        rx_cs_valid    = Signal()

        self.specials += Instance("aes3_rx",
            p_CLK_FREQ = sys_clk_freq,
            i_clk      = ClockSignal("sys"),
            i_rst      = ResetSignal("sys"),
            i_aes3_in  = pads_in,
            o_audio_l      = rx_audio_l,
            o_audio_r      = rx_audio_r,
            o_audio_valid  = rx_audio_valid,
            o_audio_sub    = Signal(24),  # unused
            o_sub_valid    = Signal(),    # unused
            o_channel_status = rx_cs,
            o_cs_valid     = rx_cs_valid,
            o_locked       = rx_locked,
            o_is_96k       = rx_is_96k,
            o_error_count  = rx_error_count,
        )

        # RX FIFO: AES3 RX writes sample pairs, firmware reads via CSR.
        # Width = 48 bits (24-bit L + 24-bit R), depth = 64 samples (~1.3 ms at 48k).
        self.rx_fifo = rx_fifo = SyncFIFO(48, 64)

        self.comb += [
            rx_fifo.din.eq(Cat(rx_audio_r, rx_audio_l)),  # [47:24]=L, [23:0]=R
            rx_fifo.we.eq(rx_audio_valid),
        ]

        # --- AES3 TX (output to external destination) ---

        # Signals to aes3_tx Verilog module
        tx_audio_l     = Signal(24)
        tx_audio_r     = Signal(24)
        tx_audio_valid = Signal()
        tx_audio_ready = Signal()
        tx_cs          = Signal(192)

        use_ext_tick = 1 if biphase_tick is not None else 0
        ext_tick_sig = biphase_tick if biphase_tick is not None else Signal()

        self.specials += Instance("aes3_tx",
            p_CLK_FREQ     = sys_clk_freq,
            p_FS           = 48000,
            p_USE_EXT_TICK = use_ext_tick,
            i_clk      = ClockSignal("sys"),
            i_rst      = ResetSignal("sys"),
            i_audio_l      = tx_audio_l,
            i_audio_r      = tx_audio_r,
            i_audio_valid  = tx_audio_valid,
            o_audio_ready  = tx_audio_ready,
            i_ext_biphase_tick = ext_tick_sig,
            i_channel_status = tx_cs,
            o_aes3_out     = pads_out,
        )

        # TX FIFO: firmware writes sample pairs via CSR, AES3 TX reads.
        # Same 48-bit width, depth 64.
        self.tx_fifo = tx_fifo = SyncFIFO(48, 64)

        # Feed AES3 TX from FIFO when TX is ready and FIFO has data.
        self.comb += [
            tx_audio_l.eq(tx_fifo.dout[24:48]),
            tx_audio_r.eq(tx_fifo.dout[0:24]),
            tx_audio_valid.eq(tx_fifo.readable & tx_audio_ready),
            tx_fifo.re.eq(tx_fifo.readable & tx_audio_ready),
        ]

        # --- CSR Registers ---

        # RX status
        self._rx_locked  = CSRStatus(1,  description="AES3 RX PLL locked.")
        self._rx_is_96k  = CSRStatus(1,  description="AES3 RX detected 96 kHz.")
        self._rx_errors  = CSRStatus(4,  description="AES3 RX error counter.")
        self._rx_level   = CSRStatus(7,  description="AES3 RX FIFO level (0-64).")

        self.comb += [
            self._rx_locked.status.eq(rx_locked),
            self._rx_is_96k.status.eq(rx_is_96k),
            self._rx_errors.status.eq(rx_error_count),
            self._rx_level.status.eq(rx_fifo.level),
        ]

        # RX audio read: read _rx_audio_l first (latches pair), then _rx_audio_r.
        self._rx_audio_l = CSRStatus(24, description="AES3 RX left channel — read first to pop FIFO.")
        self._rx_audio_r = CSRStatus(24, description="AES3 RX right channel.")

        rx_l_latch = Signal(24)
        rx_r_latch = Signal(24)

        # Pop FIFO and latch on read of _rx_audio_l.
        self.comb += rx_fifo.re.eq(self._rx_audio_l.we & rx_fifo.readable)
        self.sync += If(self._rx_audio_l.we & rx_fifo.readable,
            rx_l_latch.eq(rx_fifo.dout[24:48]),
            rx_r_latch.eq(rx_fifo.dout[0:24]),
        )
        self.comb += [
            self._rx_audio_l.status.eq(rx_l_latch),
            self._rx_audio_r.status.eq(rx_r_latch),
        ]

        # TX audio write: write _tx_audio_l, then _tx_audio_r, then strobe _tx_push.
        self._tx_audio_l = CSRStorage(24, description="AES3 TX left channel.")
        self._tx_audio_r = CSRStorage(24, description="AES3 TX right channel.")
        self._tx_push    = CSRStorage(1,  description="Write 1 to push L/R pair to TX FIFO.")
        self._tx_level   = CSRStatus(7,   description="AES3 TX FIFO level (0-64).")

        self.comb += [
            tx_fifo.din.eq(Cat(self._tx_audio_r.storage, self._tx_audio_l.storage)),
            tx_fifo.we.eq(self._tx_push.re & tx_fifo.writable),
            self._tx_level.status.eq(tx_fifo.level),
        ]

        # TX channel status (192 bits, written as 6 × 32-bit CSRs).
        self._tx_cs0 = CSRStorage(32, description="TX channel status [31:0].")
        self._tx_cs1 = CSRStorage(32, description="TX channel status [63:32].")
        self._tx_cs2 = CSRStorage(32, description="TX channel status [95:64].")
        self._tx_cs3 = CSRStorage(32, description="TX channel status [127:96].")
        self._tx_cs4 = CSRStorage(32, description="TX channel status [159:128].")
        self._tx_cs5 = CSRStorage(32, description="TX channel status [191:160].")

        self.comb += tx_cs.eq(Cat(
            self._tx_cs0.storage, self._tx_cs1.storage,
            self._tx_cs2.storage, self._tx_cs3.storage,
            self._tx_cs4.storage, self._tx_cs5.storage,
        ))

        # RX channel status (read-only, latched on cs_valid).
        self._rx_cs0 = CSRStatus(32, description="RX channel status [31:0].")
        self._rx_cs1 = CSRStatus(32, description="RX channel status [63:32].")
        self._rx_cs2 = CSRStatus(32, description="RX channel status [95:64].")
        self._rx_cs3 = CSRStatus(32, description="RX channel status [127:96].")
        self._rx_cs4 = CSRStatus(32, description="RX channel status [159:128].")
        self._rx_cs5 = CSRStatus(32, description="RX channel status [191:160].")

        rx_cs_latch = Signal(192)
        self.sync += If(rx_cs_valid, rx_cs_latch.eq(rx_cs))
        self.comb += [
            self._rx_cs0.status.eq(rx_cs_latch[0:32]),
            self._rx_cs1.status.eq(rx_cs_latch[32:64]),
            self._rx_cs2.status.eq(rx_cs_latch[64:96]),
            self._rx_cs3.status.eq(rx_cs_latch[96:128]),
            self._rx_cs4.status.eq(rx_cs_latch[128:160]),
            self._rx_cs5.status.eq(rx_cs_latch[160:192]),
        ]

        # Add Verilog source files.
        platform.add_source(os.path.join(os.path.dirname(__file__), "rtl", "aes3_rx.v"))
        platform.add_source(os.path.join(os.path.dirname(__file__), "rtl", "aes3_tx.v"))

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

        # Ethernet MAC (Wishbone interface for firmware raw frame access).
        # Pass TSU timestamp for RX/TX frame timestamping.
        self.add_ethernet(
            phy            = self.ethphy,
            data_width     = 8,
            with_timestamp = False,  # We use our own TSU, not timer0 uptime
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

        # Audio Clock Generator.
        self.audio_clk = AudioClkGen()

        # Media Clock Recovery NCO — driven by firmware PI servo on CRF.
        self.mcr = MCRNco(sys_clk_freq, fs=48000)

        # AES3 TX/RX.
        platform.add_extension(aes3_io())
        aes3_out_pad = platform.request("aes3_out")
        aes3_in_pad  = platform.request("aes3_in")
        self.aes3 = AES3WithCSRs(platform, sys_clk_freq, aes3_out_pad, aes3_in_pad,
                                  biphase_tick=self.audio_clk.biphase_tick)

        # I2S DAC output (PCM5102A).
        platform.add_extension(i2s_io())
        i2s_bck_pad  = platform.request("i2s_bck")
        i2s_lrck_pad = platform.request("i2s_lrck")
        i2s_dout_pad = platform.request("i2s_dout")

        # I2S TX runs in the audio clock domain (12.288 MHz).
        # Audio source: AES3 RX FIFO output (L/R from the RX side).
        i2s_bck  = Signal()
        i2s_lrck = Signal()
        i2s_dout = Signal()

        # CSR to select I2S source: 0 = AES3 RX, 1 = AVTP RX (firmware-written)
        self._i2s_audio_l = CSRStorage(24, description="I2S TX left channel (firmware write).")
        self._i2s_audio_r = CSRStorage(24, description="I2S TX right channel (firmware write).")
        self._i2s_push    = CSRStorage(1,  description="Write 1 to push L/R to I2S.")
        self._i2s_source  = CSRStorage(1,  description="I2S source: 0=AES3 RX direct, 1=firmware.")

        # I2S sample holding registers (sys_clk domain)
        i2s_l = Signal(24)
        i2s_r = Signal(24)
        i2s_valid = Signal()

        # Mux: AES3 RX direct or firmware-written samples
        self.comb += [
            If(self._i2s_source.storage == 0,
                # Direct from AES3 RX (last valid sample pair)
                i2s_l.eq(self.aes3._rx_audio_l.status),
                i2s_r.eq(self.aes3._rx_audio_r.status),
            ).Else(
                i2s_l.eq(self._i2s_audio_l.storage),
                i2s_r.eq(self._i2s_audio_r.storage),
            ),
        ]

        self.specials += Instance("i2s_tx",
            i_clk         = ClockSignal("audio"),
            i_rst         = ResetSignal("audio"),
            i_audio_l     = i2s_l,
            i_audio_r     = i2s_r,
            i_audio_valid = 1,  # Always valid — repeat last sample if no new data
            o_bck         = i2s_bck,
            o_lrck        = i2s_lrck,
            o_dout        = i2s_dout,
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
        # nextpnr-xilinx seed selection. The design is on the timing edge.
        # eth_tx_clk wants 125 MHz; a 10-seed sweep on the AEM-fixed firmware
        # gave 99–138 MHz, with seed=23 (137.89 MHz) and seed=3 (130.24 MHz)
        # cleanly passing. Seed=23 picked for best margin. If a future change
        # pushes placement worse, re-sweep:
        #     for s in 2 3 4 5 7 11 13 17 19 23; do
        #         nextpnr-xilinx … --seed $s --freq 125 …
        #     done
        # and pick the seed with the highest eth_tx_clk PASS.
        builder.build(seed=107)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
