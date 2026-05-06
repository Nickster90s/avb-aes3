"""
RGMII PHY for 7-series with runtime-tunable RX IDELAY.

Drop-in replacement for liteeth.phy.s7rgmii.LiteEthPHYRGMII. The RX data
and rx_ctl IDELAYE2 instances run in VAR_LOAD mode so the tap count
(0..31) can be reloaded at runtime via a CSR. Use this to sweep for the
optimal RGMII RX sample point on a given board/PHY combination.

CSRs exposed (under whatever name the SoC assigns):
    rx_idelay_tap   : CSRStorage(5)  - tap value 0..31
    rx_idelay_load  : CSRStorage(1)  - rising edge -> load tap into IDELAYE2

Loading: write tap value, then write 1 then 0 to load. The strobe is
synced into the iodelay clock domain.
"""

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
from migen.genlib.cdc import PulseSynchronizer

from litex.gen import *
from litex.soc.cores.clock import S7PLL
from litex.soc.interconnect.csr import CSRStorage

from liteeth.common import *
from liteeth.phy.common import *
from liteeth.phy.s7rgmii import LiteEthPHYRGMIITX, LiteEthPHYRGMIICRG


class LiteEthPHYRGMIIRXVar(LiteXModule):
    """RX side with VAR_LOAD IDELAYE2 — tap count drivable at runtime."""

    def __init__(self, pads, iodelay_clk_freq=200e6):
        self.source     = source = stream.Endpoint(eth_phy_description(8))
        self.tap_value  = Signal(5)
        self.tap_load   = Signal()  # one-cycle strobe in eth_rx domain

        # # #

        assert iodelay_clk_freq in [200e6, 300e6, 400e6]

        # The IDELAYE2 LD strobe must be synchronous to the same clock that
        # drives the IDELAY's C input. Use the iodelay refclk domain for
        # both — that way the load is unambiguously timed.
        ld = Signal()
        ps = PulseSynchronizer("eth_rx", "idelay")
        self.submodules += ps
        self.comb += [
            ps.i.eq(self.tap_load),
            ld.eq(ps.o),
        ]

        # The tap value crosses with the strobe: hold it stable across the
        # synchronizer, gray-not-needed for 5 bits in practice but use
        # MultiReg for safety.
        from migen.genlib.cdc import MultiReg
        tap_sync = Signal(5)
        self.specials += MultiReg(self.tap_value, tap_sync, "idelay")

        rx_ctl_ibuf    = Signal()
        rx_ctl_idelay  = Signal()
        rx_ctl         = Signal()
        rx_data_ibuf   = Signal(4)
        rx_data_idelay = Signal(4)
        rx_data        = Signal(8)

        self.specials += [
            Instance("IBUF", i_I=pads.rx_ctl, o_O=rx_ctl_ibuf),
            Instance("IDELAYE2",
                p_IDELAY_TYPE      = "VAR_LOAD",
                p_IDELAY_VALUE     = 0,
                p_REFCLK_FREQUENCY = iodelay_clk_freq/1e6,
                p_HIGH_PERFORMANCE_MODE = "TRUE",
                p_PIPE_SEL         = "FALSE",
                p_DELAY_SRC        = "IDATAIN",
                p_SIGNAL_PATTERN   = "DATA",
                i_C        = ClockSignal("idelay"),
                i_LD       = ld,
                i_CE       = 0,
                i_INC      = 0,
                i_LDPIPEEN = 0,
                i_CINVCTRL = 0,
                i_CNTVALUEIN = tap_sync,
                i_IDATAIN  = rx_ctl_ibuf,
                o_DATAOUT  = rx_ctl_idelay,
            ),
            Instance("IDDR",
                p_DDR_CLK_EDGE = "SAME_EDGE",
                i_C  = ClockSignal("eth_rx"),
                i_CE = 1,
                i_S  = 0,
                i_R  = 0,
                i_D  = rx_ctl_idelay,
                o_Q1 = rx_ctl,
                o_Q2 = Signal(),
            ),
        ]
        for i in range(4):
            self.specials += [
                Instance("IBUF",
                    i_I = pads.rx_data[i],
                    o_O = rx_data_ibuf[i],
                ),
                Instance("IDELAYE2",
                    p_IDELAY_TYPE      = "VAR_LOAD",
                    p_IDELAY_VALUE     = 0,
                    p_REFCLK_FREQUENCY = iodelay_clk_freq/1e6,
                    p_HIGH_PERFORMANCE_MODE = "TRUE",
                    p_PIPE_SEL         = "FALSE",
                    p_DELAY_SRC        = "IDATAIN",
                    p_SIGNAL_PATTERN   = "DATA",
                    i_C        = ClockSignal("idelay"),
                    i_LD       = ld,
                    i_CE       = 0,
                    i_INC      = 0,
                    i_LDPIPEEN = 0,
                    i_CINVCTRL = 0,
                    i_CNTVALUEIN = tap_sync,
                    i_IDATAIN  = rx_data_ibuf[i],
                    o_DATAOUT  = rx_data_idelay[i],
                ),
                Instance("IDDR",
                    p_DDR_CLK_EDGE = "SAME_EDGE",
                    i_C  = ClockSignal("eth_rx"),
                    i_CE = 1,
                    i_S  = 0,
                    i_R  = 0,
                    i_D  = rx_data_idelay[i],
                    o_Q1 = rx_data[i],
                    o_Q2 = rx_data[i+4],
                ),
            ]

        rx_ctl_d = Signal()
        self.sync += rx_ctl_d.eq(rx_ctl)

        last = Signal()
        self.comb += last.eq(~rx_ctl & rx_ctl_d)
        self.sync += [
            source.valid.eq(rx_ctl),
            source.data.eq(rx_data),
        ]
        self.comb += source.last.eq(last)


class LiteEthPHYRGMIIVar(LiteXModule):
    """Drop-in replacement for LiteEthPHYRGMII with runtime-tunable RX IDELAY.

    The SoC must drive the `tap_value_in` (5b) and `tap_load_in` (rising-edge
    pulse in sys domain) signals from CSRs of its own — putting CSRs in the
    PHY itself doesn't surface them via LiteX auto-bridge in this codebase.
    """

    dw          = 8
    tx_clk_freq = 125e6
    rx_clk_freq = 125e6

    def __init__(self, clock_pads, pads,
                 with_hw_init_reset = True,
                 tx_delay           = 0,
                 iodelay_clk_freq   = 200e6,
                 hw_reset_cycles    = 256):
        # External signals the SoC drives.
        self.tap_value_in = Signal(5)
        self.tap_load_in  = Signal()  # rising-edge pulse in sys domain

        self.crg = LiteEthPHYRGMIICRG(clock_pads, pads, with_hw_init_reset,
                                      tx_delay, hw_reset_cycles)
        self.tx  = ClockDomainsRenamer("eth_tx")(LiteEthPHYRGMIITX(pads))

        rx = LiteEthPHYRGMIIRXVar(pads, iodelay_clk_freq)
        self.rx = ClockDomainsRenamer("eth_rx")(rx)

        # Cross the load strobe (sys domain) into eth_rx as a one-cycle pulse.
        from migen.genlib.cdc import PulseSynchronizer
        load_ps = PulseSynchronizer("sys", "eth_rx")
        self.submodules += load_ps
        load_prev  = Signal()
        load_pulse = Signal()
        self.sync += load_prev.eq(self.tap_load_in)
        self.comb += load_pulse.eq(self.tap_load_in & ~load_prev)
        self.comb += [
            load_ps.i.eq(load_pulse),
            rx.tap_load.eq(load_ps.o),
        ]
        self.comb += rx.tap_value.eq(self.tap_value_in)

        self.sink, self.source = self.tx.sink, self.rx.source

        if hasattr(pads, "mdc"):
            self.mdio = LiteEthPHYMDIO(pads)
