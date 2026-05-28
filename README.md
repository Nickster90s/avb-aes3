# AVB-AES3 Endpoint — Colorlight i9plus v6.1

Standalone FPGA AVB endpoint on a single XC7A50T. One bitstream now carries:

- **gPTP** (IEEE 802.1AS slave, ±30 ns servo lock to a Class-A master)
- **AVTP** (IEEE 1722) **AVDECC** (1722.1, Milan v1.2 compliant in Hive), **MSRP**
- **AAF** 8-channel listener + talker, **CRF** media-clock recovery with PI servo
- **AES3** (S/PDIF/AES3id) digital I/O
- **PCM5102A** I2S DAC for monitoring
- **USB UAC2 High-Speed** device (`1209:eab1`) — playback/capture for DAW
- **Gigabit RGMII** to the AVB switch

VexRiscv runs all the protocol stacks bare-metal — no Linux on the device.

---

## 1. Status (what works as of 2026-05-28)

| Subsystem  | State                                                            |
|------------|------------------------------------------------------------------|
| gPTP       | Locks ±30 ns to MOTU AVB Switch / Auvitran grandmaster           |
| AVDECC     | Hive sees entity, full AEM tree, patchable in Hive               |
| MSRP       | Talker + Listener registrar; READY substate at CONNECT_RX        |
| ACMP       | Fast-path + slow-path (zero-stream-id) listener flows            |
| AAF        | 8 ch RX + TX at 48 kHz / 24 bit, with jitter buffer              |
| CRF        | RX parser feeds NCO PI servo (±2 µs lock, single-sample exit)    |
| AES3       | TX wired to AAF RX ch 0+1 (loopback bench-verified)              |
| USB UAC2   | HS enumerate + playback robust; capture flow-control TBD         |
| Eth gigabit| `eth_tx_clk` 150-163 MHz robust (TX-only sys-datapath patch)     |

Working build archived at
`bitstreams/675fb9e-dirty_2026-05-28_1001_usb-avb-working-no-floorplan.bit`
with full `git log -1` sidecar.

---

## 2. Quickstart

```sh
# Environment (LiteX/Migen + openXC7 toolchain)
export CHIPDB=/home/lisp/FPGA/demo-projects/chipdb
export PRJXRAY_DB_DIR=/home/lisp/openxc7/openxc7/opt/nextpnr-xilinx/external/prjxray-db

# Build firmware then gateware (one combined build, seed pinned to 4,
# floorplan OFF by default — see §8 for why)
cd /home/lisp/FPGA/avb-aes3
( cd firmware && make )
python3 avb_soc.py --build --firmware firmware/firmware.bin --seed 4

# Flash to SRAM (volatile — see §6.2 for SPI-flash recipe)
sudo /home/lisp/openocd/src/openocd \
  -s /home/lisp/openocd/tcl \
  -f /home/lisp/FPGA/Colorlight-FPGA-Projects/tools/ch347.cfg \
  -c "init; pld load 0 build/colorlight_i9plus/gateware/colorlight_i9plus.bit; exit"

# Console (NOTE: 1 Mbaud, not the old 115200)
sudo picocom -b 1000000 /dev/ttyACM0

# Archive the .bit so the next build can't overwrite it
./bitstreams/archive.sh some-meaningful-label
```

---

## 3. Hardware

### 3.1 Board

- **Colorlight i9plus v6.1** (Xilinx XC7A50T-FGG484)
- Board pinout: `/home/lisp/FPGA/Colorlight-FPGA-Projects/colorlight_i9plus_v6.1.md`
- The cabled-up RJ45 jack on this specific board is U9 = PHY1 (MDIO address 1), so
  LiteEth uses `eth_clocks` / `eth` index 1.

### 3.2 CH347T (JTAG + UART over one USB-C)

| CH347T pin       | Signal | FPGA pin | Direction               |
|------------------|--------|----------|-------------------------|
| TXD1             | UART   | M3       | CH347 → FPGA            |
| RXD1             | UART   | R3       | FPGA → CH347            |
| TCK/TMS/TDI/TDO  | JTAG   | (header) | bidirectional           |

Both UART and JTAG land on `/dev/ttyACM0`/the same CH347 USB. UART runs at
**1 Mbaud + 64-byte HW FIFO** so periodic prints (gPTP/SRP/AVDECC dumps)
never block the main loop.

### 3.3 USB UAC2 (USB3300 ULPI breakout on P2)

| Signal | FPGA pin | SODIMM pin | Notes                                  |
|--------|----------|------------|----------------------------------------|
| CLK    | T4       | 51         | MRCC pin (clock-capable, required)     |
| DIR    | T3       | 49         |                                        |
| NXT    | U2       | 57         |                                        |
| STP    | U3       | 59         |                                        |
| RST    | R2       | 41         | **Active HIGH** (USB3300 datasheet)    |
| D0–D7  | V2 V3 W1 W2 Y1 AA1 AB1 Y2 | 61–75 | bidirectional bus                |

ULPI wiring critical: each timing-critical signal twisted with **its own GND**,
never another signal — see `feedback_ulpi_twisted_pair_wiring` memory.

### 3.4 PCM5102A I2S DAC (optional)

| DAC label | FPGA pin | SODIMM pin | Notes                                                              |
|-----------|----------|------------|--------------------------------------------------------------------|
| VIN       | 3V3      | —          | 3.3 V power                                                        |
| GND       | GND      | —          | Ground                                                             |
| SCK       | **GND**  | —          | Tie LOW so the PCM5102A uses its internal PLL                      |
| BCK       | U7       | 46         | 3.072 MHz bit clock                                                |
| LCK       | U6       | 48         | 48 kHz word select (LRCK)                                          |
| DIN       | U5       | 50         | Serial audio data                                                  |

### 3.5 AES3 I/O (optional)

| Signal   | FPGA pin | SODIMM pin | External hardware                                  |
|----------|----------|------------|----------------------------------------------------|
| AES3 OUT | P5       | 42         | RS-422 driver → 110 Ω xfmr → XLR-3M                |
| AES3 IN  | T6       | 44         | XLR-3F → 110 Ω xfmr → RS-422 receiver              |

---

## 4. Build

### 4.1 Environment

```sh
export CHIPDB=/home/lisp/FPGA/demo-projects/chipdb
export PRJXRAY_DB_DIR=/home/lisp/openxc7/openxc7/opt/nextpnr-xilinx/external/prjxray-db
```

These two env vars are mandatory for openXC7 nextpnr-xilinx. The Python tooling
(migen / litex / amaranth) is found automatically through
`~/.local/lib/python3.11/site-packages/litex-tools.pth` — no venv activation needed.

### 4.2 Build flow

The bitstream **embeds** the firmware in its BRAM ROM, so the firmware must be
compiled *before* the gateware build:

```sh
cd /home/lisp/FPGA/avb-aes3/firmware && make           # rebuilds firmware.bin
cd ..                                                  # back to repo root
python3 avb_soc.py --build --firmware firmware/firmware.bin --seed 4
```

If you `--build` without `--firmware`, the ROM bakes the LiteX BIOS only and
you get a `litex>` prompt instead of the `[AVB-AES3]` boot banner.

A clean build takes ~6-10 min on this box. Watch for:

```
Info: Max frequency for clock         'eth_tx_clk': ~150 MHz (PASS at 125)
Info: Max frequency for clock         'eth_rx_clk': ~140 MHz (PASS at 125)
2 warnings, 0 errors
```

### 4.3 Seed and floorplan

- `--seed 4` is pinned in `avb_soc.py` defaults. The TX-only sys-datapath patch
  (LITEETH_PATCHES.md #3) made `eth_tx_clk` robust across seeds — the pin is just
  for build reproducibility, not because the design is on a timing edge.
- The `--floorplan` flag is **OFF by default**. It was previously the only way
  to recover gigabit `eth_tx_clk` with the USB block present, but patch #3
  superseded it and the floorplan was actively harming USB ULPI sampling (see
  §8). Only pass `--floorplan` if you're reproducing the pre-patch-#3 recipe.

### 4.4 Verify

After a build, before declaring it good:

```sh
ls -la build/colorlight_i9plus/gateware/colorlight_i9plus.bit   # fresh
grep "Max frequency.*'eth_tx_clk'"   build/.../colorlight_i9plus.log   # > 125
```

After flashing, the boot output on the 1 Mbaud serial console should show:

```
[gPTP] Initialized. MAC=02:00:00:00:00:42
[SRP] Initialized (MSRP)
[AVDECC] Entity ID=02:00:00:ff:fe:00:00:42
```

…and on the host side `lsusb` should list
`1209:eab1 Generic N-Series AVB Switchover`.

---

## 5. Bitstream archive

This project's original sin was losing a known-working bitstream
("fp_build5") when a later `--build` overwrote the same output path —
source was in git but yosys non-determinism meant rebuilds didn't reproduce.
`bitstreams/archive.sh` exists to make that class of loss impossible:

```sh
./bitstreams/archive.sh [label]
```

Snapshots the current `build/.../colorlight_i9plus.bit` to
`bitstreams/<short-sha>[-dirty]_YYYY-MM-DD_HHMM[_label].bit` and writes a
sidecar `.info` with `git log -1` + uncommitted-diff stat. **Run it after every
verified-working build.**

`.bit` and `.info` are gitignored — the script + ignore are tracked.

---

## 6. Programming

### 6.1 Volatile (SRAM — disappears on power cycle)

```sh
sudo /home/lisp/openocd/src/openocd \
  -s /home/lisp/openocd/tcl \
  -f /home/lisp/FPGA/Colorlight-FPGA-Projects/tools/ch347.cfg \
  -c "init; pld load 0 build/colorlight_i9plus/gateware/colorlight_i9plus.bit; exit"
```

The board does **not** have `openFPGALoader` available — only the local
openocd build above.

### 6.2 Persistent (SPI flash — survives power-cycle)

Two steps. The MX25L128 flash is write-protected from the factory; unlock once
per board, then write.

```sh
# Step 1 — unlock (first time only per board)
sudo /home/lisp/openocd/src/openocd -s /home/lisp/openocd/tcl \
  -f /home/lisp/FPGA/Colorlight-FPGA-Projects/tools/ch347.cfg \
  -c "init; pld load 0 /home/lisp/FPGA/Colorlight-FPGA-Projects/tools/unlock_flash_xc7a50t.bit; exit"

# Step 2 — write + reboot
TOOLS=/home/lisp/FPGA/Colorlight-FPGA-Projects/tools
BIT=/home/lisp/FPGA/avb-aes3/build/colorlight_i9plus/gateware/colorlight_i9plus.bit
sudo /home/lisp/openocd/src/openocd -s /home/lisp/openocd/tcl -f $TOOLS/ch347.cfg -c "
  set XC7_JSHUTDOWN 0x0d
  set XC7_JPROGRAM  0x0b
  set XC7_BYPASS    0x3f
  init
  pld load 0 $TOOLS/bscan_spi_xc7a50t.bit
  reset halt
  flash probe 0
  flash protect 0 0 50 off
  flash write_image erase $BIT 0x0 bin
  irscan xc7.tap \$XC7_JSHUTDOWN
  irscan xc7.tap \$XC7_JPROGRAM
  runtest 60000
  runtest 2000
  irscan xc7.tap \$XC7_BYPASS
  runtest 2000
  reset
" -c exit
```

---

## 7. Console

```sh
sudo picocom -b 1000000 /dev/ttyACM0    # Ctrl-A Ctrl-X to exit
```

If `/dev/ttyACM0` is a regular file (sometimes recreated wrong after kernel
oddities), restore the char device:

```sh
sudo rm /dev/ttyACM0
sudo mknod /dev/ttyACM0 c 166 0
sudo chgrp dialout /dev/ttyACM0 && sudo chmod 660 /dev/ttyACM0
```

### Commands (current firmware)

| Key | Action                                                                  |
|-----|-------------------------------------------------------------------------|
| `h` | Show command help                                                       |
| `s` | Status — gPTP, SRP, AVDECC, AVTP TX/RX counters                         |
| `e` | RX dispatcher stats (ptp / avtp / msrp / stream_mcast)                  |
| `m` | MCR/CRF state — locked, increment, |delta| stats                        |
| `a` | AAF channel state — RX bound stream / TX activity                       |
| `t` | Toggle AVB **talker** (also enables SRP advertise)                      |
| `T` | Force a fresh AVDECC TalkerAdvertise burst                              |
| `b` | Benchmark — 1-second main-loop iteration and event-rate window          |
| `D` | DAC diagnostics — I2S sample count, underruns                           |
| `r` | Reboot CPU                                                              |

---

## 8. How we got here (the journey)

This is the engineering log of how the current build came to work, so the
next round of changes doesn't undo years of debugging.

### Phase 1 — gPTP baseline (commits `fce7fe0` … `1ad6102`)

RGMII bring-up on the B50612D PHY needed a **nibble-swap** in the LiteEth
s7rgmii RX IDDR (`o_Q1`↔`o_Q2`) — without it the SFD `0xD5` decodes as
`0x5D` and every frame fails preamble. PHY power-down testing pinned the
cabled RJ45 to U9 / PHY1 (MDIO address 1). gPTP locks ±30 ns; the LiteEth TSU
needed a shift-add tree replacement for `full_addend * 1e9` so nextpnr-xilinx
could route the multiplier without infering an unrouteable DSP cascade.

### Phase 2 — AVB stack (commits `afba966` … `957d80f`)

CRF parser + MCR PI NCO; AAF 8ch RX with per-channel jitter buffer; full
AVDECC AEM tree with the Milan-compliant `MEDIA_CLOCK_SINK` / `AEM_SUPPORTED`
capability bits. Many small interop fixes were needed before Hive would
patch us:

- The cd-bit in ADP/AECP/ACMP byte 0 must be `0xFA/0xFB/0xFC`, not `0x7A/…`
  (`feedback_avdecc_cd_bit` memory).
- MSRP `AttrListLen` must include the inner EndMark — off-by-2 makes
  bridges drop everything but the first attribute.
- The applicant must emit `MRPDU_NEW(0)` for the first two cycles after
  attach, then `JoinIn(1)` — `JoinMt(3)` forever makes the bridge bounce
  `TalkerFailed` back at the talker.
- AECP `SET_STREAM_FORMAT` must be **permissive** on the listener (accept
  any input format); strict on output.
- The listener must declare `READY` at `srp_listener_enable` time, not
  `AskingFailed` — Auvitran/MOTU otherwise refuse to stream.
- AVTP RX dispatcher must drain *all* pending slots per call. One-shot
  drains lose 99 %+ of CRF/MSRP frames under load.

### Phase 3 — USB UAC2 (avb-usb-host project)

Started as a *separate* standalone Amaranth+LUNA project on the same board.
Many false leads — see `project_luna_openxc7_status` memory for the long
form. The keys that finally worked:

1. **USB_PHASE=0** (PLLE2_ADV no shift). VCO must be ≥ 800 MHz on the -1
   Artix-7 part — MULT=16, CLKOUT0_DIVIDE=16, VCO=960 (`artix7-pll-vco-minimum`).
2. **SAMPLE_EDGE=off** — feed the ultraembedded ULPI wrapper the *raw* ULPI
   input pins, no extra resampling layer.
3. **Startup reset pulse for the wrapper** — `ResetSignal("usb")` is never
   asserted, so the wrapper never kicked off its register-config sequence.
   Pulse `i_ulpi_rst_i` high for ~64 cycles at boot, then release.
4. **PHY RESET pin is active-HIGH** on the USB3300 (`usb3300-rst-polarity`).
5. **Twisted-pair wiring** with each critical signal paired with **its own**
   GND, never another signal (`ulpi-twisted-pair-wiring`).
6. **`interface.claim`** must be asserted in custom handler branches or
   LUNA's request multiplexer routes every UAC2 class request to the fallback
   stall handler.

### Phase 3.2 — integrate USB into avb-aes3 (commits `e9fb257` … `7bff87c`)

The Verilog wrapper from avb-usb-host (`rtl/generated/usb_avb_subsystem.v` +
the ultraembedded ULPI wrapper) drops into avb-aes3 as a `Migen.Instance`
with a `TSTriple` for the ULPI data bus. Elaborated cleanly first try.

But: with the full SoC, `eth_tx_clk` collapsed from 131 MHz (no USB) to
~112 MHz (with USB) — failing the 125 MHz gigabit RGMII target. A seed sweep
(1..30) gave 92-112 MHz across the board; no seed cleared 125. **A
floorplan was added** (`floorplan_usb.py`) confining the USB block to
`X >= 78` (right half of the die) to keep the eth TX cluster's region clear.
With floorplan + seed sweep, seed 4 cleared 134 MHz.

This is where commit `675fb9e` landed — `Gigabit USB+AVB: TX-only
sys-datapath fix + USB floorplan, seed 4`. On the *specific* build verified
("fp_build5"), USB enumerated `1209:eab1` and AVB was patchable in Hive.

### The trap — the lucky bitstream got overwritten

The fp_build5 `.bit` was overwritten by a subsequent `--build`. Source was
in git, but yosys synthesis is non-deterministic on this toolchain — rebuilds
landed *different* placements, most of which failed USB enumeration with
`error -71` (HS device-not-accepting-address). AVB was unaffected (separate
clock domain, separate side of chip).

This kicked off a brutal multi-session debugging round chasing the wrong
mechanism:

- The "fix" sitting in memory was to add `Misc("IOB=TRUE")` to `ulpi_io()`.
  But the working *standalone* `top.fasm` has **0 IFF cells** — meaning
  `Attrs(IOB="TRUE")` doesn't actually pack input flops into IOBs on
  Amaranth/yosys/nextpnr-xilinx. The theory was wrong.
- An IDELAY-based sweep (firmware-tunable `IDELAYE2` taps on the ULPI inputs)
  was built but never validated — the host port disables after a few
  `error -71` retries, making it impossible to sweep the eye-tap in real time.

### The actual root cause (commit `9bdae1b`, 2026-05-28)

**The floorplan was the bug.** It pinned the USB block to `X = 78..114`
(right edge of the die), while the **ULPI input pins are at `X = 1`** (left
edge). The wrapper's first-stage sampling FFs were being forced ~77 columns
of routing away from where they were sampling — guaranteed long propagation,
60 MHz ULPI setup time blown on most placements.

The standalone build, with no floorplan, lets the wrapper land naturally near
the IO. That's why it just worked.

And critically: **`LITEETH_PATCHES.md` #3** (the TX-only sys-datapath patch,
commit `7bff87c`) had already made `eth_tx_clk` robust across seeds — no
floorplan needed for eth anymore. The floorplan kept being applied because
it was on by default in `avb_soc.py`.

**The fix is one line of intent**: flip `--no-floorplan` (opt-out) to
`--floorplan` (opt-in), default OFF. eth_tx still clears 150 MHz comfortably,
USB cells land near the ULPI pins, both work on one bitstream, robust
across rebuilds.

Same day: `bitstreams/archive.sh` was added so we can't lose another
working build to an overwrite.

### Today

```
9bdae1b floorplan off by default: it was breaking USB ULPI sampling
336fce4 tooling: bitstream archive script with git-log sidecar
675fb9e Gigabit USB+AVB: TX-only sys-datapath fix + USB floorplan, seed 4
```

USB + AVB coexist on one bitstream; verified working build archived to
`bitstreams/`. Next focuses: USB ↔ AVB bridge (Phase 3.3), MCR-driven I2S
TX clocking, and (when scale demands it) moving CPU RAM from BRAM to the
on-board SDRAM via LiteDRAM to free ~190 KB BRAM for audio FIFOs.

---

## 9. Project layout

```
avb-aes3/
├── avb_soc.py                   # LiteX SoC top-level (Python/Migen)
├── floorplan_usb.py             # nextpnr --pre-place hook (opt-in via --floorplan)
├── LITEETH_PATCHES.md           # The 3 in-tree LiteEth patches (#3 is THE fix)
├── BENCHMARK_BASELINE.md        # Stage-0 firmware-on-audio baseline (2026-05-21)
├── bitstreams/
│   ├── archive.sh               # Snapshot tool — run after every working build
│   └── .gitignore               # (*.bit and *.info kept locally, not in git)
├── rtl/
│   ├── aes3_rx.v / aes3_tx.v    # AES3 RX/TX (biphase-mark + DPLL)
│   ├── i2s_tx.v                 # I2S transmitter (PCM5102A)
│   └── generated/
│       └── usb_avb_subsystem.v  # USB UAC2 sink, generated by avb-usb-host
├── firmware/                    # Bare-metal C running on VexRiscv
│   ├── main.c                   # Entry, RX dispatch, UART CLI
│   ├── gptp.[ch]                # IEEE 802.1AS gPTP slave + PI servo
│   ├── avtp.[ch]                # IEEE 1722 AVTP
│   ├── srp.[ch]                 # IEEE 802.1Q SRP/MRP
│   ├── avdecc.[ch]              # IEEE 1722.1 AVDECC (ADP + ACMP + AECP + MVU)
│   ├── mcr.[ch]                 # Media-clock recovery (CRF → NCO PI servo)
│   ├── aaf.[ch]                 # AAF 8ch RX with jitter buffer + 8ch TX
│   └── aes3.[ch]                # AES3 ↔ AVTP bridge
└── build/                       # Generated; gitignored
```

LiteEth lives at `/home/lisp/litex/liteeth/`, branch
`colorlight-i9plus-b50612d-fixes` — pair with this repo's HEAD.

---

## 10. Related projects

- **avb-usb-host** (`/home/lisp/FPGA/avb-usb-host`) — the standalone
  Amaranth/LUNA USB UAC2 device. Its
  `rtl/generated/usb_avb_subsystem.v` is what we instance here.
- **Colorlight-FPGA-Projects** (`/home/lisp/FPGA/Colorlight-FPGA-Projects`) —
  i9plus board pinout, ch347.cfg, unlock_flash bitstream.
- **openXC7** (`/home/lisp/openxc7/`) — yosys + nextpnr-xilinx + prjxray-db
  used for synthesis.
