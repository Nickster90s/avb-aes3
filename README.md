# AVB-AES3 Endpoint — Colorlight i9plus v6.1

Standalone FPGA AVB endpoint with AES3 digital audio I/O and PCM5102A I2S DAC.
Runs entirely on the FPGA: VexRiscv + LiteEth + gPTP + AVTP + AVDECC + SRP. No host PC needed at runtime.

---

## 1. Hardware

### 1.1 Board

- **Colorlight i9plus v6.1** (Xilinx XC7A50T-FGG484)
- Board pinout: `/home/lisp/FPGA/Colorlight-FPGA-Projects/colorlight_i9plus_v6.1.md`

### 1.2 Programmer / UART (Ext-Board with CH347T)

The CH347T does JTAG **and** UART over a single USB-C connector. Both appear on `/dev/ttyACM0`.

| CH347T pin | Signal | FPGA pin | Direction      |
|------------|--------|----------|----------------|
| TXD1       | UART   | M3       | CH347 → FPGA   |
| RXD1       | UART   | R3       | FPGA → CH347   |
| TCK/TMS/TDI/TDO | JTAG | (Ext-Board JTAG header) | bidirectional |

### 1.3 PCM5102A I2S DAC (optional — for monitoring)

Connect the PCM5102A breakout to the SODIMM connector. Breakout labels vary —
this board uses `SCK BCK DIN LCK GND VIN`:

| DAC label | FPGA pin | SODIMM pin | Notes                              |
|-----------|----------|------------|------------------------------------|
| VIN       | 3V3      | —          | 3.3 V power                        |
| GND       | GND      | —          | Ground                             |
| SCK       | **GND**  | —          | Master-clock input — tie LOW so PCM5102A uses its internal PLL |
| BCK       | U7       | 46         | 3.072 MHz bit clock                |
| LCK       | U6       | 48         | 48 kHz word select (= LRCK)        |
| DIN       | U5       | 50         | Serial audio data                  |

### 1.4 AES3 I/O (optional — for digital audio in/out)

| Signal     | FPGA pin | SODIMM pin | External hardware                        |
|------------|----------|------------|------------------------------------------|
| AES3 OUT   | P5       | 42         | RS-422 driver (AM26LS31) → 110 Ω xfmr → XLR-3M |
| AES3 IN    | T6       | 44         | XLR-3F → 110 Ω xfmr → RS-422 receiver (AM26LS32) |

### 1.5 Ethernet (required for AVB)

PHY0 (U5 on board) is RGMII — connect a magjack to the AVB network.
Pin assignments (RGMII): see `colorlight_i9plus_v6.1.md` (GTXCLK=A1, RXC=H4, MDIO/MDC=G2/G1, etc.).

---

## 2. Build

### 2.1 Environment

```sh
source /home/lisp/FPGA/env.sh
```

This activates the LiteX venv and points to the openXC7 toolchain.

### 2.2 Build SoC + firmware

```sh
cd /home/lisp/FPGA/avb-aes3
python3 avb_soc.py
```

Produces:
- `build/colorlight_i9plus/gateware/colorlight_i9plus.bit` — FPGA bitstream
- `build/colorlight_i9plus/software/firmware/firmware.bin` — VexRiscv firmware (linked into the bitstream's BRAM)

To rebuild only firmware after editing `firmware/*.c`:

```sh
cd /home/lisp/FPGA/avb-aes3/firmware
make
# then re-run python3 avb_soc.py to bake new firmware into the bitstream
```

---

## 3. Programming

### 3.1 Volatile (SRAM — disappears on power cycle)

Fastest for development. Bitstream stays until power-off.

```sh
sudo /home/lisp/openocd/src/openocd \
  -s /home/lisp/openocd/tcl \
  -f /home/lisp/FPGA/Colorlight-FPGA-Projects/tools/ch347.cfg \
  -c "init; pld load 0 /home/lisp/FPGA/avb-aes3/build/colorlight_i9plus/gateware/colorlight_i9plus.bit;" \
  -c exit
```

### 3.2 Persistent (SPI flash — survives reboot)

Two-step process. The MX25L128 flash is write-protected from the factory.

**Step 1 — Unlock flash (only first time per board):**

```sh
sudo /home/lisp/openocd/src/openocd \
  -s /home/lisp/openocd/tcl \
  -f /home/lisp/FPGA/Colorlight-FPGA-Projects/tools/ch347.cfg \
  -c "init; pld load 0 /home/lisp/FPGA/Colorlight-FPGA-Projects/tools/unlock_flash_xc7a50t.bit;" \
  -c exit
```

**Step 2 — Write bitstream to flash and reboot:**

```sh
TOOLS=/home/lisp/FPGA/Colorlight-FPGA-Projects/tools
BIT=/home/lisp/FPGA/avb-aes3/build/colorlight_i9plus/gateware/colorlight_i9plus.bit
sudo /home/lisp/openocd/src/openocd \
  -s /home/lisp/openocd/tcl \
  -f $TOOLS/ch347.cfg -c "
    set XC7_JSHUTDOWN 0x0d
    set XC7_JPROGRAM 0x0b
    set XC7_BYPASS 0x3f
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

After flashing, the FPGA boots from flash on every power-up.

---

## 4. Terminal (UART)

Connect the Ext-Board USB-C → host PC. The CH347T enumerates as `/dev/ttyACM0`.

### 4.1 Open serial console

```sh
sudo picocom -b 115200 /dev/ttyACM0
```

Exit picocom: **Ctrl-A Ctrl-X**

Alternative: `sudo screen /dev/ttyACM0 115200` (exit: Ctrl-A k y).

### 4.2 Boot output

After reset you should see:
```
========================================
  AVB-AES3 Firmware
  Built <date> <time>
  Clock: 50 MHz
========================================
[main] Press 'h' for commands.
```

### 4.3 Commands

| Key | Action                                                    |
|-----|-----------------------------------------------------------|
| `h` | Show command help                                         |
| `s` | Status — gPTP, AVTP, AES3, SRP, AVDECC counters & state   |
| `t` | Toggle AVB **talker** (also enables/disables SRP advertise) |
| `l` | Toggle AVB **listener** (loops own stream ID, for self-test) |
| `d` | Toggle 1 kHz sine test tone on PCM5102A DAC               |
| `r` | Reboot CPU                                                |

### 4.4 Example status output

```
[gPTP] state=2 time=12345.678901234
  sync=42 pdelay=10 offset=-12 ns delay=850 ns
  addend=2147483648 locked=1
[AVTP TX] pkts=8000 ring=0
[AVTP RX] pkts=8000 seq_err=0 ring=0
[AES3] locked=1 rx_samples=384000 tx_samples=384000 overrun=0 underrun=0
[SRP] tx=1 rx=1 domain=1 talker_reg=1
[AVDECC] adp_tx=20 acmp=2/2 aecp=5/5 talker=1 listener=0
```

**Healthy signs:**
- `gPTP servo_locked=1`, `offset` < 1000 ns
- `AES3 locked=1` (when AES3 source connected)
- `AVTP TX/RX` counters incrementing when streaming
- `SRP domain=1, talker_reg=1` when network has SRP-aware switch

---

## 5. Project layout

```
avb-aes3/
├── avb_soc.py              # LiteX SoC top-level (Python)
├── rtl/
│   ├── aes3_rx.v           # AES3 receiver (biphase-mark + DPLL)
│   ├── aes3_tx.v           # AES3 transmitter
│   └── i2s_tx.v            # I2S transmitter for PCM5102A
├── firmware/               # Bare-metal C running on VexRiscv
│   ├── main.c              # Entry point, RX dispatch, UART CLI
│   ├── gptp.[ch]           # IEEE 802.1AS gPTP slave
│   ├── avtp.[ch]           # IEEE 1722 AVTP / IEC 61883-6 AM824
│   ├── srp.[ch]            # IEEE 802.1Q SRP/MRP
│   ├── avdecc.[ch]         # IEEE 1722.1 AVDECC (ADP + ACMP + AECP)
│   └── aes3.[ch]           # AES3 ↔ AVTP ring-buffer bridge
└── build/                  # Generated by `python3 avb_soc.py`
```

---

## 6. Quick reference

| Task                          | Command                                                |
|-------------------------------|--------------------------------------------------------|
| Setup env                     | `source /home/lisp/FPGA/env.sh`                        |
| Build everything              | `cd /home/lisp/FPGA/avb-aes3 && python3 avb_soc.py`    |
| Load to SRAM (volatile)       | See §3.1                                               |
| Flash to SPI (persistent)     | See §3.2                                               |
| Open console                  | `sudo picocom -b 115200 /dev/ttyACM0`                  |
| Status                        | Press `s` in the console                               |
| Test DAC                      | Press `d` (toggle 1 kHz sine)                          |
