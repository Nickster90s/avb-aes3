# Toolchain — versions, paths, and how to reproduce

Recorded 2026-05-28. This is the environment the working bitstreams in
`bitstreams/` were built against. Verify any of this with `git log` /
`pip show` / etc. on your own machine if drifted.

---

## 1. Open Xilinx synthesis toolchain (`openXC7`)

| Component | Location | Version / state | Source |
|---|---|---|---|
| openXC7 snap | `/home/lisp/openxc7/openxc7_0.8.2_amd64.snap` (169 MB) | 0.8.2 (Jun 2024 release) | https://github.com/openXC7/toolchain-installer |
| Extracted tree | `/home/lisp/openxc7/openxc7/` | stock, no local mods | (snap unpack) |
| yosys binary | `/home/lisp/openxc7/bin/yosys` | from snap | stock |
| nextpnr-xilinx | `/home/lisp/openxc7/bin/nextpnr-xilinx` | from snap | stock |
| fasm2frames / xc7frames2bit | `/home/lisp/openxc7/bin/` | from snap | stock |
| prjxray-db | `/home/lisp/openxc7/openxc7/opt/nextpnr-xilinx/external/prjxray-db/` | shipped inside snap | stock |
| chipdb (xc7a50tfgg484) | `/home/lisp/FPGA/demo-projects/chipdb/xc7a50tfgg484-1.bin` (88 MB, Oct 2024) | pre-built | https://github.com/openXC7/demo-projects |

**No local patches.** openXC7 is consumed as released — we have not
touched yosys, nextpnr, the chipdb generator, or any wrapper script.

**Required env vars** (used by `python3 avb_soc.py --build`):

```sh
export CHIPDB=/home/lisp/FPGA/demo-projects/chipdb
export PRJXRAY_DB_DIR=/home/lisp/openxc7/openxc7/opt/nextpnr-xilinx/external/prjxray-db
```

---

## 2. LiteX / Migen / LiteEth (LiteX SoC framework)

LiteX is the Python SoC framework that hosts the VexRiscv CPU, LiteEth
MAC, MCR NCO, MCRI2STx, CRF extractor, and the USB block instance. All
checkouts live under `/home/lisp/litex/`.

| Repo | Path | Branch | HEAD (2026-05-28) | Patches |
|---|---|---|---|---|
| litex | `/home/lisp/litex/litex` | `master` | `f377764d7` | none |
| litex-boards | `/home/lisp/litex/litex-boards` | tracked | (stock) | none |
| migen | `/home/lisp/litex/migen` | (detached) | `4c2ae8d` | none |
| **liteeth** | `/home/lisp/litex/liteeth` | **`colorlight-i9plus-b50612d-fixes`** | `0907416` | **3 in-tree patches — see `LITEETH_PATCHES.md` in this repo** |
| litedram | `/home/lisp/litex/litedram` | tracked | (stock) | none |
| pythondata-cpu-vexriscv | `/home/lisp/litex/pythondata-cpu-vexriscv` | tracked | (stock) | none |
| pythondata-software-picolibc | `/home/lisp/litex/pythondata-software-picolibc` | tracked | (stock) | none |
| pythondata-software-compiler_rt | `/home/lisp/litex/pythondata-software-compiler_rt` | tracked | (stock) | none |

### LiteEth patches we maintain

See `LITEETH_PATCHES.md` for full detail. Summary:

1. **`core/ptp.py`** — TSU 52×30 addend multiplier rewritten as a
   shift-add tree so nextpnr-xilinx can route it (the natural form
   infers a DSP48 cascade that fails routing on XC7A50T).
2. **`mac/core.py`** — TX skid buffer before the CDC (marginal win,
   superseded by #3 but harmless).
3. **`mac/core.py`** — ★ **`with_sys_datapath_tx`** (TX-only sys-domain
   datapath). Moves CRC/preamble/padding into sys clock, leaving
   eth_tx_clk carrying only the converter. Drops the eth_tx critical
   path from 8.2 ns routing to under 6 ns; eth_tx 114 → 163 MHz robust
   across seeds. **The fix.** RX path is left bit-identical so the
   gPTP RX-timestamp ring is untouched.

These patches live on the `colorlight-i9plus-b50612d-fixes` branch.
Pair this repo's HEAD with that liteeth HEAD. Drift either side and
gigabit eth_tx returns to the timing knife-edge.

### Python source discovery (no venv activation needed)

A single `.pth` file makes all the LiteX checkouts importable system-wide:

```
$ cat ~/.local/lib/python3.11/site-packages/litex-tools.pth
/home/lisp/litex/venv/lib/python3.11/site-packages
/home/lisp/litex/litex
/home/lisp/litex/litex-boards
/home/lisp/litex/liteeth
/home/lisp/litex/litedram
… (etc.)
```

That's why `python3 avb_soc.py --build` Just Works without `source
venv/bin/activate`. See memory `reference_python_env_pth`.

---

## 3. Amaranth + LUNA (USB UAC2 device gateware)

The USB block instanced into `avb_soc.py` (`Instance("usb_avb_subsystem",
…)`) is *generated* from an Amaranth/LUNA project — **avb-usb-host** —
and dropped in as plain Verilog. We don't run Amaranth at avb-aes3 build
time; only when we regenerate the `usb_avb_subsystem.v` blackbox.

| Component | Location | Version | Source |
|---|---|---|---|
| amaranth | `/home/lisp/litex/venv/lib/python3.11/site-packages/amaranth/` | `0.5.8` (pip-installed in `litex/venv`) | stock |
| luna | `/home/lisp/litex/venv/lib/python3.11/site-packages/luna/` | (pip-installed) | stock |
| usb_protocol | `/home/lisp/litex/venv/lib/python3.11/site-packages/usb_protocol/` | (pip-installed) | stock |
| avb-usb-host | `/home/lisp/FPGA/avb-usb-host` | `master` @ `7895049` | OUR project (not yet pushed) |
| ultraembedded `ulpi_wrapper.v` | `avb-usb-host/rtl/ulpi_ultraembedded/ulpi_wrapper.v` (also copied into `avb-aes3/rtl/`) | from upstream `ultraembedded/core_ulpi_wrapper` | vendored |

### Amaranth-on-openXC7 quirks (memory: `feedback_amaranth_xray_toolchain`)

Stock Amaranth's `XilinxPlatform` assumes Vivado. To target our open
toolchain we override **four** things in
`avb-usb-host/gateware/colorlight_i9plus_platform.py`:

1. `__init__(toolchain="Xray")` — selects yosys + nextpnr-xilinx +
   fasm2frames templates.
2. `vendor_toolchain` property returns `True` — without this, the
   generic `Platform.get_input_output` path rejects `Attrs(IOSTANDARD=…)`
   on bidirectional pins like the ULPI data lines.
3. `_xray_device` returns `f"{self.device}{self.package}"` — Amaranth's
   default returns just family (`xc7a50t`), but the chipdb is
   per-package (`xc7a50tfgg484.bin`).
4. PLLE2_ADV must use `p_COMPENSATION="INTERNAL"` (not Vivado's
   default `"ZHOLD"`, which crashes nextpnr-xilinx at fasm legalisation,
   `fasm.cc:1572`).

Without all four, the build either won't elaborate or won't fasm-legalise.

### USB working recipe (avb-usb-host, memory: `project_luna_openxc7_status`)

After much debugging, the standalone USB UAC2 HS sink finally works on
the openXC7 + USB3300 stack. The recipe (build with `USB_PHASE=0
SAMPLE_EDGE=off python3 usb_utmi_top.py`):

1. **USB_PHASE=0** — PLL with no shift (proven by `ulpi_phasescan_top.py`
   eye-scan).
2. **SAMPLE_EDGE=off** — wrapper fed raw ULPI pins; no extra resampling
   layer.
3. **Wrapper startup reset pulse** — hold `i_ulpi_rst_i` high ~64 usb
   cycles after boot then release; without it the wrapper never starts
   its register-config sequence.
4. **USB3300 RESET is active-HIGH** (datasheet pin 9) — use `Pins(...)`
   not `PinsN(...)`. See memory `usb3300-rst-polarity`.
5. **Wiring** — each timing-critical signal twisted with **its own**
   GND, never another signal. See memory `ulpi-twisted-pair-wiring`.
6. **`interface.claim`** must be asserted in custom request-handler
   branches or LUNA's request multiplexer stalls every UAC2 class
   request.

### Regenerating `usb_avb_subsystem.v`

From `avb-usb-host`:

```sh
cd /home/lisp/FPGA/avb-usb-host/gateware
USB_PHASE=0 SAMPLE_EDGE=off python3 usb_utmi_top.py
# → produces rtl/generated/usb_avb_subsystem.v
cp rtl/generated/usb_avb_subsystem.v /home/lisp/FPGA/avb-aes3/rtl/
```

Don't regenerate on every build — only when the USB block itself needs
to change (e.g. adding an internal FIFO for #67, see open items in
README §8).

---

## 4. Cross-project Verilog dropped into `avb-aes3/rtl/`

| File | Source | Purpose |
|---|---|---|
| `rtl/usb_avb_subsystem.v` | generated from avb-usb-host | USB UAC2 HS device + channel stream + feedback |
| `rtl/ulpi_wrapper.v` | `ultraembedded/core_ulpi_wrapper` | ULPI↔UTMI bridge, instanced by `usb_avb_subsystem` |
| `rtl/aes3_rx.v` / `aes3_tx.v` | OUR — see git history | AES3 biphase-mark RX/TX |
| `rtl/i2s_tx.v` | OUR — see git history | (Obsolete since `7199108`: superseded by Migen `MCRI2STx`; file kept for reference, not instanced) |
| `rtl/rgmii_var_delay.py` | OUR | Variable-delay RGMII PHY wrapper |

---

## 5. Programming the board (CH347T)

| Tool | Path | Purpose |
|---|---|---|
| openocd | `/home/lisp/openocd/src/openocd` | JTAG + SPI flash via the CH347T. NOT the distro openocd — needs the CH347 patch. |
| ch347.cfg | `/home/lisp/FPGA/Colorlight-FPGA-Projects/tools/ch347.cfg` | openocd config for the i9plus Ext-Board's CH347T |
| unlock_flash_xc7a50t.bit | `…/Colorlight-FPGA-Projects/tools/` | SPI flash unlock helper (one-shot per board) |
| bscan_spi_xc7a50t.bit | `…/Colorlight-FPGA-Projects/tools/` | BSCAN-SPI proxy bitstream for `flash write_image` |

`openFPGALoader` is **NOT** installed — see memory `reference_i9plus_load`.

---

## 6. Verifying / reproducing on a fresh machine

```sh
# 1. openXC7 toolchain
sudo snap install --dangerous openxc7_0.8.2_amd64.snap     # from openXC7 releases
# (sets up yosys, nextpnr-xilinx, prjxray-db, fasm tools)

# 2. chipdb
git clone https://github.com/openXC7/demo-projects ~/FPGA/demo-projects
# → contains chipdb/xc7a50tfgg484-1.bin

# 3. LiteX (paths must match the .pth recipe)
mkdir -p ~/litex && cd ~/litex
git clone https://github.com/enjoy-digital/litex
git clone https://github.com/enjoy-digital/litex-boards
git clone https://github.com/m-labs/migen
git clone https://github.com/enjoy-digital/liteeth
git clone https://github.com/enjoy-digital/litedram
git clone https://github.com/litex-hub/pythondata-cpu-vexriscv
git clone https://github.com/litex-hub/pythondata-software-picolibc
git clone https://github.com/litex-hub/pythondata-software-compiler_rt
# … plus the other pythondata-* repos listed above

# 4. Apply OUR liteeth patches
cd ~/litex/liteeth
git checkout -b colorlight-i9plus-b50612d-fixes
# Then apply the 3 patches from LITEETH_PATCHES.md, or:
git remote add nick git@github.com:Nickster90s/liteeth.git    # if mirrored
git pull nick colorlight-i9plus-b50612d-fixes

# 5. .pth so imports Just Work without venv activation
mkdir -p ~/.local/lib/python3.11/site-packages
cat > ~/.local/lib/python3.11/site-packages/litex-tools.pth <<'EOF'
/home/lisp/litex/venv/lib/python3.11/site-packages
/home/lisp/litex/litex
/home/lisp/litex/litex-boards
/home/lisp/litex/liteeth
/home/lisp/litex/litedram
/home/lisp/litex/pythondata-software-picolibc
/home/lisp/litex/pythondata-software-compiler_rt
/home/lisp/litex/pythondata-cpu-vexriscv
EOF

# 6. amaranth + luna (only needed to regenerate USB Verilog)
python3 -m venv ~/litex/venv
~/litex/venv/bin/pip install amaranth==0.5.8 luna usb-protocol

# 7. avb-aes3 itself
git clone https://github.com/Nickster90s/avb-aes3 ~/FPGA/avb-aes3

# 8. (Optional, only to regenerate USB block) avb-usb-host
# git clone <when_pushed> ~/FPGA/avb-usb-host

# 9. Build
cd ~/FPGA/avb-aes3
export CHIPDB=/home/lisp/FPGA/demo-projects/chipdb
export PRJXRAY_DB_DIR=/home/lisp/openxc7/openxc7/opt/nextpnr-xilinx/external/prjxray-db
( cd firmware && make clean && make )
python3 avb_soc.py --build --firmware firmware/firmware.bin --seed 4
```

The yosys synthesis is non-deterministic, so the bitstream you get
won't be byte-identical to those in `bitstreams/`. The archived
`.bit` + `.info` pairs are the authoritative reference for any
specific commit.

---

## 7. Related repos / external tools (not patched, but consumed)

| Repo | Purpose |
|---|---|
| `ultraembedded/core_ulpi_wrapper` | ULPI↔UTMI Verilog bridge (vendored in `rtl/`) |
| `enjoy-digital/litex_buildenv` | Reference for `--seed` / `--floorplan` Builder hooks |
| `jdksavdecc-c` | AEM descriptor IDs (memory `feedback_aem_descriptor_ids`) |
| `genavb-stack` (Freescale/NXP) | MAAP / full MRP reference (memory `project_avb_srp_todo`) |
| `avdecc-endpoint/tools/acmp_connect.c` | Bare-metal AVDECC controller for bench testing (memory `reference_avdecc_endpoint_test_flow`) |

---

## 8. Open follow-ups for this doc

- **avb-usb-host** is now pushed: https://github.com/Nickster90s/avb-usb-host (proven-good `top.bit` archived under LFS at commit `a9d701e`). Mirror the LiteEth `colorlight-i9plus-b50612d-fixes` branch when convenient so step 4 of §6 doesn't depend on local-only patches.
  should link to it (and the LiteEth patches branch should be mirrored
  too so step 4 above doesn't require local-only patches).
- The 4× `pythondata-*` repos are version-sensitive in subtle ways; if
  builds drift, lock to commits and document.
- The dev box has additional installs (picocom, riscv toolchain, etc.)
  that aren't documented here but are listed in shell history — see
  the OS-level package state if reproducing fully.
