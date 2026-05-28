# Licensing — what's covered, by what, and what it means

Short version: **the original code we wrote is Apache 2.0** (see
`LICENSE`). Some imported files keep their original licenses. See
`NOTICE` for the per-component breakdown.

The two specific things worth understanding before reusing this
project commercially:

## 1. `rtl/ulpi_wrapper.v` is GPL

This file is vendored verbatim from
[ultraembedded/core_ulpi_wrapper](https://github.com/ultraembedded/core_ulpi_wrapper)
and carries the **GNU General Public License, any later version**.
That has consequences if you redistribute a bitstream built from this
repo:

- **Open-source distribution** — fine. The repo as a whole is
  effectively GPL-compatible because everything else (Apache 2.0,
  BSD-2/3-Clause, CERN-OHL-W-2.0) can be combined with GPL
  *downstream*.
- **Commercial / closed product** — the bitstream is a derivative
  work of GPL'd Verilog. Options:
    1. Release your product (firmware + bitstream + any modifications)
       under GPL too. Sometimes that's fine.
    2. Buy a commercial licence from ultraembedded. The file's own
       header hints at this: *"If you would like a version with a more
       permissive license for [commercial use]…"* — they sell one.
    3. Replace `ulpi_wrapper.v` with an MIT/Apache-licensed
       equivalent (or write your own). Possible but non-trivial — see
       README §8 in `avb-usb-host` for what the wrapper does.

If you don't redistribute the bitstream (private/internal use), GPL
doesn't kick in. The obligation only arises on distribution.

## 2. Imported files retain their headers

| File | Upstream | Licence |
|---|---|---|
| `rtl/ulpi_wrapper.v` | ultraembedded/core_ulpi_wrapper | GPL (any later) |
| `rtl/usb_avb_subsystem.v` | generated from avb-usb-host | Apache 2.0 (the *original* parts) + inherited per upstream NOTICE |

Don't strip these headers when modifying. Modifications to
GPL/CERN-OHL files are themselves subject to those terms.

## 3. Apache 2.0 covers everything else

That includes — exhaustively — the LiteX SoC top
(`avb_soc.py`), the firmware (`firmware/*.[ch]`), the AES3
gateware (`rtl/aes3_*.v`), `crf_extractor.py`, `avtp_extractor.py`,
`floorplan_usb.py`, `bitstreams/archive.sh`, the entire documentation
set (`README.md`, `TOOLCHAIN.md`, `LITEETH_PATCHES.md`,
`BENCHMARK_BASELINE.md`).

Apache 2.0 = use, modify, redistribute (open or closed), as long as
you preserve the LICENSE + NOTICE files and don't claim our trademarks.
Includes an explicit patent grant — important in pro-audio / AVB.

## 4. If you adapt this for your own project

- Keep `LICENSE` and `NOTICE` as-is.
- Add your own `Copyright (c)` line for your changes.
- Preserve every SPDX header in upstream files.
- If you remove `ulpi_wrapper.v` (replacing it), update `NOTICE` to
  drop the GPL line and you can then call the result fully Apache 2.0
  (assuming you also haven't pulled in anything else copyleft).

## 5. Trademarks etc.

This project doesn't claim any trademarks. AVB, Milan, USB, UAC2,
AAF, CRF, gPTP are protocol names; Hive is © L-Acoustics; Auvitran,
MOTU, PCM5102A are vendor names. No affiliation implied.

## 6. Want to suggest a different licence?

Open an issue in `github.com/Nickster90s/avb-aes3`. The big things to
change would be:

- Replace `ulpi_wrapper.v` (free the project from GPL).
- Switch the whole project to a copyleft hardware licence
  (CERN-OHL-S-2.0) — this maximises community pressure to keep
  derivatives open, at the cost of some commercial-adoption friction.

Either direction would be a real decision, not a paperwork change.
