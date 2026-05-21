# Stage 0 Baseline — Firmware-on-Audio Architecture

Tag: `audio-in-firmware-baseline`
Captured: 2026-05-21 (UART `b` command, 1-second rate windows)

## Hardware / build

- Colorlight i9plus v6.1 (Xilinx XC7A50T)
- LiteX SoC, VexRiscv @ 50 MHz
- LiteEth RGMII with B50612D PHY (nibble-swap patch in s7rgmii.py)
- nrxslots=2, ntxslots=2 (>2 silently breaks TX)
- seed=23 (post-clean-rebuild placement)

## Three operating points

### Idle (no streams patched)

```
main_loop: 4083 iter/s  avg 244823 ns  max 798687 ns
RX:        0 writer_err/s  0 aaf/s  0 crf/s
AVDECC:    0 aecp_rx/s  0 aecp_tx/s
gPTP:      8 sync_rx/s  1 pdresp_rx/s
DAC:       0 samples/s  47981 underruns/s
```

Main loop already only at **4 kHz** with zero traffic. Each iteration takes
~245 µs. The 2-slot RX FIFO depth (250 µs at AAF rate) is *matched* to this
loop rate — there's zero margin for any extra work.

### CRF only patched

```
main_loop: 3810 iter/s  avg 262417 ns  max 809638 ns
RX:        0 writer_err/s  0 aaf/s  500 crf/s
AVDECC:    0 aecp_rx/s  0 aecp_tx/s
gPTP:      8 sync_rx/s  1 pdresp_rx/s
DAC:       0 samples/s  47982 underruns/s
```

CRF (1000 fps × 2 ms ≈ 500/s here) costs ~7% of the main loop rate. Still
zero writer_errors. System healthy.

### CRF + AAF in patched (peak overload)

```
main_loop: 1969 iter/s  avg 507657 ns  max 791057 ns
RX:        3435 writer_err/s  4570 aaf/s  0 crf/s
AVDECC:    0 aecp_rx/s  0 aecp_tx/s
gPTP:      0 sync_rx/s  1 pdresp_rx/s
DAC:       27420 samples/s  20555 underruns/s
```

**Main loop collapses to half rate.** AAF arrives at 8000 fps; we process
4570 and drop 3435 at the MAC FIFO. CRF, gPTP Sync, AECP — all starved.
Within ~30 seconds the bridge times out the talker and the audio path dies.

## Numbers to beat at each subsequent stage

| metric | Stage 0 baseline | Stage N target (gateware) |
|---|---|---|
| main_loop iter/s under AAF | 1969 | > 10000 |
| writer_err/s under AAF | 3435 | 0 |
| aaf/s seen by firmware | 4570 | 0 (gateware extracts) |
| crf/s under AAF flood | 0 | 1000 |
| gPTP sync_rx/s under AAF | 0 | 8 |
| DAC samples/s | 27420 | exactly 48000 |
| DAC underruns/s | 20555 | 0 |
| Max # of concurrent streams | 1 | 16 |

## Bench command

```
b   # UART command in firmware — prints last 1-second window
```

Each Stage gate should record its own copy of all three operating points
in a section below this one.
