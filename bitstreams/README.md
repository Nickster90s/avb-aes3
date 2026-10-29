# bitstreams/

Why this folder is structured: `--build` always writes the **same fixed path**,
`build/colorlight_i9plus/gateware/colorlight_i9plus.bit`. Every build silently
destroys the previous one. That has already cost this project two verified
bitstreams (the 14:07 breakout-USB1 build, and the 16:40 breakout-USB2 build),
each of which had to be rebuilt from scratch to re-establish a reference.

## Layout

```
bitstreams/
  known-good/          curated symlinks -> builds VERIFIED on hardware
  <board>/<phy>/       every archived build, newest name sorts last
  _legacy/             everything from before this convention (untouched)
  archive.sh           the only thing that should write here
```

`<board>` is `ext` or `breakout`; `<phy>` is `usb1` or `usb2`. Both are derived
from the **same env vars that select the pins** (`BOARD`, `USB_BACKUP`), so a
bitstream cannot be filed under a PHY it was not built for.

Filenames: `YYYY-MM-DD_HHMM_<sha>[-dirty][_label].bit` — date first so plain
`ls` is chronological, plus a `.info` sidecar and optionally a `.log`.

## Reading it

```sh
ls known-good/                              # what actually works
ls breakout/usb2/                           # every USB2 attempt, oldest first
cat breakout/usb1/*_breakout-usb1-avb.info  # how that one was built
```

## Writing it

Builds self-archive — `build.sh` calls `archive.sh` on success, so an
un-archived build is no longer possible. Name and promote a good one:

```sh
ARCHIVE_LABEL=usb2-first-light BOARD=breakout USB_BACKUP=1 ./build.sh
BOARD=breakout USB_BACKUP=1 bitstreams/archive.sh usb2-first-light --good
```

`--good` creates `known-good/<board>-<phy>-<label>.bit`. Promote **only** after
the hardware confirms it (enumeration, ALSA card, gPTP lock — whatever the build
was for). `NO_ARCHIVE=1 ./build.sh` skips archiving for a throwaway.

`archive.sh` refuses to store a byte-identical duplicate and tells you where the
existing copy is, so re-running it is always safe.

## The `.info` sidecar

Carries a **REBUILD RECIPE** line with the full env (`BOARD`, `USB_BACKUP`,
`USB_PHASE`, floorplan vars, seed args), the env as it actually was, `git log -1`,
the uncommitted diff stat, and Fmax/utilisation pulled from the build log. The
env is the part that was missing before: `BOARD=breakout USB_BACKUP=1` is the
only difference between the USB1 and USB2 bitstreams, and nothing recorded it.

## Pruning

Keep anything in `known-good/` (and its target) and anything with a meaningful
label. Builds auto-archived as `_auto` are the prunable ones:

```sh
find . -name '*_auto.bit' -mtime +30        # review first
```

Never delete a `known-good/` target. `.bit`/`.info` are tracked via git-lfs
(see `.gitattributes`).
