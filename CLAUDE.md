# RepRapFirmware — Unlayered3D fork

Fork of Duet3D/RepRapFirmware carrying five-axis kinematics, a Prusa MMU2S driver, per-motor
motion limits and per-motor wear statistics. See
[Developer-documentation/Unlayered-fork.md](Developer-documentation/Unlayered-fork.md) for what
each feature is, which upstream files it touches, and the design notes.

## Workspace layout

This repo is one of several sibling checkouts under `C:\rrf`, and the build reaches out to
them via `WORKSPACE := ..`:

```
C:\rrf\
  RepRapFirmware\   <- this repo
  CoreN2G\  RRFLibraries\  FreeRTOS\  CANlib\  LibTinyusb\  LibMbedTls\
  arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi -> Program Files install (symlink)
```

**All sibling libraries must be on their `3.7-dev` branch** (LibMbedTls uses `master`) to
match this fork's 3.7 branch. A library on the wrong branch produces confusing link errors
rather than a clear failure.

## Building

```bash
make                      # list targets
make test-toolchain       # verify the toolchain resolves
make Duet3Mini5plus       # Duet 3 Mini 5+ (SAME51) -> .bin + .uf2
make Duet3_MB6HC          # Duet 3 MB6HC (SAME70)   -> .bin
make Duet3_MB6XD          # Duet 3 MB6XD (SAME70)
make Duet3_CAN0           # MB6HC over CAN0
make Duet3_MB6HC_no_SD    # MB6HC without SD support
make all                  # everything except CAN0 and no_SD (duplicate output names)
make clean                # remove build outputs
make clean-all            # also clean the sibling libraries
V=1                       # verbose
DEBUG=1                   # -g3 -Og -DDEBUG
```

Output lands in a per-board directory (`Duet3Mini5plus/`, `Duet3_MB6HC/`, ...), all gitignored.
`-j8` is safe and much faster.

The default `CROSS_COMPILE` points at the 13.2 symlink in the workspace root. Override for a
different toolchain: `make Duet3Mini5plus CROSS_COMPILE=/path/to/bin/arm-none-eabi-`.

### Build-system rules

- **Makefiles are the source of truth.** `.cproject`/`.project`/`.settings` remain tracked for
  the Eclipse OpenOCD debug launch configs, but are *not* maintained. Do not hand-edit them
  when adding sources — the makefiles discover sources with `find`, so new files are picked up
  automatically.
- **Never discover sources with find's `! -path <glob>`.** GNU make 4.4.1 on Windows expands
  the globs itself before find sees them, so find aborts and returns nothing — an empty source
  list and undefined references at link time, with no obvious cause. Filter in make with
  `$(foreach)`/`$(findstring)` instead. This applies to the sibling libraries too.
- The editor index is `.clangd` + `compile_commands.json`. `.clangd` documents its own
  `compiledb` regeneration command; rerun it after adding sources or changing include paths.
- If a library archive looks stale (undefined references to symbols that clearly exist), delete
  that library's build directory and rebuild it. Objects inherited from the old Eclipse build
  do not always have working header dependency tracking.

## Bench printers

| IP | Machine | Notes |
|---|---|---|
| 192.168.1.50 | Five-axis (RPPPR) | Duet 3 Mini 5+, custom DWC |
| 192.168.1.60 | MK3-class | Duet 3 Mini 5+, MMU2S via `M1750` |
| 192.168.1.80 | Five-axis (XYZBC) | Duet 3 Mini 5+, differential X/B + Y/C, probe `calibrate*.g`, sensorless homing |

All run `M669 K16` (`coreXBYC2`) where five-axis applies.

### Flashing

```bash
curl -s -m 8 "http://192.168.1.80/rr_connect?password=reprap&time=$(date +%Y-%m-%dT%H:%M:%S)"
curl -s -m 120 -H "X-Session-Key:0" --data-binary @Duet3Mini5plus/Duet3Firmware_Mini5plus.uf2 \
     -H 'Content-Type: application/octet-stream' \
     "http://192.168.1.80/rr_upload?name=0:/firmware/Duet3Firmware_Mini5plus.uf2"
curl -s -m 8 -H "X-Session-Key:0" "http://192.168.1.80/rr_gcode?gcode=M997%20S0"
```

Read state with `rr_model?key=...`, run G-code with `rr_gcode?gcode=...`, and collect the
response with `rr_reply`.

## Custom G/M-codes

| Code | Purpose |
|---|---|
| `G33` | Run `calibrate.g`, the machine calibration macro. Same locking as `G32` |
| `M1750` | MMU2S control — see the fork doc for all subcommands |
| `M201.2 P<motor> S<mm/s^2>` | Per-motor maximum acceleration |
| `M203.2 P<motor> S<mm/min>` | Per-motor maximum feedrate |
| `M205.2 P<motor> S<mm/s>` | Per-motor maximum jerk |
| `M557.1 I<0\|1> Q<mm>` | Height-map interpolation: `I0` bilinear (default), `I1` bicubic. `Q` sets the segmentation chord tolerance; `Q0` restores the fixed 2-segments-per-cell rule |
| `M669 K16 A D R Q X Y U V W B P` | Five-axis geometry. **`Q` sets cRatio, not `S`** — an `S` parameter is silently ignored |
| `M122` | Last diagnostics part reports lifetime/per-motor wear statistics |

## Conventions

Match the surrounding code: hard tabs, Allman braces, `noexcept` on nearly everything,
trailing `// comments` aligned with tabs. Note the build uses `-fsingle-precision-constant`,
so an unsuffixed literal like `0.0` is a **float** — mixing it with a `double` trips
`-Werror=double-promotion`. The build is `-Werror`, so warnings are build failures.

Guard optional features with a `SUPPORT_*` macro defaulted in `src/Config/Pins.h` and enabled
per board in the `Pins_*.h` files, wrapping both the header and the `.cpp`.

When editing files shared with upstream, keep the diff minimal — every touched line is a
future merge conflict. The fork doc lists the current merge surface and why each file is on it.
