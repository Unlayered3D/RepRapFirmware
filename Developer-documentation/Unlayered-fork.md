# The Unlayered3D fork of RepRapFirmware

This document covers what this fork adds on top of Duet3D/RepRapFirmware, which upstream
files it touches and why, and design notes that would otherwise be lost.

- Upstream: `https://github.com/Duet3D/RepRapFirmware` (remote `upstream`)
- Fork: `https://github.com/Unlayered3D/RepRapFirmware` (remote `origin`)
- Working branch: `print-logging-fix`, forked from upstream at `ed7e034c7` (3.7.0-beta.1)

## Features

### 1. Five-axis kinematics — `M669 K15` / `K16` / `K17`

`src/Movement/Kinematics/FiveAxisKinematics.{h,cpp}`, guarded by `SUPPORT_FIVEAXIS`
(`src/Config/Pins.h`), instantiated in `Kinematics::Create`.

Three differential mappings exist:

| Type | `M669 K` | Status |
|---|---|---|
| `coreXBYC`  | 15 | The earlier hand-derived mapping. Superseded. |
| `coreXBYC2` | 16 | **In production** on both five-axis machines. |
| `coreXBYC3` | 17 | K16's matrix, but homes the differential pairs as individual drives (`GetHomingMode`). The only behavioural difference from K16. |

All three are kept so existing `config.g` files keep working. Prefer K16 for new machines.

Geometry is configured by `M669` parameters, all documented on the member declarations in
the header: `A`=a5, `D`=d6, `R`=bRatio, `Q`=cRatio, `X`=xSkew, `Y`=ySkew, `U`=xzSkew,
`V`=yzSkew, `W`=xySkew, `B`=bSkew, `P`=degreesPerSegment.

> **Known trap.** `Q` sets `cRatio`. An `S` parameter is *not* read — the `TryGetFValue('S', ...)`
> call is commented out in `Configure`. A `config.g` line of the form
> `M669 A{...} D{...} R{global.bratio} S{global.cratio}` therefore leaves `cRatio` at its
> default and silently mis-scales the C axis. Use `Q` for cRatio.

**Known limitation:** `Recalc` inverts the differential matrix with
`FixedMatrix::GaussJordan`, which eliminates without partial pivoting. It can lose precision,
or fail outright on a singular leading entry, for ill-conditioned geometry. The matrices here
are small, well scaled and near-diagonal so it has not caused trouble in practice; partial
pivoting is the correct fix if a configuration ever produces a bad forward matrix.

### 2. Prusa MMU2S / MMU3 driver — `M1750`

`src/Comms/MMU2S/{MMU2S.h,MMU2S.cpp,MMU2SProtocol.h}`, guarded by `SUPPORT_MMU2S`
(enabled per board in `Pins_Duet3Mini.h`, `Pins_Duet3_MB6HC.h`, `Pins_Duet3_MB6XD.h`).
Speaks the Prusa hex+CRC8 protocol over a UART configured with `M575 P{n} S8 B115200`.

`M1750` subcommands: `T`=toolchange, `L`=load, `U`=unload, `E`=eject, `K`=cut, `H`=home,
`R`=reset+re-handshake, `P`=FINDA query, `B`=button, `W`=wait for the current operation,
`V`=verbose UART logging, `F`=manual filament-sensor state, `D`=sensor trip delay,
`Z`=Bowden length, `X`=extra load distance, `C`=assign a real sensor pin. Bare `M1750`
prints status. Adding `S1` to an operation starts it asynchronously; pair with `M1750 W`.

### 3. Per-motor motion limits — `M201.2` / `M203.2` / `M205.2`

`P<motor> S<value>` sets per-physical-motor acceleration, feedrate and jerk. Applied by the
DDA from the real motor step deltas, which is why the kinematics classes do not handle them.

### 4. Per-motor wear statistics

`src/Platform/PrinterStatistics.{h,cpp}`, persisted to `0:/sys/printerstats.json`
(format version 2). Reported as the last `M122` diagnostics part.

Travel is accumulated per **logical drive** in absolute microsteps, drained from
`Move::GetAccumulatedWear` (an atomic read-and-clear on `DriveMovement::wearAccumulator`).
Because the DDA runs in motor space via `CartesianToMotorSteps`, this is true per-motor
travel and is correct on every kinematic — Cartesian, CoreXY, delta, SCARA, five-axis, IDEX.

Note `lifetimePrintJobs` counts jobs **started**, incremented on the idle→printing
transition, not jobs completed. A pause/resume cycle does not double-count, because the
counter only re-arms once the machine is neither printing nor paused.

Format v1 stored per-*axis* travel, which cannot be converted to per-motor microsteps. When
a v1 file is loaded, print time and job count carry over and the per-motor counters start at
zero.

## Build system

Makefiles are the source of truth. `.cproject`/`.project`/`.settings` are still tracked for
the Eclipse OpenOCD debug launch configurations, but are **not** kept in sync — do not
hand-edit them when adding sources. The editor is served by `.clangd` plus
`compile_commands.json`; `.clangd` documents its own `compiledb` regeneration command.

See `CLAUDE.md` for build commands and the workspace layout.

### The find-glob trap

Source discovery must not use find's `! -path <glob>` predicates. GNU make 4.4.1 on Windows
runs `$(shell find ...)` without handing it to a shell and expands the wildcards itself, so
find receives pre-expanded paths, aborts with `paths must precede expression`, and returns
nothing — yielding an empty source list, an empty archive, and undefined references at link
time. Quote style is irrelevant: `'*/dir/*'`, `"*/dir/*"` and the single-line form all fail
identically, while the same command run by hand under `sh -c` works, which makes it easy to
misdiagnose. Collect with a plain `find` and filter with `$(foreach)`/`$(findstring)`.

## Upstream merge surface

41 files under `src/` differ from upstream, excluding the vendored `MQTT_C` and `Lwip` trees:
**7 new fork-owned files** plus **34 modified upstream files**. Only the latter 34 can conflict,
and keeping that number down is what makes merging from `upstream/3.7-dev` tractable.

Regenerate these counts with:

```bash
git diff --diff-filter=A --name-only ed7e034c7 HEAD -- src/ \
    ':!src/Networking/MQTT/MQTT_C' ':!src/Networking/LwipEthernet/Lwip'   # new files
git diff --diff-filter=M --name-only ed7e034c7 HEAD -- src/ \
    ':!src/Networking/MQTT/MQTT_C' ':!src/Networking/LwipEthernet/Lwip'   # modified files
```

**Fork-owned new files** (7) — no conflict risk:
`Comms/MMU2S/{MMU2S.h,MMU2S.cpp,MMU2SProtocol.h}` ·
`Movement/Kinematics/FiveAxisKinematics.{h,cpp}` · `Platform/PrinterStatistics.{h,cpp}`

**Upstream files touched, and why:**

| Area | Files | Reason |
|---|---|---|
| Feature flags | `Config/Pins.h`, `Pins_Duet3Mini.h`, `Pins_Duet3_MB6HC.h`, `Pins_Duet3_MB6XD.h` | Define `SUPPORT_FIVEAXIS` / `SUPPORT_MMU2S` |
| Kinematics registry | `Movement/Kinematics/Kinematics.{h,cpp}` | New `KinematicsType` values + `Create` cases |
| G-code dispatch | `GCodes/GCodes.{h,cpp}`, `GCodes2.cpp`, `GCodes4.cpp`, `GCodes6.cpp`, `GCodeMachineState.h`, `StraightProbeSettings.h` | `M1750`, `M201.2/M203.2/M205.2`, `M301/M304` rework, MMU2S state machine |
| Per-motor limits | `Movement/DDA.{h,cpp}`, `Platform/Platform.cpp` | Store and apply the per-motor limits |
| Wear tracking | `Movement/DriveMovement.{h,cpp}`, `Movement/Move.{h,cpp}` | `wearAccumulator` + `GetAccumulatedWear` |
| Statistics host | `Platform/RepRap.{h,cpp}` | Owns `PrinterStatistics`; calls `Spin`/`Init`/`Report` |
| Heater PID | `Heating/{FOPDT.h,FOPDT.cpp,Heat.h,Heat.cpp,Heater.h}` | `M301`/`M304` PID override support |
| Input shaping | `Movement/AxisShaper.{h,cpp}` | Three negative shapers (`nzvum`, `nzvdum`, `neium`) |
| Serial | `Comms/AuxDevice.{h,cpp}`, `CAN/CanInterface.cpp` | MMU2S UART mode |
| Probing | `GCodes/StraightProbeSettings.h` | Probing changes for the five-axis machines |
| Identity | `Version.h` | `+unlayered.1` suffix so local builds are identifiable |
| Misc | `RepRapFirmware.h` | Forward declarations |

Deliberately **not** touched, to keep the surface small: `Movement/BedProbing/Grid.cpp`,
`Movement/Kinematics/CoreKinematics.cpp` and `PolarKinematics.cpp` were reverted to upstream
in July 2026 because their only local changes were comments (see the design notes below).

## Design notes

### Height-map interpolation: G1 continuity

`HeightMap::InterpolateAxis0Axis1` (`src/Movement/BedProbing/Grid.cpp`) is bilinear over the
four surrounding grid points: C0 continuous, but the surface normal jumps at every cell
boundary. The four terms are, in order: the amount unaffected by either axis, the amount
affected only by axis0, only by axis1, and by both.

Idea for G1 continuity, recorded before the note was reverted out of the upstream file:

> This is the function to modify to add G1 continuity — look out an additional point in the
> grid. It is basically a correlation problem: each grid height needs a "slope", found by
> looking at its neighbours, using just the one neighbour where only one exists. Probably a
> Bézier.

### Why the negative shaper coefficients are curve fits

The `Mt<N><shaper>` tables in `AxisShaper.h` have no closed-form solution. Each row is a
least-squares cubic in the damping ratio ζ giving the normalised time of impulse N:
`t_N * frequency = M[0] + M[1]ζ + M[2]ζ² + M[3]ζ³`. From
*Time-Optimal Negative Input Shapers*,
https://asmedigitalcollection.asme.org/dynamicsystems/article/119/2/198/442325

### The `InputShaperType` enum is not alphabetical

The enum comment requires alphabetical order and the three fork shapers are appended after
`zvddd` instead. This is deliberate: `NamedEnumLookup`
(`RRFLibraries/src/General/NamedEnum.cpp`) is a linear `strcmp` scan, so ordering does not
affect name lookup, whereas inserting them would renumber `none`/`zvd`/`zvdd`/`zvddd` and
rewrite a block upstream also appends to — a guaranteed conflict on every future upstream
shaper.
