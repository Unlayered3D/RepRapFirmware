# Piezo support log and support bundle — contract

Three parts, built together on 2026-10-03:

1. **Firmware** (this repo): a continuous 30 s rolling log of the Z probe (piezo) signal on the SD card,
   plus frozen snapshots and an event log. Controlled by `M1770`.
2. **Slicer** (C:\orcaslicer): "Help > Send printer logs to Unlayered" collects a support bundle from the
   printer over `rr_*`, zips it, and uploads it.
3. **Website** (C:\unlayered-pilot): `POST /api/support/upload` stores the bundle and returns a ticket id.

## Why the ring lives on the SD card

Measured on .90 with +unlayered.15: `never used RAM 8320`. 30 s at 1 kHz x 2 bytes = 60 KB, which cannot
be held in RAM. The tick ISR fills three 512-byte blocks in RAM (1.5 KB), and `Platform::Spin` writes each full
block in place into a fixed-size ring file. That is 2 KB/s of SD writes.

## Files on the printer — `0:/logs/piezo/`

| file | what |
|---|---|
| `ring.bin` | the live 30 s ring. Blocks are written in place, wrapping at 122 blocks, so it is NOT in time order: sort blocks by `seq` |
| `snap-NNNN.bin` | a frozen ring. Same format as `ring.bin`. Created by renaming `ring.bin` (no copy) |
| `snap-NNNN.txt` | sidecar for the snapshot: reason, time, firmware, probe config, machine position, temperatures (text, `key: value` lines) |
| `events.log` | text, one line per probe event (arm / trigger / stop / snapshot / boot), rolling: renamed to `events.1.log` past 256 KB |
| `events.1.log` | the previous events log |

The last 10 snapshots are kept (NNNN increases monotonically; the oldest are deleted). At boot, the previous
boot's `ring.bin` becomes a snapshot with reason `boot`, so the 30 s before a crash or power cut survive (minus
at most ~0.75 s still held in RAM).

### Block format (`ring.bin`, `snap-NNNN.bin`), little-endian, 512 bytes per block

```
offset size
0      4    magic   'P','Z','L','1'  (0x314C5A50 as LE u32)
4      4    seq     u32, increments by 1 per block written since boot (0 = first block after boot)
8      4    millis  u32, millis() at the first sample in this block
12     2    count   u16, number of valid samples in this block (<= 248; < 248 only for the last block before a snapshot)
14     2    dropped u16, samples lost immediately BEFORE this block because the SD writer fell behind
16     496  samples 248 x u16
```

One sample per 1 kHz tick (1 ms apart). Sample word:

```
bits 0-9   raw reading, 10-bit (0..1023), what LocalZProbe::GetRawReading returns
bit  10    detector trigger active (diffTrigger != 0)            - type 12 only, else 0
bit  11    trigger sign negative (diffTrigger < 0)                - type 12 only
bit  12    probing move armed (SetProbing(true) .. SetProbing(false))
bit  13    in blanking window after arm                           - type 12 only
bit  14    detector still priming (not yet able to trigger)       - type 12 only
bit  15    reserved, 0
```

Unused blocks in a ring that has not filled yet do not exist (the file just ends). A block whose magic does not
match is ignored. Because the type 12 detector is integer-only and deterministic, delta / filtered can be
recomputed bit-exactly offline from `raw` + the `M558`/`G31` values in the sidecar.

### `events.log` line format

```
<millis> <ISO time or "-"> <EVENT> key=value key=value ...
```

EVENTs: `BOOT`, `ARM` (probing move starts), `TRIG` (trigger edge 0 -> nonzero **during a probing move**; edges
outside one, e.g. homing's shake test, are only in the ring), `STOP` (probing move ends, `trig=0|1` = triggered at
any point during the move — a type 3 reading has usually decayed below threshold again by the time the move ends), `FAIL` (probe error, `msg="..."`), `SNAP` (`n=NNNN reason="..."`). Keys include `x= y= z= b= c=`
(machine coordinates), `thr=` (G31 P), `base=` (armed baseline).

## `M1770` — piezo log control (Unlayered fork)

```
M1770                 report: enabled, blocks written, dropped samples, snapshot count, last error
M1770 S"reason"       freeze the current ring into the next snapshot now (macros call this, e.g. calfail.g)
M1770 E0 / E1         disable / enable logging (default enabled when a Z probe of an analog type exists)
```

A failed probe ("Probe was not triggered", "already triggered") takes a snapshot automatically with
reason `probe-fail`, ~1 s after the failure so the aftermath is in the window.

## Support bundle (slicer -> website)

The slicer, on "Send printer logs to Unlayered":

1. `M1770 S"support"`, wait ~1.5 s, so the current 30 s are frozen as a snapshot.
2. Downloads over `rr_download` (binary-safe!):
   - every file in `0:/logs/piezo/` (`rr_filelist?dir=0:/logs/piezo`)
   - `0:/sys/eventlog.txt` (if present), `0:/sys/config.g`, `0:/sys/config-override.g`,
     `0:/sys/calglobals.g`, `0:/sys/printerstats.json` — each optional, a 404 is skipped
3. Runs `M122` and stores the `rr_reply` text as `m122.txt`.
4. Stores `rr_model?flags=d99vn` as `model.json`.
5. Writes `manifest.json`:
   ```json
   { "format": 1, "created": "<ISO>", "slicerVersion": "...", "printerName": "...",
     "printerHost": "192.168.1.90", "firmware": "3.7.0-beta.1+unlayered.17", "boardUniqueId": "...",
     "note": "<what the customer typed>", "email": "<optional>", "files": ["piezo/ring.bin", ...] }
   ```
6. Zips it as `unlayered-support-<printerName>-<yyyymmdd-hhmmss>.zip` (piezo files under `piezo/`,
   sys files under `sys/`), also saving a copy locally so the customer can attach it by hand.
7. `POST https://unlayered3d.com/api/support/upload`, body = the raw zip bytes,
   `Content-Type: application/zip`, headers `X-Support-Note` (URL-encoded, <= 2000 chars),
   `X-Support-Email` (optional), `X-Printer-Firmware`, `X-Board-Id`.
   Response `200 {"ok":true,"ticket":"SUP-7K3QX2"}`; errors `4xx/5xx {"ok":false,"error":"..."}`.
   Size limit 8 MB (under the 10 MB Next.js proxy body-clone limit).
8. Shows the ticket id, with a copy button and the path of the local zip.
