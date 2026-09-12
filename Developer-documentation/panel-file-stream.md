# Printing a file from the panel's own card — `M1760` / `M1761` / `M1762`

**This document is the contract between two codebases.** The ESP32 panel at `C:\unlayered-panel`
sends; RRF receives. Nothing here is an implementation detail — if one end changes a framing
decision below, the other breaks silently, hours into a print. Change this file first.

- Receiver: `src/Comms/PanelPrintStream.{h,cpp}`, guarded by `SUPPORT_PANEL_PRINT`, plus the read
  gate in `GCodes::DoFilePrint` / `FileGCodeInput::ReadFromFile` and the length clamp in
  `FileInfoParser`.
- Sender: the panel's `duet_link` uploader (ported from the 3.5" panel's `M559` uploader) and its
  Files tab.

## Why this shape

The panel's microSD is the only slot a user can reach once a machine ships with the Duet's card
internal, and the aux UART on `IO_0` is the only wire between them. Two ways to print from it were
rejected:

- **Streaming G-code line by line** (what a BTT TFT does) makes the panel the source of the print.
  RRF's whole job pipeline — `job.file`, progress, pause/resume seeking the file, `M0`, DWC's job
  card, `M36` thumbnails, resurrect — assumes a seekable file on the Duet's card. All of it would be
  lost, and RRF's 256-byte input buffer stalls on fine detail at any baud this link runs.
- **Upload, then print** keeps every job semantic but makes a 20 MB file wait ~30 minutes at
  115200 before the first move.

So the panel **uploads the file into a cache on the Duet's card while RRF prints from that same
file**. The print is an ordinary SD print in every respect; RRF only has to stop its reader at the
point the upload has reached, and the panel only has to stay ahead of the nozzle on average. The
whole card is the buffer, so bursts of fine detail do not matter — only the average consumption rate
has to be below the link rate.

## Preconditions

- The aux channel is in **PanelDue mode** and the panel is on **`IO_0`** (`M575 P2 S1 B115200` on
  a 3.7 board). Checksummed `N…*cs` lines work: the header lines go through the normal parser.
- 115200 baud is proven; 57600 works and halves the rate. See the panel's `CLAUDE.md` for 230400.
- One stream at a time, one cache file at a time: `0:/gcodes/panel/<name>`. A new announce deletes
  the previous cache unless it is the file being printed.

## Framing

Every command is an ordinary G-code line. Raw payload bytes appear **only** after an `M1761`
line, and exactly as many as that line declared — so RRF is in line mode by default and reads
binary for a pre-announced count only. Every reply is **one JSON object on one line with a single
`pfile` key**, sent raw (an RRF reply that starts with `{` is not wrapped in `{"seq":…,"resp":…}`
on a PanelDue-mode channel). Replies are always `GCodeResult::ok` at the RRF end, because an error
result gets `Error:` prefixed and stops being JSON — **the panel must read `reason`, not the result**.

### 1. Announce (panel → Duet)

```
M1760 P"impeller.gcode" S23456789 C3735928559
```

- `P` — the bare file name. **No path separators, no drive prefix**; RRF puts it under
  `0:/gcodes/panel/`. The panel does not choose where the cache lives.
- `S` — total bytes. `C` — CRC-32 of the whole file, decimal, unsigned.

RRF creates the file **at its final size immediately** (`f_expand`, falling back to a seek-extend on
a fragmented card), so `Length()`, `job.file.size` and the progress fraction are right from the
first byte. The contents beyond the committed frontier are whatever those sectors held before —
which is why every reader RRF has is clamped (below).

Reply:

```
{"pfile":{"ready":true,"path":"0:/gcodes/panel/impeller.gcode","committed":0,"chunk":1024}}
{"pfile":{"ready":false,"reason":"printing"}}
```

- `chunk` — the largest body RRF accepts. Take it from the wire; do not assume 1024.
- `committed` — where to start sending. **0 for a new file; nonzero means RRF still holds this same
  file (same name, size and CRC) part-received** — the panel lost the link or rebooted — and the
  panel resumes from that offset. RRF's state does not survive its own reboot, so after a Duet power
  cycle the announce starts a fresh file at 0.
- `reason`: `name` (empty or contains `/ \ :`), `size` (0), `long` (path too long), `printing`
  (the existing cache is what is being printed — cancel that print first), `directory`, `open`,
  `space` (the card could not hold it), `reopen`.

### 2. Chunk (panel → Duet), repeating

```
M1761 O0 L1024 C2596743674
<exactly 1024 raw bytes, no terminator>
```

- `O` — byte offset. **Must equal RRF's `committed`**: chunks are strictly sequential from 0, which
  is what lets RRF keep one running CRC and one frontier, and lets the metadata parser and the print
  reader both trust "everything below `committed` is real".
- `L` — body length, `1 ≤ L ≤ chunk`. Every chunk but the last is `chunk` long.
- `C` — CRC-32 of this body only.

RRF takes the body straight off the port — the `M1761` stays "executing" until all `L` bytes have
arrived, so its GCodeBuffer does not refill and no body byte is ever parsed as G-code — then checks
the CRC, writes at `O`, **syncs the card**, advances `committed`, and replies:

```
{"pfile":{"ack":0,"committed":1024}}
{"pfile":{"nak":0,"reason":"crc","committed":0}}
```

`reason`: `crc`, `order` (`O ≠ committed`, or `O+L > size` — resend from the returned
`committed`), `timeout` (the body stalled), `write` (card error), `nostream` (no announce is in
force), `length` (`L` outside `1..chunk` — the one refusal that does **not** drain the body, because
RRF cannot know how much to swallow; the panel must never send it).

**The per-chunk ACK is the only flow control.** RRF's aux receive ring on `IO_0` is 2048 bytes
(raised from 512 for this), the body is at most 1024, and the ACK is sent only after the card write
has synced — so the next body can never arrive while RRF is busy with the last one, and a main-loop
stall of any length cannot overrun the ring. **Send nothing until the ACK or NAK arrives.**

**Timeouts are a pair.** RRF abandons a body that has been silent for **2 s** and NAKs `timeout`.
The panel's own wait for an ACK must therefore be **longer than 2 s plus the body's transmission
time** (4 s is right at 115200), or its resend lands while RRF is still collecting the previous body
and is swallowed into it — a guaranteed CRC mismatch and a desync. On any NAK, or on that timeout,
resend the same chunk unchanged; three failures on one chunk, give up (`M1762 S0`).

### 3. Start the print (panel → Duet), whenever the panel likes

```
M32 "0:/gcodes/panel/impeller.gcode"
```

The path is the one the announce returned. **Ten seconds after the first ACK is the design point**,
not "after the header" or "after the footer": RRF's job reader is held at `committed` (the print
simply waits there, status stays `processing`), and RRF's metadata parser reads only up to
`committed` — so neither can read stale sectors, and the order the file is sent in is the natural
one, 0 → end. OrcaSlicer puts print time, layer count, filament and height in its **header** block,
so those appear straight away; anything only in the footer appears when the upload completes and RRF
**re-parses the whole file once** (`PrintMonitor::ReparseFileInfo`, triggered when `committed`
reaches `size`).

While the reader is starved RRF says so — `Waiting for the panel to send more of <path>` — at most
once a minute. Heating takes minutes and buffers megabytes; a print that then consumes faster than
the link stalls in place, with everything still consistent. Keep uploading between status polls for
the whole print; the panel's own poll interval is what it costs.

For the job card's thumbnail the panel should read **its own copy of the file** — it has the bytes
at MB/s and `M36.1` on a still-arriving file returns stale sectors past `committed`.

### 4. Finish or abort (panel → Duet)

```
M1762 S1        -> {"pfile":{"done":true}}
                   {"pfile":{"done":false,"reason":"short","committed":N}}   not all bytes were sent
                   {"pfile":{"done":false,"reason":"crc"}}                    running CRC ≠ announced CRC
M1762 S0        -> {"pfile":{"aborted":true}}                                cache deleted
                   {"pfile":{"aborted":false,"reason":"printing"}}           cancel the print first
M1762           -> {"pfile":{"active":true,"path":"…","size":N,"committed":N,"printing":true}}
```

The whole-file CRC costs nothing extra: chunks are sequential, so the running CRC over accepted
bodies is the file's. On `done:false` with `crc` RRF deletes the cache unless it is being printed —
the print has been consuming chunk-verified data, so the panel decides whether to cancel.

**The cache is kept after a successful print** so it can be reprinted from DWC. The next announce
replaces it. Nothing cleans `0:/gcodes/panel/` at boot; a partial cache left by a Duet power cycle is
overwritten by the next announce of any name.

## CRC

**CRC-32/ISO-HDLC** (reflected `0xEDB88320`, init `0xFFFFFFFF`, final XOR `0xFFFFFFFF`) for both
the whole file and each body — the `CRC32` class here (hardware on the SAME5x), the bitwise
`crc32_add()` on the panel. The pair is already proven to agree by the firmware push and the panel's
`M559` upload path.

## Rates

Net rate at 115200 with 1024-byte bodies is about 8–9 kB/s (the wire is 11.5 kB/s; the rest is the
per-chunk turnaround: RRF's write + sync and the ACK). A 4 h / 30 MB print averages 2 kB/s of
consumption. 57600 halves the rate. Bodies could be 2048 if `PanelPrintChunkMax` and the panel both
change — the ring already holds it — for roughly +15%; a higher baud is the bigger lever once
230400 is cleared on machine wiring.

## Known limits

- RRF's stream state is in RAM: a Duet power cycle mid-upload means the panel starts over
  (resurrect of that job is therefore also not supported yet).
- Text `.gcode` only. The cache name is the panel's file name, so the panel is responsible for it
  being unique enough for the operator.
- `M1760–M1762` are accepted only from an aux port — the body is read from that port, so there is
  nowhere else it could come from.

## Where the panel's file comes from (2026-09-09)

Nothing above changes, but the panel's card is no longer filled by hand. The panel runs a small HTTP
server (`components/panel_http` in the panel repo) that answers the four `rr_*` URLs the Unlayered
Slicer's print host calls - `rr_connect`, `POST rr_upload`, `rr_gcode` with `M32`, `rr_disconnect` -
so the slicer uploads to the panel's card over WiFi as if the panel were a Duet, and `M32` starts the
stream above. The machine needs no network of its own for this. The panel also sends `M118 P3` lines
(HTTP console) at each stage of the stream, so the transfer can be watched from DWC or `rr_reply`;
RRF drops those when no HTTP session is open. Detail and status: the panel repo's `CLAUDE.md`,
section "2026-09-09: the panel as a WiFi print host".
