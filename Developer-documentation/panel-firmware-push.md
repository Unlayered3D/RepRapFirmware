# Pushing panel firmware over an aux port — `M997 S5`

**This document is the contract between two codebases.** RRF sends; the ESP32 panel firmware
at `C:\unlayered-panel-35` receives. They are written by different people at different times,
so nothing here is an implementation detail — if one end changes any framing decision below,
the other end breaks silently and mid-transfer. Change this file first.

- Sender: `src/Comms/PanelOtaUpdater.{h,cpp}` in this repo, guarded by `SUPPORT_PANEL_OTA`.
- Receiver: `components/panel_ota` + the link task in `components/duet_link`, driving
  `panel_ota_begin()` / `panel_ota_feed()` / `panel_ota_finish()`.

## Why this exists

The panel is screwed to a machine. Its USB port faces the wrong way, reaching it means taking
the panel off, and plugging it in **resets the board** — so the cable destroys the state you
attached it to observe. This is the update path that avoids it.

## Why it is not the PanelDue's method

RRF flashes a PanelDue by sending the running app `{"controlCommand":"eraseAndReset"}` so its
ATSAM jumps into the SAM-BA ROM loader, then speaking SAM-BA over the same wire. That cannot be
copied here: **the ESP32's ROM loader is on UART0 and needs GPIO0 low at reset**, and this link
is UART1. There is no equivalent.

It is also worse. SAM-BA **erases before it receives**, so a dropped transfer leaves a device
that cannot boot and cannot be reached without the cable this whole feature exists to avoid.
The panel instead has two OTA slots: the running image is untouched until the new one has
booted and confirmed itself, and the bootloader reverts if it does not.

## Preconditions

- The aux channel is in **PanelDue mode** (`M575 P2 S0 B57600 H0` on a 3.7 board — `P2`, not
  `P1`; a second USB channel shifted the aux ports up in 3.7). `M997` refuses on a raw channel,
  same as `S4`.
- **aux0 only** (the `IO_0` connector), because `ALLOW_ARBITRARY_PANELDUE_PORT` is 0 upstream
  and the aux-output suppression is written as `auxNumber == 0`. A panel on aux1 would have
  RRF's own messages interleaved into the byte stream. Same constraint the PanelDue flasher has.
- The image is at `0:/firmware/<name>.bin`, default `unlayered-panel.bin`.
- Like every `M997`, this switches all heaters off and disables drives before it starts.

While a push is running, RRF suppresses its own aux output and stops parsing the aux channel as
G-code — otherwise the object-model push would interleave with the byte stream and the panel's
ACKs would be read as commands. Both are the same gate the PanelDue flasher already uses.

## Framing

Every line is newline-terminated ASCII. Raw payload bytes appear **only** inside a chunk body,
whose length was declared in the line immediately before it. So the receiver is in line mode by
default and only ever reads binary for an exact, pre-announced byte count.

Base64 and hex framing were both rejected: 33% and 100% overhead on a link where the transfer
is already the bottleneck.

### 1. Announce (Duet → panel)

Preceded by a bare `\n`, so any partial line already in the panel's parser is terminated first.

```
{"fwPush":{"size":762144,"crc":3735928559,"chunk":1024,"name":"unlayered-panel.bin"}}
```

- `size` — total image bytes.
- `crc` — CRC-32 of the **whole image**, decimal, unsigned.
- `chunk` — bytes per chunk. Every chunk carries exactly this many except the last.
- `name` — informational, for the panel's progress label. Not a path.

**The whole-image CRC is sent up front, before a single content byte.** Same ordering the
panel→Duet upload path uses (`M559 P"..." C<crc>`) and for the same reason: the receiver knows
what it is checking against without having to trust the sender afterwards.

### 2. Ready (panel → Duet)

```
fwPush ready
```

or

```
fwPush abort <reason>
```

The panel sends `ready` once `panel_ota_begin(size)` has opened the inactive slot. It should
refuse — with `abort` — if the image cannot fit the slot, if an install is already running, or
if a print is in progress. Timeout: **5 s**, then RRF aborts.

A real PanelDue on this port never answers, so the push times out having sent ~90 bytes and
nothing else happens. That is the intended behaviour, not a failure to handle.

### 3. Chunks (Duet → panel), repeating

```
{"fwData":{"seq":0,"len":1024,"crc":2596743674}}
<exactly 1024 raw bytes, no terminator>
```

- `seq` — 0-based, increments by one. Not a byte offset.
- `len` — bytes in this chunk's body. Equals `chunk` except for the last.
- `crc` — CRC-32 of **this chunk's body only**.

The panel buffers `len` bytes, checks the CRC, and only then calls `panel_ota_feed()`. It must
not feed unverified bytes — `panel_ota_feed()` is append-only and cannot rewind, so a chunk fed
and then found bad would force the entire transfer to restart.

### 4. Acknowledgement (panel → Duet), one per chunk

```
fwPush ack 0
```

```
fwPush nak 0 <reason>
```

**The per-chunk ACK is the only flow control on this link.** There is no RTS/CTS, the panel's
RX ring is 8 kB, and a flash write stalls its reader for tens of milliseconds. A free-running
sender overruns it silently — the bytes are simply gone, and the failure surfaces as a CRC
mismatch hundreds of chunks later. RRF sends nothing further until the ACK for `seq` arrives.

On `nak`, or on a **2 s** timeout plus transmission time, RRF resends the same `seq` unchanged.
Three failures on the same chunk abort the transfer.

**The receiver abandons a stalled body after 1.5 s** (since 2026-08-31) and naks it with
`fwPush nak <seq> timeout`. Before that there was no body timeout at all, and losing the tail
of one chunk wedged the receiver in its body phase — the sender's resend was then consumed as
the stuck body's remainder: a guaranteed CRC mismatch, the rest of the resend fed to the JSON
parser as junk, and a desync no retry ladder recovers (seen live at chunk 138 of the first
230400 bench push). The two timeouts are a contract: **a sender must never resend sooner than
1.5 s after its last byte**, or the resend arrives while the receiver is still mid-body. RRF's
2 s + transmission time satisfies this; keep it that way if either number changes.

The panel must ACK the sequence number it actually received. An ACK for the wrong `seq` is
treated as a desync and aborts — it is not silently accepted, because the alternative is
writing an image with a hole in it that then passes its own header check.

### 5. Completion (Duet → panel)

```
{"fwPush":{"done":true}}
```

The panel calls `panel_ota_finish()`, which is where `esp_ota_end()` validates the app header
and checksum, and replies:

```
fwPush done
```

```
fwPush abort <reason>
```

Timeout: **30 s**, because `esp_ota_end()` hashes the whole slot.

The panel **does not reboot itself**. It reports that the image takes effect on restart. A
panel that restarts the instant a transfer finishes looks exactly like a crash, which is the
one thing an update mechanism must never look like.

### Aborting

Either end may send `fwPush abort <reason>` at any point. RRF closes the file and reports the
reason; the panel calls `panel_ota_abort()`. The running image is untouched in every case.

## CRC

**CRC-32/ISO-HDLC** (reflected polynomial `0xEDB88320`, init `0xFFFFFFFF`, final XOR
`0xFFFFFFFF`) at both ends, for both the whole image and each chunk. One algorithm, so there is
one thing to get wrong rather than two.

- RRF: the `CRC32` class in `src/Storage/CRC32.h`, which is hardware-accelerated on the SAME5x.
- Panel: the existing bitwise `crc32_add()` in `duet_link.c`.

**These two are already known to agree**, which is why no new verification of the algorithm is
needed: the working `M559` upload path has the panel computing a CRC with `crc32_add()` and RRF
checking it with `FileStore::GetCRC32()`, which is this same class.

## Timing — measured 2026-08-31, 1.71 MB image over the USB bench path

| baud   | outcome | elapsed | net rate |
|--------|---------|---------|----------|
| 57600  | OK, 0 naks | 456 s | 3.77 kB/s (65% of wire) |
| 115200 | OK, 0 naks | 301 s | 5.70 kB/s (49% of wire) |
| 230400 | **panel crashed and rebooted mid-transfer, twice** (chunks 138 and 2) | — | — |

The fixed ~80 ms per-chunk turnaround (CRC + flash write + ACK) is why doubling the baud gives
1.5x, not 2x — the wire stops being the bottleneck. The next lever is **chunk size**: the
receiver already accepts 2048 (`OTA_RX_CHUNK_MAX`), so raising `PanelOtaChunkSize` from 1024
is an RRF-side change only. 4096 would need both ends.

**The baud is paired config, not a constant**: the panel reads `link_baud` from `panel.cfg`
(default 57600) and the machine sets `M575 ... B<baud>`. A mismatch is rx=0 — indistinguishable
from a dead wire from either end — so change both together or neither.

**230400 is not cleared.** Both bench attempts killed the panel outright (its counters came
back from zero after ~60 s: a reboot, not byte loss), which no protocol robustness can absorb.
It could not be diagnosed on the bench because the USB link rides the console pins (GPIO43/44),
so the panic goes out unreadable at the wrong baud. On real machine wiring the link is on
GPIO17/18 and the console stays free — debug it there with `tools/crashlog.py` watching while a
230400 push runs, before trusting that rate anywhere. 115200 is the fastest proven rate.

The hand-made-cable caveat still stands on top: these numbers are from a USB bench cable, and
marginal signalling on the real loom presents as chunks that need resending. The stall-nak above
now makes that survivable, but prove a new rate on the machine before adopting it.

## Known hazard

RRF has **no timeout that abandons a partial line on an aux channel**. A corrupted announce or
`fwData` line whose quoting is broken puts the parser into `parsingQuotedString`, where it
swallows every following byte including newlines, and the channel cannot recover without being
reinitialised. This is a pre-existing fault, not one this feature introduces, but a 745 kB
transfer rides this wire for two minutes and so meets it far more often than a `M409` does.
Enabling checksums (`M575 ... S1` with `LINK_CHECKSUMS` on the panel) is the available
mitigation until the fork grows a partial-line timeout.
