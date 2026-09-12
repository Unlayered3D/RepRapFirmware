/*
 * PanelOtaUpdater.h - stream a firmware image out of an aux port to an Unlayered panel.
 *
 * Unlayered3D fork. The wire protocol is specified in
 * Developer-documentation/panel-firmware-push.md and the receiving end lives in a separate
 * repository (C:\unlayered-panel-35). Treat that document as the contract: any framing change
 * here breaks a codebase that is not in this tree, silently and mid-transfer.
 *
 * WHY THIS IS NOT PanelDueUpdater. RRF flashes a PanelDue by sending the running app
 * {"controlCommand":"eraseAndReset"} so its ATSAM jumps into the SAM-BA ROM loader, then
 * speaking SAM-BA over the same wire. That cannot be copied for an ESP32 panel: its ROM loader
 * is on UART0 and needs GPIO0 held low at reset, and the link is UART1. It is also worse -
 * SAM-BA erases before it receives, so a dropped transfer leaves a device that can neither boot
 * nor be reached without the USB cable this feature exists to avoid. The panel has two OTA
 * slots instead: the running image is untouched until the new one has booted and confirmed
 * itself, and its bootloader reverts if it does not.
 *
 * So this class is only a sender. It has no idea how the far end stores what it is given, and
 * deliberately makes no decision the panel is better placed to make - it does not know the
 * slot size, and MaxImageSize below is a "you pointed this at a G-code file" check, not a
 * capacity check. The panel refuses with an abort if the image will not fit.
 *
 * IT MUST NOT BLOCK. Spin() is driven from the GCode state machine, and AsyncSerial::write()
 * spins until the whole block is buffered - so every write is clamped to canWrite() first and
 * the remainder waits for the next call. A 745 kB image is roughly two minutes on the wire.
 */

#ifndef SRC_COMMS_PANELOTAUPDATER_H_
#define SRC_COMMS_PANELOTAUPDATER_H_

#include <RepRapFirmware.h>

#if SUPPORT_PANEL_OTA

#include <General/NamedEnum.h>
#include <General/StringRef.h>
#include <Storage/CRC32.h>

#include <AsyncSerial.h>

class FileStore;

// Bytes of image per chunk. Sent in the announce, so the panel takes it from the wire rather
// than agreeing it here - but 1024 is what it has been proved with. Larger chunks amortise the
// per-chunk acknowledgement over more payload; smaller ones bound how much is resent when one
// goes bad. At 57600 baud 1024 bytes is ~180 ms.
constexpr uint32_t PanelOtaChunkSize = 1024;

// A sanity ceiling, NOT the panel's slot size - see the note above.
constexpr uint32_t PanelOtaMaxImageSize = 2u * 1024u * 1024u;

// The announce line carries the filename, so the header buffer has to hold the longest one plus
// the JSON around it. Sizing this to StringLength100 truncates a long name, and a truncated line
// is not a cosmetic problem: RRF has no timeout that abandons a partial aux line, so a chopped
// quoted string wedges the far end's parser.
constexpr size_t PanelOtaHeaderLength = MaxFilenameLength + 100;

constexpr uint32_t PanelOtaReadyTimeout = 5000;		// panel has to open its inactive slot
constexpr uint32_t PanelOtaAckTimeout = 2000;		// per chunk, on top of transmission time
constexpr uint32_t PanelOtaDoneTimeout = 30000;		// esp_ota_end() hashes the whole slot
constexpr uint8_t PanelOtaMaxRetries = 3;			// resends of one chunk before giving up

class PanelOtaUpdater
{
public:
	PanelOtaUpdater() noexcept;

	void Spin() noexcept;
	void Start(const StringRef& filenameRef, uint32_t serialChan) noexcept;
	bool Idle() const noexcept { return state.RawValue() == PushState::idle; }

private:
	NamedEnum(PushState, uint8_t,
		idle,
		hashing,			// reading the whole file once to CRC it, a slice per Spin
		announce,			// sending the {"fwPush":{...}} line
		waitReady,			// waiting for "fwPush ready"
		loadChunk,			// reading the next chunk off the SD card and framing it
		sendChunk,			// pushing header + body into the UART, canWrite() at a time
		waitAck,			// waiting for "fwPush ack <seq>"
		sendDone,			// sending {"fwPush":{"done":true}}
		waitDone,			// waiting for "fwPush done" after the panel validates
		done				// tidy up and return to idle
	);

	AsyncSerial *_ecv_null GetAuxPort() const noexcept;

	// Push as much of header[headerSent..] then buffer[..chunkLength] as the UART will take
	// without blocking. Returns true when the whole frame has been handed over.
	bool PushFrame() noexcept;

	// Accumulate received bytes into replyLine. Returns true when a complete line is ready.
	bool CollectReply() noexcept;

	void Fail(const char *_ecv_array why) noexcept;
	void BeginWait() noexcept;
	bool WaitedTooLong(uint32_t timeout) const noexcept;

	uint32_t serialChannel;
	FileStore *_ecv_null firmwareFile;
	AsyncSerial::InterruptCallbackFn _ecv_null currentInterruptCallbackFn;

	CRC32 hashCrc;						// accumulates the whole-image CRC during the hashing pass

	uint32_t fileSize;					// bytes in the image
	uint32_t imageCrc;					// CRC-32 of the whole image, sent in the announce
	uint32_t bytesHashed;				// progress through the hashing pass
	uint32_t bytesSent;					// progress through the transfer proper
	uint32_t seq;						// 0-based chunk number
	uint32_t chunkLength;				// payload bytes in the chunk being sent
	uint32_t framePos;					// how much of header+payload has reached the UART
	uint32_t headerLength;
	uint32_t whenStartedWaiting;
	uint32_t lastPercentReported;
	uint8_t retries;					// resends of the current chunk

	PushState state;

	String<StringLength100> replyLine;
	String<MaxFilenameLength> filename;
	char header[PanelOtaHeaderLength];
	uint8_t buffer[PanelOtaChunkSize];
};

#endif	// SUPPORT_PANEL_OTA

#endif /* SRC_COMMS_PANELOTAUPDATER_H_ */
