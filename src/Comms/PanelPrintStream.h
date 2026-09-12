/*
 * PanelPrintStream.h - accept a G-code file from an Unlayered panel over an aux port, into a cache on
 * the SD card, and let it be printed while it is still arriving.
 *
 * Unlayered3D fork. The wire protocol is specified in Developer-documentation/panel-file-stream.md
 * and the sending end lives in a separate repository (C:\unlayered-panel). Treat that document as
 * the contract: a framing change here breaks a codebase that is not in this tree, hours into a print.
 *
 * WHY A CACHE ON OUR CARD RATHER THAN THE PANEL STREAMING LINES. Every job semantic RRF has -
 * job.file, the progress fraction, pause and resume seeking the file, M0, DWC's job card, M36
 * thumbnails, resurrect - assumes a seekable file on this card. Streaming lines would give all of
 * that up and stall on fine detail at any baud the link runs. So the panel uploads INTO a file here
 * and prints THAT file, and the only thing this class asks of the rest of RRF is: do not read past
 * the point the upload has reached. Three readers exist and all three are clamped: the job reader
 * (GCodes::DoFilePrint -> FileGCodeInput::ReadFromFile), the metadata parser (FileInfoParser), and
 * nothing else - M36.1 is not, which is why the panel reads thumbnails from its own copy.
 *
 * THE FILE IS CREATED AT ITS FINAL SIZE. f_expand (or a seek-extend if the card is too fragmented
 * for a contiguous block) means Length(), job.file.size and the progress fraction are right from the
 * first byte, and FatFs' tiny mode means a reader sees a writer's sectors as soon as they are synced.
 * The price is that the unwritten part holds whatever those sectors held before, hence the clamps.
 *
 * THE CHUNK BODY IS READ FROM THE PORT BY THE M1761 HANDLER ITSELF, which returns notFinished until
 * the body is complete. While a command is executing its GCodeBuffer is not refilled, so no body
 * byte ever reaches the G-code parser - and nothing here has to gate the aux channel the way the
 * firmware push does. It does mean this class must never block: a body arrives at 11 bytes/ms and
 * the handler is re-entered from the main loop until it has them all.
 *
 * NO WRITE BUFFER. The writer is opened in append mode, which does not take one of the board's
 * FileWriteBuffers (the Mini 5+ has exactly one, and resurrect.g, the log and DWC uploads all want
 * it for the hours this upload can last). Each chunk is written and synced as it arrives; the sync
 * is what makes 'committed' true for the readers.
 */

#ifndef SRC_COMMS_PANELPRINTSTREAM_H_
#define SRC_COMMS_PANELPRINTSTREAM_H_

#include <RepRapFirmware.h>

#if SUPPORT_PANEL_PRINT

#include <General/StringRef.h>
#include <Storage/CRC32.h>

class GCodeBuffer;
class FileStore;
class AsyncSerial;

// The largest chunk body we accept. Sent in the M1760 reply, so the panel takes it from the wire rather than
// agreeing it here. It must be smaller than the aux port's receive ring (Serial0Params.numRxSlots, 2048 on the
// Mini 5+) with room for the next header line, so that a main-loop stall of any length while a body is in the
// ring cannot overrun it - the panel sends nothing until the previous body has been acknowledged.
constexpr size_t PanelPrintChunkMax = 1024;

constexpr uint32_t PanelPrintBodyTimeout = 2000;			// ms without a byte, mid-body, before the chunk is abandoned and NAKed.
															// The panel's resend timeout MUST exceed this plus transmission time.
constexpr uint32_t PanelPrintStallReportInterval = 60000;	// ms between "waiting for the panel" messages while the job reader is starved

constexpr const char *_ecv_array PanelPrintDirectory = "0:/gcodes/panel/";

class PanelPrintStream
{
public:
	PanelPrintStream() noexcept;

	// The three M-codes. Every reply is a JSON object with a single "pfile" key and the result is GCodeResult::ok
	// even when refusing - an error result gets "Error: " prefixed and stops being JSON. ReceiveChunk returns
	// notFinished until the whole body has been taken off the port.
	GCodeResult Announce(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);							// M1760
	GCodeResult ReceiveChunk(GCodeBuffer& gb, AsyncSerial& port, const StringRef& reply) THROWS(GCodeException);	// M1761
	GCodeResult Finish(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);							// M1762

	// The print side, called from GCodes
	void PrintStarting(const char *_ecv_array _ecv_null printingFilename) noexcept;	// a file print is starting: is it our cache?
	void PrintStopped() noexcept;													// the print has ended, however it ended
	bool IsGatingPrint() const noexcept { return gatingPrint; }
	size_t ReadableFrom(FilePosition pos) noexcept;									// how many bytes from 'pos' the job reader may take; SIZE_MAX if unlimited

	// The metadata parser side
	bool IsStreamingFile(const char *_ecv_array filePath) const noexcept;			// is this the cache file, and still arriving?
	FilePosition CommittedLength() const noexcept { return committed; }

private:
	enum class State : uint8_t
	{
		idle,			// no cache file announced
		receiving,		// announced, chunks arriving, committed < declaredSize
		complete		// every byte is on the card; awaiting or past M1762 S1
	};

	void Discard(bool deleteFile) noexcept;
	void ReplyRefused(const StringRef& reply, const char *_ecv_array reason) const noexcept;
	void ReplyNak(const StringRef& reply, const char *_ecv_array reason) const noexcept;

	String<MaxFilenameLength> path;			// full path of the cache file, empty when idle
	FileStore *_ecv_null writer;			// open in append mode while receiving
	uint8_t *_ecv_null body;				// PanelPrintChunkMax bytes, allocated on first use and kept
	CRC32 fileCrc;							// running CRC of everything committed - the file's CRC, because chunks are sequential
	FilePosition declaredSize;
	FilePosition committed;					// bytes from the start of the file that are written and synced
	uint32_t declaredCrc;

	// The chunk in flight
	uint32_t bodyOffset;
	uint32_t bodyLength;
	uint32_t bodyCrc;
	uint32_t bodyReceived;
	uint32_t bodyLastByteTime;
	uint32_t lastStallReport;

	State state;
	bool inBody;							// an M1761 header has been read and its body is being collected
	bool gatingPrint;						// the file being printed is this one, so its reader is held at 'committed'
};

#endif	// SUPPORT_PANEL_PRINT

#endif /* SRC_COMMS_PANELPRINTSTREAM_H_ */
