/*
 * PanelPrintStream.cpp - accept a G-code file from an Unlayered panel into a cache and print it while it arrives.
 *
 * Unlayered3D fork. Protocol: Developer-documentation/panel-file-stream.md.
 */

#include "PanelPrintStream.h"

#if SUPPORT_PANEL_PRINT

#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <PrintMonitor/PrintMonitor.h>
#include <Storage/MassStorage.h>
#include <Storage/FileStore.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <General/StringFunctions.h>
#include <AsyncSerial.h>

PanelPrintStream::PanelPrintStream() noexcept
	: writer(nullptr)
	, body(nullptr)
	, declaredSize(0)
	, committed(0)
	, declaredCrc(0)
	, bodyOffset(0)
	, bodyLength(0)
	, bodyCrc(0)
	, bodyReceived(0)
	, bodyLastByteTime(0)
	, lastStallReport(0)
	, state(State::idle)
	, inBody(false)
	, gatingPrint(false)
{
}

// Forget the current file, optionally deleting it. Callers check gatingPrint first: deleting the file that
// the job reader has open would leave FatFs (FF_FS_LOCK is 0) with a handle onto freed clusters.
void PanelPrintStream::Discard(bool deleteFile) noexcept
{
	if (writer != nullptr)
	{
		writer->Close();
		writer = nullptr;
	}
	if (deleteFile && !path.IsEmpty())
	{
		(void)MassStorage::Delete(path.GetRef(), ErrorMessageMode::noMessage);
	}
	path.Clear();
	declaredSize = committed = 0;
	declaredCrc = 0;
	inBody = false;
	gatingPrint = false;
	state = State::idle;
}

void PanelPrintStream::ReplyRefused(const StringRef& reply, const char *_ecv_array reason) const noexcept
{
	reply.printf("{\"pfile\":{\"ready\":false,\"reason\":\"%s\"}}", reason);
}

void PanelPrintStream::ReplyNak(const StringRef& reply, const char *_ecv_array reason) const noexcept
{
	reply.printf("{\"pfile\":{\"nak\":%" PRIu32 ",\"reason\":\"%s\",\"committed\":%" PRIu32 "}}", bodyOffset, reason, (uint32_t)committed);
}

// M1760 P"name" S<size> C<crc32>
GCodeResult PanelPrintStream::Announce(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	String<MaxFilenameLength> name;
	gb.MustSee('P');
	gb.GetQuotedString(name.GetRef());
	gb.MustSee('S');
	const FilePosition size = gb.GetUIValue();
	gb.MustSee('C');
	const uint32_t crc = gb.GetUIValue();

	// A bare name only. The panel does not choose where the cache lives, and a separator would let it write anywhere.
	if (name.IsEmpty() || strchr(name.c_str(), '/') != nullptr || strchr(name.c_str(), '\\') != nullptr || strchr(name.c_str(), ':') != nullptr)
	{
		ReplyRefused(reply, "name");
		return GCodeResult::ok;
	}
	if (size == 0)
	{
		ReplyRefused(reply, "size");
		return GCodeResult::ok;
	}

	String<MaxFilenameLength> newPath;
	if (!MassStorage::CombineName(newPath.GetRef(), PanelPrintDirectory, name.c_str()))
	{
		ReplyRefused(reply, "long");
		return GCodeResult::ok;
	}

	// The same file announced again mid-stream is the panel coming back after losing the link or rebooting:
	// tell it where we got to and carry on. Its state is gone; ours is not.
	if (state == State::receiving && size == declaredSize && crc == declaredCrc && StringEqualsIgnoreCase(newPath.c_str(), path.c_str()))
	{
		inBody = false;
		reply.printf("{\"pfile\":{\"ready\":true,\"path\":\"%s\",\"committed\":%" PRIu32 ",\"chunk\":%u}}",
						path.c_str(), (uint32_t)committed, (unsigned int)PanelPrintChunkMax);
		return GCodeResult::ok;
	}

	// Anything else replaces the previous cache - unless that cache is what is being printed right now.
	if (gatingPrint)
	{
		ReplyRefused(reply, "printing");
		return GCodeResult::ok;
	}
	Discard(true);

	if (!MassStorage::EnsurePath(newPath.c_str(), false))
	{
		ReplyRefused(reply, "directory");
		return GCodeResult::ok;
	}

	/*
	 * Create the file at its final size before a byte of content exists. Every reader then sees the real
	 * length: job.file.size, the progress fraction and the parser's footer search are all right from the
	 * start, and the print can begin ten seconds into the upload instead of after it.
	 *
	 * Opening in write mode with a preallocation size runs f_expand, which wants a contiguous block; on a
	 * fragmented card that fails and the file stays empty, so fall back to seeking to the end, which FatFs
	 * honours in write mode by allocating cluster by cluster. Either way the file is then closed again,
	 * because write mode took the board's only FileWriteBuffer and this upload can last for hours.
	 */
	Platform& platform = reprap.GetPlatform();
	FileStore *_ecv_null f = platform.OpenFile(PanelPrintDirectory, name.c_str(), OpenMode::write, (uint32_t)size);
	if (f == nullptr)
	{
		ReplyRefused(reply, "open");
		return GCodeResult::ok;
	}
	if (f->Length() < size)
	{
		(void)f->Seek(size);
	}
	const bool sized = (f->Length() >= size);
	f->Close();
	if (!sized)
	{
		(void)MassStorage::Delete(newPath.GetRef(), ErrorMessageMode::noMessage);
		ReplyRefused(reply, "space");
		return GCodeResult::ok;
	}

	// Reopen for the actual writing. Append mode takes no write buffer, keeps the file readable, and puts the
	// pointer at the end - so seek back to the start, where the first chunk goes.
	writer = platform.OpenFile(PanelPrintDirectory, name.c_str(), OpenMode::append);
	if (writer == nullptr || !writer->Seek(0))
	{
		path.copy(newPath.c_str());
		Discard(true);
		ReplyRefused(reply, "reopen");
		return GCodeResult::ok;
	}

	if (body == nullptr)
	{
		body = new uint8_t[PanelPrintChunkMax];		// allocated once and kept, like every other permanent buffer
	}

	path.copy(newPath.c_str());
	declaredSize = size;
	declaredCrc = crc;
	committed = 0;
	fileCrc.Reset();
	inBody = false;
	lastStallReport = 0;
	state = State::receiving;

	reply.printf("{\"pfile\":{\"ready\":true,\"path\":\"%s\",\"committed\":0,\"chunk\":%u}}", path.c_str(), (unsigned int)PanelPrintChunkMax);
	return GCodeResult::ok;
}

// M1761 O<offset> L<len> C<crc32> followed by exactly <len> raw bytes.
// Re-entered from the main loop until the body is in: the command stays executing, so the GCodeBuffer is not
// refilled and the body bytes are ours to take off the port.
GCodeResult PanelPrintStream::ReceiveChunk(GCodeBuffer& gb, AsyncSerial& port, const StringRef& reply) THROWS(GCodeException)
{
	if (!inBody)
	{
		gb.MustSee('O');
		const uint32_t off = gb.GetUIValue();
		gb.MustSee('L');
		const uint32_t len = gb.GetUIValue();
		gb.MustSee('C');
		const uint32_t crc = gb.GetUIValue();

		bodyOffset = off;
		if (len == 0 || len > PanelPrintChunkMax)
		{
			// The one refusal that cannot drain: we will not accept a body this size, so we cannot know how much to
			// swallow. The panel must never send it; if it does, the body is parsed as G-code and rejected line by line.
			ReplyNak(reply, "length");
			return GCodeResult::ok;
		}
		bodyLength = len;
		bodyCrc = crc;
		bodyReceived = 0;
		bodyLastByteTime = millis();
		inBody = true;
	}

	// Take whatever has arrived. A body streams in at ~11 bytes/ms at 115200 and this runs every main-loop
	// iteration, so it normally completes in a handful of calls.
	const uint32_t now = millis();
	bool gotAny = false;
	while (bodyReceived < bodyLength)
	{
		const int c = port.read();
		if (c < 0)
		{
			break;
		}
		body[bodyReceived++] = (uint8_t)c;
		gotAny = true;
	}
	if (gotAny)
	{
		bodyLastByteTime = now;
	}
	if (bodyReceived < bodyLength)
	{
		if (now - bodyLastByteTime > PanelPrintBodyTimeout)
		{
			// The tail of the body is not coming. Give the channel back to the parser and say so; the panel resends.
			// Its resend must not start before this fires - see the timeout pair in the protocol document.
			inBody = false;
			ReplyNak(reply, "timeout");
			return GCodeResult::ok;
		}
		return GCodeResult::notFinished;
	}
	inBody = false;

	// The body is in. Everything below drained it, so the channel is in sync whatever the verdict.
	if (state != State::receiving || writer == nullptr)
	{
		ReplyNak(reply, "nostream");
		return GCodeResult::ok;
	}
	if (bodyOffset != committed || bodyOffset + bodyLength > declaredSize)
	{
		ReplyNak(reply, "order");
		return GCodeResult::ok;
	}

	CRC32 chunkCrc;
	chunkCrc.Reset();
	chunkCrc.Update(reinterpret_cast<const char *_ecv_array>(body), bodyLength);
	if (chunkCrc.Get() != bodyCrc)
	{
		ReplyNak(reply, "crc");
		return GCodeResult::ok;
	}

	// Write and SYNC. The sync is what turns these bytes into 'committed': FatFs is in tiny mode here, so a reader
	// sees a synced sector through the same volume window with no stale per-file buffer in the way.
	if (!writer->Seek(committed) || !writer->Write(body, bodyLength) || !writer->Flush())
	{
		ReplyNak(reply, "write");
		return GCodeResult::ok;
	}
	fileCrc.Update(reinterpret_cast<const char *_ecv_array>(body), bodyLength);
	committed += bodyLength;

	if (committed == declaredSize)
	{
		writer->Close();
		writer = nullptr;
		state = State::complete;
		// The file is whole. The metadata parse that stopped at the frontier - footer fields, the rest of the
		// thumbnail list - can now run over all of it.
		if (gatingPrint)
		{
			reprap.GetPrintMonitor().ReparseFileInfo();
		}
	}

	reply.printf("{\"pfile\":{\"ack\":%" PRIu32 ",\"committed\":%" PRIu32 "}}", bodyOffset, (uint32_t)committed);
	return GCodeResult::ok;
}

// M1762 S1 finish and verify; M1762 S0 abort and delete; bare M1762 reports.
GCodeResult PanelPrintStream::Finish(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (!gb.Seen('S'))
	{
		reply.printf("{\"pfile\":{\"active\":%s,\"path\":\"%s\",\"size\":%" PRIu32 ",\"committed\":%" PRIu32 ",\"printing\":%s}}",
						(state != State::idle) ? "true" : "false", path.c_str(), (uint32_t)declaredSize, (uint32_t)committed,
						gatingPrint ? "true" : "false");
		return GCodeResult::ok;
	}

	const uint32_t s = gb.GetUIValue();
	if (s == 0)
	{
		if (gatingPrint)
		{
			reply.copy("{\"pfile\":{\"aborted\":false,\"reason\":\"printing\"}}");
			return GCodeResult::ok;
		}
		Discard(true);
		reply.copy("{\"pfile\":{\"aborted\":true}}");
		return GCodeResult::ok;
	}

	if (state == State::idle)
	{
		reply.copy("{\"pfile\":{\"done\":false,\"reason\":\"nostream\"}}");
	}
	else if (committed != declaredSize)
	{
		reply.printf("{\"pfile\":{\"done\":false,\"reason\":\"short\",\"committed\":%" PRIu32 "}}", (uint32_t)committed);
	}
	else if (fileCrc.Get() != declaredCrc)
	{
		// Every chunk passed its own CRC, so this means a write went wrong somewhere. Not deleted while it is being
		// printed - the print has been consuming chunk-verified data and the panel decides whether to cancel.
		if (!gatingPrint)
		{
			Discard(true);
		}
		reply.copy("{\"pfile\":{\"done\":false,\"reason\":\"crc\"}}");
	}
	else
	{
		reply.copy("{\"pfile\":{\"done\":true}}");
	}
	return GCodeResult::ok;
}

void PanelPrintStream::PrintStarting(const char *_ecv_array _ecv_null printingFilename) noexcept
{
	gatingPrint = (state != State::idle && printingFilename != nullptr && StringEqualsIgnoreCase(printingFilename, path.c_str()));
	lastStallReport = 0;
}

void PanelPrintStream::PrintStopped() noexcept
{
	gatingPrint = false;
}

// How many bytes from 'pos' the job reader may take. Only meaningful while gatingPrint is set.
size_t PanelPrintStream::ReadableFrom(FilePosition pos) noexcept
{
	if (state != State::receiving)
	{
		return SIZE_MAX;
	}
	if (pos < committed)
	{
		return (size_t)(committed - pos);
	}

	// Starved. Say so, but not every spin: the reader is here thousands of times a second while it waits.
	const uint32_t now = millis();
	if (lastStallReport == 0 || now - lastStallReport > PanelPrintStallReportInterval)
	{
		lastStallReport = now;
		reprap.GetPlatform().MessageF(WarningMessage, "Waiting for the panel to send more of %s (%" PRIu32 " of %" PRIu32 " bytes so far)\n",
										path.c_str(), (uint32_t)committed, (uint32_t)declaredSize);
	}
	return 0;
}

bool PanelPrintStream::IsStreamingFile(const char *_ecv_array filePath) const noexcept
{
	return state == State::receiving && !path.IsEmpty() && StringEqualsIgnoreCase(filePath, path.c_str());
}

#endif	// SUPPORT_PANEL_PRINT

// End
