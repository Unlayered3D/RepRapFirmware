/*
 * PanelOtaUpdater.cpp - stream a firmware image out of an aux port to an Unlayered panel.
 *
 * Unlayered3D fork. Protocol: Developer-documentation/panel-firmware-push.md.
 */

#include "PanelOtaUpdater.h"

#if SUPPORT_PANEL_OTA

#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <Storage/CRC32.h>
#include <Storage/FileStore.h>
#include <General/SafeStrtod.h>

PanelOtaUpdater::PanelOtaUpdater() noexcept
	: serialChannel(NumSerialChannels + 1)
	, firmwareFile(nullptr)
	, currentInterruptCallbackFn(nullptr)
	, fileSize(0)
	, imageCrc(0)
	, bytesHashed(0)
	, bytesSent(0)
	, seq(0)
	, chunkLength(0)
	, framePos(0)
	, headerLength(0)
	, whenStartedWaiting(0)
	, lastPercentReported(0)
	, retries(0)
	, state(PushState::idle)
{
}

void PanelOtaUpdater::Start(const StringRef& filenameRef, const uint32_t serialChan) noexcept
{
	if (state.RawValue() != PushState::idle)
	{
		return;
	}

	serialChannel = serialChan;
	filename.copy(filenameRef.IsEmpty() ? PANEL_OTA_FIRMWARE_FILE : filenameRef.c_str());

	firmwareFile = reprap.GetPlatform().OpenFile(FIRMWARE_DIRECTORY, filename.c_str(), OpenMode::read);
	if (firmwareFile == nullptr)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Can't open file %s\n", filename.c_str());
		state = PushState::done;
		return;
	}

	fileSize = (uint32_t)firmwareFile->Length();
	if (fileSize == 0 || fileSize > PanelOtaMaxImageSize)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Panel firmware file %s is empty or too large\n", filename.c_str());
		state = PushState::done;
		return;
	}

	/*
	 * Take the aux port out of the emergency-stop path for the duration. Two reasons, and the
	 * second is the load-bearing one:
	 *
	 * Drives and heaters are already off - M997 does that before it gets here - so there is
	 * nothing left for an emergency stop to stop, and a spurious trigger part way through a
	 * two-minute transfer would be pure loss.
	 *
	 * And GCodes::Spin() DRAINS THIS PORT when emergencyStopCommanded is set
	 * (`while (GetAsyncPort(0)->read() >= 0) {}`). That would swallow the panel's
	 * acknowledgements out from under CollectReply(), and the transfer would stall on a
	 * timeout with no indication of where the bytes went.
	 */
	AsyncSerial *_ecv_null const auxPort = GetAuxPort();
	if (auxPort != nullptr)
	{
		currentInterruptCallbackFn = auxPort->SetInterruptCallback(nullptr);
	}

	bytesHashed = 0;
	bytesSent = 0;
	seq = 0;
	retries = 0;
	lastPercentReported = 0;
	imageCrc = 0;
	headerLength = 0;					// "no frame built yet" - see the announce state
	chunkLength = 0;
	framePos = 0;
	replyLine.Clear();

	/*
	 * NOTE the absence of a "starting" message here, which is the obvious thing to add and is
	 * wrong. GCodes4 does not set isPushingPanelOta until Start() has returned, so anything
	 * reported from here is queued on the aux output stack BEFORE the gate closes, and
	 * Platform::FlushMessages() then writes it down the same wire the transfer is about to use.
	 * Everything reported from Spin() is safe, because by then AppendAuxReply drops aux traffic
	 * outright. The two error paths above are the exception and are fine: they abandon the
	 * transfer, so nothing follows them onto the wire.
	 */
	state = PushState::hashing;
}

/*
 * One slice of work. Every branch either finishes quickly or hands control back, because this
 * runs on the main loop.
 */
void PanelOtaUpdater::Spin() noexcept
{
	AsyncSerial *_ecv_null const auxPort = GetAuxPort();
	if (auxPort == nullptr && state.RawValue() != PushState::idle && state.RawValue() != PushState::done)
	{
		Fail("aux port unavailable");
		return;
	}

	switch (state.RawValue())
	{
	case PushState::hashing:
		{
			/*
			 * The whole-image CRC goes in the announce, BEFORE any content - the same ordering
			 * the panel uses when it uploads to us, and for the same reason: the receiver then
			 * knows what it is checking against without having to trust the sender afterwards.
			 * That costs a full read of the file up front, so it is done a chunk per Spin
			 * rather than in one blocking pass.
			 */
			if (bytesHashed == 0)
			{
				hashCrc.Reset();
				reprap.GetPlatform().MessageF(GenericMessage, "Sending %s (%" PRIu32 " bytes) to the panel on aux %" PRIu32 "\n",
												filename.c_str(), fileSize, serialChannel - FirstAuxChannel);
			}

			const uint32_t toRead = min<uint32_t>(PanelOtaChunkSize, fileSize - bytesHashed);
			const int n = _ecv_not_null(firmwareFile)->Read(buffer, toRead);
			if (n <= 0 || (uint32_t)n != toRead)
			{
				Fail("read error while hashing");
				return;
			}
			hashCrc.Update(reinterpret_cast<const char *_ecv_array>(buffer), toRead);
			bytesHashed += toRead;

			if (bytesHashed == fileSize)
			{
				imageCrc = hashCrc.Get();
				if (!_ecv_not_null(firmwareFile)->Seek(0))
				{
					Fail("cannot rewind the file");
					return;
				}
				state = PushState::announce;
			}
		}
		break;

	case PushState::announce:
		{
			/*
			 * headerLength == 0 means "not built yet". It matters: PushFrame() only hands over
			 * what the UART will take without blocking, so this state can be re-entered with the
			 * frame half sent - and rebuilding it then, with framePos reset, would retransmit
			 * the bytes already gone and produce a line the far end cannot parse.
			 */
			if (headerLength == 0)
			{
				// A bare newline first, so anything half-parsed at the far end is terminated
				// before the announce arrives rather than swallowing it.
				SafeSnprintf(header, sizeof(header),
								"\n{\"fwPush\":{\"size\":%" PRIu32 ",\"crc\":%" PRIu32 ",\"chunk\":%" PRIu32 ",\"name\":\"%s\"}}\n",
								fileSize, imageCrc, PanelOtaChunkSize, filename.c_str());
				headerLength = (uint32_t)strlen(header);
				chunkLength = 0;
				framePos = 0;
			}
			if (PushFrame())
			{
				headerLength = 0;
				state = PushState::waitReady;
				BeginWait();
			}
		}
		break;

	case PushState::waitReady:
		if (CollectReply())
		{
			if (replyLine.Equals("fwPush ready"))
			{
				replyLine.Clear();
				state = PushState::loadChunk;
			}
			else if (strncmp(replyLine.c_str(), "fwPush abort", 12) == 0)
			{
				Fail(replyLine.c_str());
			}
			else
			{
				// Anything else on this channel is not for us. A real PanelDue, for instance,
				// never answers at all - which is why this waits rather than failing here.
				replyLine.Clear();
			}
		}
		else if (WaitedTooLong(PanelOtaReadyTimeout))
		{
			Fail("no response from the panel");
		}
		break;

	case PushState::loadChunk:
		{
			chunkLength = min<uint32_t>(PanelOtaChunkSize, fileSize - bytesSent);
			const int n = _ecv_not_null(firmwareFile)->Read(buffer, chunkLength);
			if (n <= 0 || (uint32_t)n != chunkLength)
			{
				Fail("read error");
				return;
			}

			CRC32 chunkCrc;
			chunkCrc.Reset();
			chunkCrc.Update(reinterpret_cast<const char *_ecv_array>(buffer), chunkLength);

			SafeSnprintf(header, sizeof(header),
							"{\"fwData\":{\"seq\":%" PRIu32 ",\"len\":%" PRIu32 ",\"crc\":%" PRIu32 "}}\n",
							seq, chunkLength, chunkCrc.Get());
			headerLength = (uint32_t)strlen(header);
			framePos = 0;
			state = PushState::sendChunk;
		}
		break;

	case PushState::sendChunk:
		if (PushFrame())
		{
			state = PushState::waitAck;
			BeginWait();
		}
		break;

	case PushState::waitAck:
		if (CollectReply())
		{
			if (strncmp(replyLine.c_str(), "fwPush ack ", 11) == 0)
			{
				const uint32_t acked = StrToU32(replyLine.c_str() + 11);
				if (acked != seq)
				{
					// Not tolerated. Accepting an ACK for the wrong chunk means writing an
					// image with a hole in it, which then passes its own header check at the
					// far end and boots into something that was never built.
					Fail("acknowledgement out of sequence");
					return;
				}
				replyLine.Clear();
				bytesSent += chunkLength;
				++seq;
				retries = 0;

				const uint32_t percent = (bytesSent * 100) / fileSize;
				if (percent >= lastPercentReported + 20)
				{
					lastPercentReported = percent;
					reprap.GetPlatform().MessageF(GenericMessage, "Panel firmware: %" PRIu32 "%%\n", percent);
				}

				if (bytesSent == fileSize)
				{
					headerLength = 0;			// sendDone builds its own frame; see the announce state
					state = PushState::sendDone;
				}
				else
				{
					state = PushState::loadChunk;
				}
			}
			else if (strncmp(replyLine.c_str(), "fwPush nak", 10) == 0)
			{
				replyLine.Clear();
				if (++retries >= PanelOtaMaxRetries)
				{
					Fail("too many failed chunks");
					return;
				}
				framePos = 0;					// resend the same frame unchanged
				state = PushState::sendChunk;
			}
			else if (strncmp(replyLine.c_str(), "fwPush abort", 12) == 0)
			{
				Fail(replyLine.c_str());
			}
			else
			{
				replyLine.Clear();
			}
		}
		else if (WaitedTooLong(PanelOtaAckTimeout))
		{
			if (++retries >= PanelOtaMaxRetries)
			{
				Fail("the panel stopped acknowledging");
				return;
			}
			replyLine.Clear();
			framePos = 0;
			state = PushState::sendChunk;
		}
		break;

	case PushState::sendDone:
		if (headerLength == 0)
		{
			SafeSnprintf(header, sizeof(header), "{\"fwPush\":{\"done\":true}}\n");
			headerLength = (uint32_t)strlen(header);
			chunkLength = 0;
			framePos = 0;
		}
		if (PushFrame())
		{
			headerLength = 0;
			state = PushState::waitDone;
			BeginWait();
		}
		break;

	case PushState::waitDone:
		if (CollectReply())
		{
			if (replyLine.Equals("fwPush done"))
			{
				reprap.GetPlatform().Message(GenericMessage, "Panel firmware sent. It takes effect when the panel restarts.\n");
				state = PushState::done;
			}
			else if (strncmp(replyLine.c_str(), "fwPush abort", 12) == 0)
			{
				Fail(replyLine.c_str());
			}
			else
			{
				replyLine.Clear();
			}
		}
		else if (WaitedTooLong(PanelOtaDoneTimeout))
		{
			Fail("the panel did not confirm the image");
		}
		break;

	case PushState::done:
		{
			AsyncSerial *_ecv_null const port = GetAuxPort();
			if (port != nullptr && currentInterruptCallbackFn != nullptr)
			{
				port->SetInterruptCallback(currentInterruptCallbackFn);
			}
			currentInterruptCallbackFn = nullptr;

			if (firmwareFile != nullptr)
			{
				firmwareFile->Close();
				firmwareFile = nullptr;
			}

			serialChannel = NumSerialChannels + 1;
			fileSize = bytesSent = bytesHashed = seq = chunkLength = framePos = headerLength = 0;
			retries = 0;
			replyLine.Clear();
			state = PushState::idle;
		}
		break;

	default:
		break;
	}
}

/*
 * Hand over as much of header[0..headerLength) followed by buffer[0..chunkLength) as the UART
 * will take right now. AsyncSerial::write() spins until the whole block is buffered, so it is
 * only ever called with a length that already fits - the rest waits for the next Spin.
 */
bool PanelOtaUpdater::PushFrame() noexcept
{
	AsyncSerial *_ecv_null const port = GetAuxPort();
	if (port == nullptr)
	{
		return false;
	}

	const uint32_t frameLength = headerLength + chunkLength;
	size_t space = port->canWrite();
	while (space != 0 && framePos < frameLength)
	{
		const uint8_t *_ecv_array src;
		size_t available;
		if (framePos < headerLength)
		{
			src = reinterpret_cast<const uint8_t *_ecv_array>(header) + framePos;
			available = headerLength - framePos;
		}
		else
		{
			src = buffer + (framePos - headerLength);
			available = frameLength - framePos;
		}

		const size_t n = min<size_t>(space, available);
		port->write(src, n);
		framePos += (uint32_t)n;
		space -= n;
	}

	return framePos == frameLength;
}

/*
 * Read whatever has arrived into replyLine, a line at a time. Returns true when a complete,
 * non-empty line is sitting in replyLine; the caller consumes it and Clear()s it.
 */
bool PanelOtaUpdater::CollectReply() noexcept
{
	AsyncSerial *_ecv_null const port = GetAuxPort();
	if (port == nullptr)
	{
		return false;
	}

	while (port->available() > 0)
	{
		const int c = port->read();
		if (c < 0)
		{
			break;
		}
		if (c == '\n' || c == '\r')
		{
			if (replyLine.strlen() != 0)
			{
				return true;
			}
			continue;						// ignore blank lines rather than treating them as replies
		}
		// An overlong line is left truncated. It then matches nothing and the state times out,
		// which is the correct outcome for a reply we could not read.
		(void)replyLine.cat((char)c);
	}
	return false;
}

void PanelOtaUpdater::Fail(const char *_ecv_array why) noexcept
{
	reprap.GetPlatform().MessageF(ErrorMessage, "Panel firmware push failed: %s\n", why);

	// Tell the far end, if it is still listening, so it releases its slot rather than waiting
	// out its own timeout with an OTA half open.
	AsyncSerial *_ecv_null const port = GetAuxPort();
	if (port != nullptr)
	{
		const char *_ecv_array const abortMsg = "\n{\"fwPush\":{\"abort\":true}}\n";
		const size_t len = strlen(abortMsg);
		if (port->canWrite() >= len)
		{
			port->write(reinterpret_cast<const uint8_t *_ecv_array>(abortMsg), len);
		}
	}

	state = PushState::done;
}

void PanelOtaUpdater::BeginWait() noexcept
{
	whenStartedWaiting = millis();
}

bool PanelOtaUpdater::WaitedTooLong(uint32_t timeout) const noexcept
{
	return millis() - whenStartedWaiting > timeout;
}

AsyncSerial *_ecv_null PanelOtaUpdater::GetAuxPort() const noexcept
{
	return (serialChannel < FirstAuxChannel || serialChannel >= NumSerialChannels)
			? nullptr
			: reprap.GetPlatform().GetAsyncPort(serialChannel - FirstAuxChannel);
}

#endif	// SUPPORT_PANEL_OTA

// End
