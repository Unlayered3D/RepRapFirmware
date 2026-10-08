/*
 * PiezoLog.cpp
 *
 *  Created on: 3 Oct 2026
 *      Author: Unlayered3D
 *
 *  See PiezoLog.h and Developer-documentation/Piezo-support-log.md.
 */

#include "PiezoLog.h"

#if SUPPORT_PIEZO_LOG

#include <atomic>
#include <cctype>
#include <ctime>
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <Storage/MassStorage.h>
#include <Storage/FileStore.h>
#include <Endstops/ZProbe.h>
#include <Movement/Move.h>
#include <Heating/Heat.h>
#include <GCodes/GCodes.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Version.h>

namespace PiezoLog
{
	// ---- Shared between the tick ISR and Spin ----
	// Each buffer is owned by the ISR while bufferFull[i] is false and by Spin while it is true. Both are single bytes,
	// written by one side each, so no read-modify-write races and no critical sections.
	static Block buffers[NumBuffers];
	static volatile bool bufferFull[NumBuffers];
	static volatile bool flushRequested = false;		// Spin asks the ISR to close the partial block (before a snapshot)
	static volatile bool resetRequested = false;		// Spin asks the ISR to restart at buffer 0 (after re-enabling)
	static volatile bool enabled = true;
	static volatile bool probingFlag = false;			// for the non-differential analog types, which have no detector state
	static volatile bool trigEdgePending = false;
	static volatile uint32_t trigEdgeMillis = 0;
	static volatile int8_t trigEdgeSign = 0;
	static volatile int8_t trigSeenWhileProbing = 0;	// set by the ISR on a trigger edge during a probing move, cleared at arm

	// ---- Tick ISR only ----
	static uint8_t fillIndex = 0;
	static uint16_t fillCount = 0;
	static uint16_t droppedInIsr = 0;
	static int8_t lastTrig = 0;

	// ---- Spin (Main task) only ----
	enum class State : uint8_t { starting, running, failed, disabled };
	static State state = State::starting;
	static FileStore *_ecv_null ringFile = nullptr;
	static uint32_t ringBlockIndex = 0;			// where the next block goes in ring.bin
	static uint32_t ringFileBlocks = 0;			// how many blocks ring.bin holds so far
	static uint32_t seq = 0;
	static uint8_t writeIndex = 0;
	static uint32_t retryAt = 0;
	static bool initialised = false;
	static unsigned int nextSnapshot = 1;
	static uint32_t blocksWritten = 0, totalDropped = 0, snapshotsTaken = 0;
	static const char *_ecv_array lastError = "none";
	static const char *_ecv_array _ecv_null lastProbeError = nullptr;
	static bool snapPending = false, snapFlushIssued = false;
	static uint32_t snapAt = 0, lastAutoSnap = 0;
	static bool haveAutoSnapped = false;
	static String<40> snapReason;
	static unsigned int lastSnapshotNumber = 0;

	constexpr uint32_t StartDelayMillis = 4000;		// let the SD card mount and config.g run first
	constexpr uint32_t RetryMillis = 10000;
	constexpr uint32_t ProbeFailSnapDelay = 1000;	// keep 1 s of aftermath in the window
	constexpr uint32_t MinAutoSnapInterval = 10000;	// one bad homing that retries must not eat every snapshot
	constexpr FilePosition MaxEventsLogSize = 256 * 1024;
	constexpr size_t MaxLoggedAxes = 5;

	static const char *_ecv_array const LogDir = "0:/logs/piezo/";
	static const char *_ecv_array const LogDirNoSlash = "0:/logs/piezo";
	static const char *_ecv_array const RingPath = "0:/logs/piezo/ring.bin";
	static const char *_ecv_array const EventsPath = "0:/logs/piezo/events.log";
	static const char *_ecv_array const OldEventsPath = "0:/logs/piezo/events.1.log";

	// Events raised from task code that must not touch the SD card itself (SetProbing runs just before a probing move)
	enum class EventKind : uint8_t { arm, stop, fail };
	struct Event
	{
		uint32_t ms;
		const char *_ecv_array _ecv_null msg;	// fail: a string literal, as GCodeException requires of SetError messages
		float pos[MaxLoggedAxes];
		int16_t thr, level;
		EventKind kind;
		int8_t trig;
	};
	constexpr size_t MaxEvents = 6;
	static Event events[MaxEvents];
	static size_t numEvents = 0;
	static uint32_t eventsLost = 0;

	static bool IsLoggedType(ZProbeType t) noexcept
	{
		return t == ZProbeType::analog || t == ZProbeType::alternateAnalog
#if SUPPORT_DIFFERENTIAL_PROBES
			|| t == ZProbeType::differentialAnalog
#endif
			;
	}
}

using namespace PiezoLog;

// ===================== Tick ISR =====================

// One store and an index increment per tick in the common case: this runs inside Platform::Tick, where every cycle is step latency.
void PiezoLog::TickSample(const ZProbe& zp) noexcept
{
	if (!enabled || !IsLoggedType(zp.GetProbeType()))
	{
		return;
	}

	if (resetRequested)
	{
		fillIndex = 0;
		fillCount = 0;
		droppedInIsr = 0;
		resetRequested = false;
	}

	if (flushRequested)
	{
		if (fillCount != 0)
		{
			buffers[fillIndex].count = fillCount;
			std::atomic_signal_fence(std::memory_order_seq_cst);		// the block must be complete before Spin may see it as full
			bufferFull[fillIndex] = true;
			fillIndex = (fillIndex + 1) % NumBuffers;
			fillCount = 0;
		}
		flushRequested = false;
	}

	int32_t raw = zp.GetRawReading();
	raw = (raw < 0) ? 0 : (raw > (int32_t)RawMask) ? (int32_t)RawMask : raw;
	uint16_t w = (uint16_t)raw;

	int8_t t;
#if SUPPORT_DIFFERENTIAL_PROBES
	if (zp.IsDifferential())
	{
		w |= zp.GetDiffLogFlags();
		t = (int8_t)zp.GetDiffTrigger();
	}
	else
#endif
	{
		if (probingFlag) { w |= FlagProbing; }
		t = (zp.Stopped()) ? 1 : 0;
		if (t != 0) { w |= FlagTriggered; }
	}

	// Only edges during a probing move become events: homing's shake test triggers dozens of times, and the ring has those anyway
	if (t != 0 && lastTrig == 0 && (w & FlagProbing) != 0)
	{
		trigSeenWhileProbing = t;			// a type 3 reading has usually decayed below threshold again by the time SetProbing(false) runs
		if (!trigEdgePending)
		{
			trigEdgeMillis = millis();
			trigEdgeSign = t;
			trigEdgePending = true;
		}
	}
	lastTrig = t;

	Block& b = buffers[fillIndex];
	if (fillCount == 0)
	{
		if (bufferFull[fillIndex])
		{
			// The SD writer has fallen behind and every buffer is full: count the loss so the decoder can show the gap
			if (droppedInIsr != 0xFFFF) { droppedInIsr = droppedInIsr + 1; }
			return;
		}
		b.millis = millis();
		b.dropped = droppedInIsr;
		droppedInIsr = 0;
	}
	b.samples[fillCount] = w;
	++fillCount;
	if (fillCount == SamplesPerBlock)
	{
		b.count = SamplesPerBlock;
		std::atomic_signal_fence(std::memory_order_seq_cst);
		bufferFull[fillIndex] = true;
		fillIndex = (fillIndex + 1) % NumBuffers;
		fillCount = 0;
	}
}

// ===================== Main task =====================

// Called from LocalZProbe::SetProbing, immediately before or after a probing move. Queue only: an SD write here
// would sit between arming the detector and starting the move, eating the blanking window.
void PiezoLog::NoteProbing(bool isProbing) noexcept
{
	const int8_t seen = trigSeenWhileProbing;
	if (isProbing)
	{
		trigSeenWhileProbing = 0;			// before the flag goes up, so the ISR cannot latch an edge that is then wiped
	}
	probingFlag = isProbing;
	if (numEvents == MaxEvents)
	{
		++eventsLost;
		return;
	}

	Event& ev = events[numEvents];
	ev.ms = millis();
	ev.msg = nullptr;
	ev.kind = (isProbing) ? EventKind::arm : EventKind::stop;
	float coords[MaxAxes];
	reprap.GetMove().GetCurrentMachinePosition(coords, 0);	// the planned position: the start point at arm, the stop point after the move
	for (size_t i = 0; i < MaxLoggedAxes; ++i)
	{
		ev.pos[i] = (i < MaxAxes) ? coords[i] : 0.0;
	}
	const ZProbe& zp = reprap.GetPlatform().GetEndstops().GetDefaultZProbeFromISR();
	ev.thr = (int16_t)constrain<int32_t>(zp.GetTargetAdcValue(), -32000, 32000);
	ev.trig = (isProbing) ? 0 : seen;		// stop: did it trigger at any point during the move, not whether it still reads triggered now
#if SUPPORT_DIFFERENTIAL_PROBES
	if (zp.IsDifferential())
	{
		ev.level = (int16_t)zp.GetDiffLevel();
	}
	else
#endif
	{
		ev.level = (int16_t)constrain<int32_t>(zp.GetRawReading(), -32000, 32000);
	}
	++numEvents;
}

// Called for every G-code error. Only the probe failures are interesting; they are all string literals in GCodes4/GCodes6.
void PiezoLog::NoteError(const char *_ecv_array msg) noexcept
{
	if (msg == nullptr || strstr(msg, "robe") == nullptr || (strstr(msg, "riggered") == nullptr && strstr(msg, "contact") == nullptr))
	{
		return;
	}

	lastProbeError = msg;
	if (numEvents < MaxEvents)
	{
		Event& ev = events[numEvents++];
		ev.ms = millis();
		ev.msg = msg;
		ev.kind = EventKind::fail;
		ev.trig = 0;
		ev.thr = ev.level = 0;
		for (float& p : ev.pos) { p = 0.0; }
	}
	else
	{
		++eventsLost;
	}

	const uint32_t now = millis();
	if (!snapPending && (!haveAutoSnapped || now - lastAutoSnap >= MinAutoSnapInterval))
	{
		snapPending = true;
		snapFlushIssued = false;
		snapAt = now + ProbeFailSnapDelay;
		snapReason.copy("probe-fail");
		lastAutoSnap = now;
		haveAutoSnapped = true;
	}
}

// Format "<millis> <ISO time or -> " for an event that happened at 'ms'
static void AppendTimePrefix(const StringRef& s, uint32_t ms) noexcept
{
	s.catf("%" PRIu32 " ", ms);
	const Platform& p = reprap.GetPlatform();
	if (p.IsDateTimeSet())
	{
		const time_t t = p.GetDateTime() - (time_t)((millis() - ms) / 1000);
		tm tmb;
		if (gmtime_r(&t, &tmb) != nullptr)
		{
			s.catf("%04u-%02u-%02uT%02u:%02u:%02u ", tmb.tm_year + 1900, tmb.tm_mon + 1, tmb.tm_mday, tmb.tm_hour, tmb.tm_min, tmb.tm_sec);
			return;
		}
	}
	s.cat("- ");
}

static void AppendPosition(const StringRef& s, const float pos[]) noexcept
{
	const char *_ecv_array letters = reprap.GetGCodes().GetAxisLetters();
	const size_t n = min<size_t>(reprap.GetGCodes().GetVisibleAxes(), MaxLoggedAxes);
	for (size_t i = 0; i < n; ++i)
	{
		s.catf(" %c=%.3f", tolower(letters[i]), (double)pos[i]);
	}
}

// Open events.log for appending, rolling it over first if it has grown past the limit. Returns null on failure.
static FileStore *_ecv_null OpenEventsLog() noexcept
{
	FileStore *_ecv_null f = MassStorage::OpenFile(EventsPath, OpenMode::append, 0);
	if (f != nullptr && f->Length() > MaxEventsLogSize)
	{
		f->Close();
		(void)MassStorage::Rename(EventsPath, OldEventsPath, true, false);
		f = MassStorage::OpenFile(EventsPath, OpenMode::append, 0);
	}
	return f;
}

static void WriteEventLine(const char *_ecv_array line) noexcept
{
	FileStore *_ecv_null const f = OpenEventsLog();
	if (f != nullptr)
	{
		f->Write(line);
		f->Write('\n');
		f->Close();
	}
}

static void FlushEvents() noexcept
{
	const bool haveTrig = trigEdgePending;
	if (numEvents == 0 && !haveTrig)
	{
		return;
	}

	FileStore *_ecv_null const f = OpenEventsLog();
	String<StringLength256> line;
	if (haveTrig)
	{
		line.Clear();
		AppendTimePrefix(line.GetRef(), trigEdgeMillis);
		line.catf("TRIG sign=%c\n", (trigEdgeSign < 0) ? '-' : '+');
		if (f != nullptr) { f->Write(line.c_str()); }
		trigEdgePending = false;
	}

	for (size_t i = 0; i < numEvents; ++i)
	{
		const Event& ev = events[i];
		line.Clear();
		AppendTimePrefix(line.GetRef(), ev.ms);
		switch (ev.kind)
		{
		case EventKind::arm:
			line.cat("ARM");
			AppendPosition(line.GetRef(), ev.pos);
			line.catf(" thr=%d level=%d", (int)ev.thr, (int)ev.level);
			break;

		case EventKind::stop:
			line.catf("STOP trig=%d sign=%c", (ev.trig != 0) ? 1 : 0, (ev.trig < 0) ? '-' : '+');
			AppendPosition(line.GetRef(), ev.pos);
			line.catf(" level=%d", (int)ev.level);
			break;

		case EventKind::fail:
			line.catf("FAIL msg=\"%s\"", (ev.msg != nullptr) ? ev.msg : "");
			break;
		}
		line.cat('\n');
		if (f != nullptr) { f->Write(line.c_str()); }
	}
	if (eventsLost != 0 && f != nullptr)
	{
		line.printf("%" PRIu32 " - LOST n=%" PRIu32 "\n", millis(), eventsLost);
		f->Write(line.c_str());
		eventsLost = 0;
	}
	numEvents = 0;
	if (f != nullptr)
	{
		f->Close();
	}
}

static void SnapshotPath(const StringRef& s, unsigned int n, const char *_ecv_array ext) noexcept
{
	s.printf("%ssnap-%04u.%s", LogDir, n, ext);
}

// Find the highest existing snapshot number so numbering survives reboots, and trim to MaxSnapshots
static void ScanSnapshots() noexcept
{
	unsigned int highest = 0;
	FileInfo info;
	if (MassStorage::FindFirst(LogDirNoSlash, info))
	{
		do
		{
			const char *_ecv_array name = info.fileName.c_str();
			if (!info.isDirectory && StringStartsWith(name, "snap-") && isdigit(name[5]))
			{
				const unsigned int n = (unsigned int)StrToU32(name + 5);
				if (n > highest) { highest = n; }
			}
		} while (MassStorage::FindNext(info));
		MassStorage::AbandonFindNext();
	}
	nextSnapshot = highest + 1;
	if (nextSnapshot > 9999) { nextSnapshot = 1; }		// the names are 4 digits; wrapping only disturbs retention once
}

static void DeleteSnapshot(unsigned int n) noexcept
{
	String<MaxFilenameLength> path;
	SnapshotPath(path.GetRef(), n, "bin");
	(void)MassStorage::Delete(path.GetRef(), ErrorMessageMode::noMessage);
	SnapshotPath(path.GetRef(), n, "txt");
	(void)MassStorage::Delete(path.GetRef(), ErrorMessageMode::noMessage);
}

static void WriteSidecar(unsigned int n, const char *_ecv_array reason, bool previousBoot) noexcept
{
	String<MaxFilenameLength> path;
	SnapshotPath(path.GetRef(), n, "txt");
	FileStore *_ecv_null const f = MassStorage::OpenFile(path.c_str(), OpenMode::append, 0);	// not write mode: that would take the board's only FileWriteBuffer from an upload or a panel print
	if (f == nullptr)
	{
		return;
	}

	String<StringLength256> s;
	s.printf("reason: %s\nsnapshot: %u\nmillis: %" PRIu32 "\ntime: ", reason, n, millis());
	{
		String<StringLength50> t;
		AppendTimePrefix(t.GetRef(), millis());				// "<millis> <ISO or -> ": keep only the ISO part
		const char *_ecv_array _ecv_null const sp = strchr(t.c_str(), ' ');
		s.cat((sp != nullptr) ? sp + 1 : "-");
	}
	s.catf("\nfirmware: %s %s (%s)\n", FIRMWARE_NAME, VERSION, DateText);
	s.catf("format: PZL1, %u blocks of %u samples, 1 sample per ms\n", (unsigned int)RingBlocks, (unsigned int)SamplesPerBlock);
	f->Write(s.c_str());

	if (!previousBoot)
	{
		// The counters describe this boot, so they are meaningless for the ring recorded before the reboot
		s.printf("last seq: %" PRIu32 "\nsince boot: blocks %" PRIu32 ", dropped samples %" PRIu32 "\n",
					(seq == 0) ? 0 : seq - 1, blocksWritten, totalDropped);
		f->Write(s.c_str());
	}

	if (previousBoot)
	{
		// Everything below describes the machine now, not the boot this ring was recorded in
		f->Write("context: ring recorded before the last reboot; the probe configuration below is the current one\n");
	}

	const ZProbe& zp = reprap.GetPlatform().GetEndstops().GetDefaultZProbeFromISR();
	s.printf("probe type: %u\nthreshold (G31 P): %" PRIi32 "\n", (unsigned int)zp.GetProbeType(), zp.GetTargetAdcValue());
#if SUPPORT_DIFFERENTIAL_PROBES
	if (zp.IsDifferential())
	{
		s.cat("detector");
		zp.AppendDifferentialReport(s.GetRef());
		s.cat('\n');
	}
#endif
	f->Write(s.c_str());

	if (!previousBoot)
	{
		float coords[MaxAxes];
		reprap.GetMove().GetCurrentMachinePosition(coords, 0);	// the planned position: the start point at arm, the stop point after the move
		float pos[MaxLoggedAxes];
		for (size_t i = 0; i < MaxLoggedAxes; ++i) { pos[i] = (i < MaxAxes) ? coords[i] : 0.0; }
		s.copy("machine position:");
		AppendPosition(s.GetRef(), pos);
		s.catf("\ntemperatures: heater0 %.1f, heater1 %.1f\nprinting: %s\n",
				(double)reprap.GetHeat().GetHeaterTemperature(0), (double)reprap.GetHeat().GetHeaterTemperature(1),
				(reprap.GetGCodes().IsReallyPrinting()) ? "yes" : "no");
		if (lastProbeError != nullptr)
		{
			s.catf("last probe error: %s\n", lastProbeError);
		}
		f->Write(s.c_str());
	}
	f->Close();
}

// Freeze ring.bin as the next snapshot. The ring file must be closed.
static bool RenameRingToSnapshot(const char *_ecv_array reason, bool previousBoot) noexcept
{
	const unsigned int n = nextSnapshot;
	DeleteSnapshot(n);									// only exists after the numbers wrap
	String<MaxFilenameLength> path;
	SnapshotPath(path.GetRef(), n, "bin");
	if (!MassStorage::Rename(RingPath, path.c_str(), true, false))
	{
		lastError = "rename failed";
		return false;
	}
	WriteSidecar(n, reason, previousBoot);
	if (n > MaxSnapshots)
	{
		DeleteSnapshot(n - MaxSnapshots);
	}
	lastSnapshotNumber = n;
	nextSnapshot = (n >= 9999) ? 1 : n + 1;
	++snapshotsTaken;

	String<StringLength256> line;
	AppendTimePrefix(line.GetRef(), millis());
	line.catf("SNAP n=%u reason=\"%s\"", n, reason);
	WriteEventLine(line.c_str());
	return true;
}

static bool OpenRingFile() noexcept
{
	ringFile = MassStorage::OpenFile(RingPath, OpenMode::append, 0);	// append mode: no shared write buffer is tied up, and in-place writes work
	if (ringFile == nullptr)
	{
		lastError = "cannot open ring.bin";
		return false;
	}
	if (ringFile->Length() != 0)
	{
		(void)ringFile->Seek(0);
		(void)ringFile->Truncate();
	}
	ringBlockIndex = 0;
	ringFileBlocks = 0;
	return true;
}

static bool Start() noexcept
{
	if (!MassStorage::IsDriveMounted(0))
	{
		lastError = "SD card not mounted";
		return false;
	}
	if (!initialised)
	{
		if (!MassStorage::EnsurePath(RingPath, false))
		{
			lastError = "cannot create 0:/logs/piezo";
			return false;
		}
		ScanSnapshots();
		if (MassStorage::FileExists(RingPath))
		{
			(void)RenameRingToSnapshot("boot", true);		// the 30 s before the reboot: a crash or a power cut is exactly when it matters
		}
		String<StringLength256> line;
		AppendTimePrefix(line.GetRef(), millis());
		line.catf("BOOT fw=\"%s\"", VERSION);
		WriteEventLine(line.c_str());
		initialised = true;
	}
	return OpenRingFile();
}

static void CloseRing() noexcept
{
	if (ringFile != nullptr)
	{
		ringFile->Close();
		ringFile = nullptr;
	}
}

static bool WriteBlock(Block& b) noexcept
{
	std::atomic_signal_fence(std::memory_order_seq_cst);
	b.magic = BlockMagic;
	b.seq = seq;
	if (!ringFile->Seek(ringBlockIndex * sizeof(Block)) || !ringFile->Write(reinterpret_cast<const char *_ecv_array>(&b), sizeof(Block)))
	{
		lastError = "write failed";
		return false;
	}
	++seq;
	++blocksWritten;
	totalDropped += b.dropped;
	if (ringBlockIndex >= ringFileBlocks)
	{
		// The file grew, so the directory entry must be updated or a power cut loses the block. Only during the first 30 s.
		ringFileBlocks = ringBlockIndex + 1;
		(void)ringFile->Flush();
	}
	if (++ringBlockIndex == RingBlocks)
	{
		ringBlockIndex = 0;
	}
	return true;
}

static void Fail(uint32_t now) noexcept
{
	CloseRing();
	state = State::failed;
	retryAt = now + RetryMillis;
}

static bool WriteFullBlocks(uint32_t now) noexcept
{
	while (bufferFull[writeIndex])
	{
		if (!WriteBlock(buffers[writeIndex]))
		{
			Fail(now);
			return false;
		}
		bufferFull[writeIndex] = false;
		writeIndex = (writeIndex + 1) % NumBuffers;
	}
	return true;
}

void PiezoLog::Spin() noexcept
{
	const uint32_t now = millis();

	if (!enabled)
	{
		if (state != State::disabled)
		{
			CloseRing();
			state = State::disabled;
		}
		return;
	}

	if (state != State::running)
	{
		if (state == State::disabled)
		{
			state = State::starting;
		}
		if (now < StartDelayMillis || (state == State::failed && (int32_t)(now - retryAt) < 0))
		{
			return;			// the buffers fill and the ISR counts what it drops, so the gap shows up in the next block
		}
		if (!Start())
		{
			Fail(now);
			return;
		}
		state = State::running;
	}

	if (!WriteFullBlocks(now))
	{
		return;
	}

	FlushEvents();

	if (snapPending && (int32_t)(now - snapAt) >= 0)
	{
		if (!snapFlushIssued)
		{
			flushRequested = true;			// the ISR closes the partial block on its next tick
			snapFlushIssued = true;
			return;
		}
		if (flushRequested)
		{
			return;
		}
		if (!WriteFullBlocks(now))
		{
			snapPending = false;
			return;
		}
		CloseRing();
		(void)RenameRingToSnapshot(snapReason.c_str(), false);
		snapPending = false;
		snapFlushIssued = false;
		if (!OpenRingFile())
		{
			Fail(now);
		}
	}
}

// M1770 [E0|E1] [S"reason"]
GCodeResult PiezoLog::Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (gb.Seen('S'))
	{
		if (!gb.IsTimerRunning())
		{
			if (state != State::running)
			{
				reply.printf("Piezo log is not running (%s)", (state == State::disabled) ? "disabled" : lastError);
				return GCodeResult::error;
			}
			String<40> reason;
			gb.GetQuotedString(reason.GetRef(), true);
			snapReason.copy((reason.IsEmpty()) ? "manual" : reason.c_str());
			snapPending = true;
			snapFlushIssued = false;
			snapAt = millis();
			gb.StartTimer();
			return GCodeResult::notFinished;
		}
		if (snapPending && millis() - gb.WhenTimerStarted() < 5000)
		{
			return GCodeResult::notFinished;		// wait, so a macro or the slicer can rely on the snapshot existing when M1770 returns
		}
		gb.StopTimer();
		if (snapPending)
		{
			reply.copy("Piezo log snapshot timed out");
			return GCodeResult::error;
		}
		reply.printf("Piezo log snapshot %u saved (%ssnap-%04u.bin)", lastSnapshotNumber, LogDir, lastSnapshotNumber);
		return GCodeResult::ok;
	}

	if (gb.Seen('E'))
	{
		const bool en = gb.GetUIValue() != 0;
		if (en && !enabled)
		{
			// Restart both sides in step: the ISR is idle while disabled, so this cannot race it
			for (volatile bool& f : bufferFull) { f = false; }
			writeIndex = 0;
			flushRequested = false;
			resetRequested = true;
			snapPending = false;
		}
		enabled = en;
		return GCodeResult::ok;
	}

	Diagnostics(reply);
	return GCodeResult::ok;
}

void PiezoLog::Diagnostics(const StringRef& reply) noexcept
{
	const char *_ecv_array const stateText = (state == State::running) ? "running"
											: (state == State::disabled) ? "disabled"
												: (state == State::starting) ? "starting" : "failed";
	reply.catf("Piezo log: %s, blocks written %" PRIu32 ", dropped samples %" PRIu32 ", snapshots %" PRIu32 " (next %u), last error: %s",
				stateText, blocksWritten, totalDropped, snapshotsTaken, nextSnapshot, lastError);
}

#endif

// End
