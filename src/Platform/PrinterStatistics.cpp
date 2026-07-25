/*
 * PrinterStatistics.cpp
 *
 *  Created on: May 19, 2026
 *      Author: stedmaja
 *
 *  See PrinterStatistics.h for what is tracked and why it is correct on every kinematic.
 */

#include "PrinterStatistics.h"

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GCodes/GCodes.h>
#include <Movement/Move.h>
#include <Storage/MassStorage.h>

void PrinterStatistics::Init() noexcept
{
	Load();
	lastUpdateMs = lastSaveMs = millis();
}

// Called from RepRap::Spin(). Drains the per-drive wear accumulators, accumulates print
// time, and persists at most once every SaveIntervalMs.
void PrinterStatistics::Spin() noexcept
{
	const uint32_t now = millis();
	const uint32_t dtMs = now - lastUpdateMs;
	lastUpdateMs = now;

	// Motor travel. Move::GetAccumulatedWear does an atomic read-and-clear, so steps from a
	// segment still in flight are simply counted on the next call rather than being lost.
	Move& move = reprap.GetMove();
	for (size_t drive = 0; drive < MaxAxesPlusExtruders; ++drive)
	{
		const uint32_t steps = move.GetAccumulatedWear(drive);
		if (steps != 0)
		{
			driveMicrosteps[drive] += (uint64_t)steps;
			dirty = true;
		}
	}

	// Print time and job count. A job is counted on the idle->printing transition, so this
	// is jobs STARTED. armedForNewJob is only re-armed once we are neither printing nor
	// paused, which stops a pause/resume cycle from counting as a second job.
	const GCodes& gCodes = reprap.GetGCodes();
	if (gCodes.IsReallyPrinting())
	{
		if (armedForNewJob)
		{
			++lifetimePrintJobs;
			armedForNewJob = false;
		}

		// Carry the sub-second remainder rather than truncating it, so long prints made up
		// of many short Spin() intervals do not lose time.
		printMillisCarry += dtMs;
		if (printMillisCarry >= 1000)
		{
			lifetimePrintSeconds += printMillisCarry / 1000;
			printMillisCarry %= 1000;
		}
		dirty = true;
	}
	else if (gCodes.GetPauseState() == PauseState::notPaused)
	{
		armedForNewJob = true;
	}

	if (dirty && now - lastSaveMs >= SaveIntervalMs)
	{
		Save();
		lastSaveMs = now;
		dirty = false;
	}
}

// Write the statistics to a temp file and rename over the live one, so an interrupted
// write cannot leave a half-written file behind.
void PrinterStatistics::Save() noexcept
{
	FileStore * const file = reprap.GetPlatform().OpenSysFile(StatsTempFileName, OpenMode::write);
	if (file == nullptr)
	{
		debugPrintf("PrinterStatistics: failed to open %s\n", StatsTempFileName);
		return;
	}

	String<JsonBufferSize> buffer;
	buffer.printf("{\n  \"version\": %" PRIu32 ",\n"
				  "  \"lifetimePrintSeconds\": %" PRIu32 ",\n"
				  "  \"lifetimePrintJobs\": %" PRIu32 ",\n"
				  "  \"driveMicrosteps\": [",
				  FileFormatVersion, lifetimePrintSeconds, lifetimePrintJobs);
	for (size_t i = 0; i < MaxAxesPlusExtruders; ++i)
	{
		buffer.catf("%s%" PRIu64, (i == 0) ? "" : ", ", driveMicrosteps[i]);
	}
	buffer.cat("]\n}\n");

	// String::catf truncates silently, which would produce invalid JSON that then fails to
	// load. JsonBufferSize is derived from MaxAxesPlusExtruders so this should be
	// unreachable, but check rather than corrupt the file if a board grows.
	if (buffer.IsFull())
	{
		debugPrintf("PrinterStatistics: JSON buffer too small, not saving\n");
		file->Close();
		return;
	}

	const bool ok = file->Write(buffer.c_str());
	file->Close();
	if (!ok)
	{
		debugPrintf("PrinterStatistics: failed to write %s\n", StatsTempFileName);
		return;
	}

	if (!MassStorage::Rename(StatsTempFileName, StatsFileName, true, true))
	{
		debugPrintf("PrinterStatistics: failed to rename %s to %s\n", StatsTempFileName, StatsFileName);
	}
}

void PrinterStatistics::Load() noexcept
{
	for (size_t i = 0; i < MaxAxesPlusExtruders; ++i)
	{
		driveMicrosteps[i] = 0;
	}

	FileStore * const file = reprap.GetPlatform().OpenSysFile(StatsFileName, OpenMode::read);
	if (file == nullptr)
	{
		return;											// no statistics yet; counters stay at zero
	}

	char buffer[JsonBufferSize];
	const int bytesRead = file->Read(buffer, sizeof(buffer) - 1);
	file->Close();

	if (bytesRead <= 0)
	{
		debugPrintf("PrinterStatistics: %s is empty\n", StatsFileName);
		return;
	}
	if ((size_t)bytesRead == sizeof(buffer) - 1)
	{
		// The file filled the buffer, so it is either truncated here or longer than we can
		// parse. Parsing on would stop mid-number and silently corrupt the counters.
		debugPrintf("PrinterStatistics: %s is larger than %u bytes, ignoring it\n",
					StatsFileName, (unsigned int)sizeof(buffer));
		return;
	}
	buffer[bytesRead] = '\0';

	// Parse the unsigned value following a key token. Returns 0 when the key is absent, so
	// keys missing from an older file simply default cleanly.
	auto parseValue = [&buffer](const char *key) noexcept -> uint64_t
	{
		const char *p = strstr(buffer, key);
		if (p == nullptr)
		{
			return 0;
		}
		p += strlen(key);
		while (*p == ' ' || *p == '\t')
		{
			++p;
		}
		uint64_t value = 0;
		while (*p >= '0' && *p <= '9')
		{
			value = (value * 10) + (uint64_t)(*p - '0');
			++p;
		}
		return value;
	};

	// These two keys exist in both the v1 and v2 formats, so print time and job count
	// always carry over across an upgrade.
	lifetimePrintSeconds = (uint32_t)parseValue("\"lifetimePrintSeconds\":");
	lifetimePrintJobs = (uint32_t)parseValue("\"lifetimePrintJobs\":");

	// driveMicrosteps[] only exists in v2. When loading a v1 file the key is absent and the
	// per-motor counters stay at zero, because v1's per-axis travel cannot be converted to
	// per-motor microsteps.
	const char *arr = strstr(buffer, "\"driveMicrosteps\":");
	if (arr != nullptr && (arr = strchr(arr, '[')) != nullptr)
	{
		++arr;
		for (size_t i = 0; i < MaxAxesPlusExtruders && *arr != '\0' && *arr != ']'; )
		{
			while (*arr == ' ' || *arr == ',' || *arr == '\t' || *arr == '\n' || *arr == '\r')
			{
				++arr;
			}
			if (*arr < '0' || *arr > '9')
			{
				break;
			}
			uint64_t value = 0;
			while (*arr >= '0' && *arr <= '9')
			{
				value = (value * 10) + (uint64_t)(*arr - '0');
				++arr;
			}
			driveMicrosteps[i++] = value;
		}
	}
}

// Append a human-readable summary. Motor travel is stored in microsteps, so it is converted
// to configured units via steps-per-mm: mm for linear axes and extruders, degrees for rotary axes.
void PrinterStatistics::Report(const StringRef& reply) const noexcept
{
	// Note: this file is compiled with -fsingle-precision-constant, so an unsuffixed literal
	// such as 86400.0 is a float. Divide by an integer literal to keep the result double and
	// avoid -Werror=double-promotion.
	reply.lcatf("Print statistics:\n"
				"  Lifetime print time: %" PRIu32 " hours (%.1f days)\n"
				"  Lifetime print jobs: %" PRIu32 "\n"
				"  Per-motor travel (absolute commanded):\n",
				lifetimePrintSeconds / 3600,
				(double)lifetimePrintSeconds / 86400,
				lifetimePrintJobs);

	const GCodes& gCodes = reprap.GetGCodes();
	Move& move = reprap.GetMove();

	const size_t numAxes = gCodes.GetTotalAxes();
	const char *_ecv_array const axisLetters = gCodes.GetAxisLetters();
	for (size_t axis = 0; axis < numAxes; ++axis)
	{
		const float stepsPerUnit = move.DriveStepsPerMm(axis);
		double units = 0.0;
		if (stepsPerUnit > 0.0f)
		{
			units = (double)driveMicrosteps[axis] / (double)stepsPerUnit;
		}
		reply.catf("    %c: %.1f (%" PRIu64 " usteps)\n", axisLetters[axis], units, driveMicrosteps[axis]);
	}

	const size_t numExtruders = gCodes.GetNumExtruders();
	for (size_t e = 0; e < numExtruders; ++e)
	{
		const size_t drive = ExtruderToLogicalDrive(e);
		const float stepsPerUnit = move.DriveStepsPerMm(drive);
		double units = 0.0;
		if (stepsPerUnit > 0.0f)
		{
			units = (double)driveMicrosteps[drive] / (double)stepsPerUnit;
		}
		reply.catf("    E%u: %.1f mm (%" PRIu64 " usteps)\n", (unsigned int)e, units, driveMicrosteps[drive]);
	}
}

// End
