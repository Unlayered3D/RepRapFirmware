/*
 * PrinterStatistics.h
 *
 *  Created on: May 19, 2026
 *      Author: stedmaja
 *
 *  Lifetime usage and per-motor wear statistics, persisted to 0:/sys/printerstats.json.
 *
 *  Motor travel is tracked per LOGICAL DRIVE in absolute microsteps. Because the DDA
 *  transforms axis coordinates into motor steps (CartesianToMotorSteps), this is true
 *  per-motor travel and is therefore correct on every kinematic (Cartesian, CoreXY,
 *  delta, SCARA, 5-axis, IDEX, ...).
 */

#ifndef SRC_PLATFORM_PRINTERSTATISTICS_H_
#define SRC_PLATFORM_PRINTERSTATISTICS_H_

#include <RepRapFirmware.h>

class PrinterStatistics
{
public:
	void Init() noexcept;								// load the saved counters; call once at the end of startup
	void Spin() noexcept;								// drain the per-motor accumulators, track print time, save periodically
	void Report(const StringRef& reply) const noexcept;	// append a human-readable summary (used by M122)

	void Load() noexcept;								// read the file; leaves counters at zero if it is absent or unreadable
	void Save() noexcept;								// write the file now, via a temp file and rename

private:
	// Worst case JSON: the fixed keys plus MaxAxesPlusExtruders 20-digit values and ", "
	// separators. Sized from the array so it cannot silently overflow as boards grow, but never
	// below 1024 - an SD card moved from a 32-drive board (MB6HC) to a 12-drive one (Duet 3 Mini)
	// carries a file sized for 32 entries, and the reader must still be able to take it in.
	static constexpr size_t SizedJsonBuffer = 128 + (MaxAxesPlusExtruders * 24);
	static constexpr size_t JsonBufferSize = (SizedJsonBuffer < 1024) ? 1024 : SizedJsonBuffer;

	static constexpr const char *StatsFileName = "0:/sys/printerstats.json";
	static constexpr const char *StatsTempFileName = "0:/sys/printerstats.tmp";
	static constexpr uint32_t FileFormatVersion = 2;		// v1 stored per-AXIS travel, which is not convertible to per-motor microsteps
	static constexpr uint32_t SaveIntervalMs = 60000;	// never write the SD card more often than this

	// --- Persisted counters ---
	uint32_t lifetimePrintSeconds = 0;					// total time spent actually printing (~136 years of range)
	uint32_t lifetimePrintJobs = 0;						// print jobs STARTED, counted on the idle->printing transition
	uint64_t driveMicrosteps[MaxAxesPlusExtruders] = { 0 };	// absolute commanded motor travel per logical drive

	// --- Transient state ---
	uint32_t lastUpdateMs = 0;							// millis() at the previous Spin()
	uint32_t lastSaveMs = 0;							// millis() at the last save attempt
	uint32_t printMillisCarry = 0;						// print time below one second, carried between calls so none is lost
	bool dirty = false;									// a counter changed since the last save
	bool armedForNewJob = false;						// set while genuinely idle, so the next printing move counts as a new job
	bool initialised = false;							// Spin() does nothing until Init() has run; RepRap::Init calls Spin() before it calls stats.Init()
	bool savingSuppressed = false;						// set when Load() could not parse the file, so we never overwrite data we failed to read
};

#endif /* SRC_PLATFORM_PRINTERSTATISTICS_H_ */
