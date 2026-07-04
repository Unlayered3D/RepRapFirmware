/*
 * MMU2SProtocol.h
 *
 * Prusa MMU2S/MMU3 UART protocol constants — firmware 3.x.
 * Request:  [CMD][HEX_VALUE]*[CRC8_HEX]\n
 * Response: [CMD][HEX_VALUE] [PARAMCODE][HEX_VALUE]*[CRC8_HEX]\n
 * CRC8-CCITT (poly 0x07, init 0x00) over code + value + value2(uint16 LE).
 * 115200 baud 8N1.
 */

#ifndef SRC_COMMS_MMU2S_MMU2SPROTOCOL_H_
#define SRC_COMMS_MMU2S_MMU2SPROTOCOL_H_

#include <cstdint>

namespace MMU2SCmd
{
	constexpr char ToolChange  = 'T';	// T{slot} — tool change to slot 0-4
	constexpr char Load        = 'L';	// L{slot} — load filament from slot
	constexpr char Unload      = 'U';	// U0 — unload current filament
	constexpr char Eject       = 'E';	// E{slot} — eject filament from slot
	constexpr char Cut         = 'K';	// K{slot} — cut filament at slot
	constexpr char FindaQuery  = 'P';	// P0 — query FINDA sensor (0=no filament, 1=present)
	constexpr char VersionQ    = 'S';	// S0=major, S1=minor, S2=revision
	constexpr char Button      = 'B';	// B{n} — simulate button press (0=left, 1=middle, 2=right)
	constexpr char Home        = 'H';	// H{n} — home MMU axes
	constexpr char Reset       = 'X';	// X0 — reset MMU ('R' means read-register in fw 3.x)
	constexpr char QueryStatus = 'Q';	// Q0 — poll running operation status
	constexpr char ReadReg     = 'R';	// R{addr} — read register
	constexpr char WriteReg    = 'W';	// W{addr} {val} — write register
	constexpr char FilamentSensor = 'f';	// f{n} — report printer filament-sensor state to MMU (lowercase)
}

// MMU register addresses (fw 3.x) — written during the init handshake to make the MMU operational.
// Without these writes + the FilamentSensor update + a final Q0, the MMU stays in "Start" scope
// and REJECTS all motion commands (T/L/U/H) with 'R'.
namespace MMU2SReg
{
	constexpr uint8_t  ExtraLoadDistance  = 0x0b;	// uint8, mm (range 0-30)
	constexpr uint8_t  PulleySlowFeedrate = 0x14;	// uint16, mm/s
	constexpr uint8_t  BowdenLength       = 0x22;	// uint16, mm (range 341-1000) — distance MMU feeds FINDA->extruder
}

constexpr uint8_t  MMU2SInitExtraLoadDistance  = 15;	// mm pushed past the (simulated) sensor toward the extruder; 5mm left filament a few mm short of the gears
constexpr uint16_t MMU2SInitPulleySlowFeedrate = 20;	// mm/s (register default)
constexpr uint16_t MMU2SDefaultBowdenLengthMm  = 560;	// mm — this printer's MMU->extruder Bowden (~550) + margin

// This printer has NO filament sensor at the extruder. During the MMU's feed-to-extruder phase
// (ProgressCode FeedingToBondtech) the MMU3 waits for the printer to report the FSensor triggered.
// We simulate it: after the MMU has been feeding for this delay (≈ time to traverse the Bowden),
// report f1 so the MMU completes the load. Tunable live via M1750 to avoid reflashing.
constexpr uint32_t MMU2SDefaultFsensorTriggerDelayMs = 6000;	// ms feeding before we fake the sensor trigger

// MMU ProgressCode values (subset we act on) — value carried by a 'P' (Processing) response.
namespace MMU2SProgress
{
	constexpr uint8_t FeedingToFinda    = 5;	// feeding from parking to FINDA (inside MMU)
	constexpr uint8_t FeedingToBondtech = 6;	// feeding through Bowden toward extruder gears
	constexpr uint8_t FeedingToNozzle   = 7;	// final feed into the nozzle
	constexpr uint8_t FeedingToFSensor  = 28;	// feeding toward the (printer) filament sensor, awaiting trigger
}

namespace MMU2SResp
{
	constexpr char Accepted   = 'A';	// command accepted (paramValue = register value for S/R)
	constexpr char Rejected   = 'R';	// command rejected
	constexpr char Processing = 'P';	// long op in progress (paramValue = step number)
	constexpr char Finished   = 'F';	// long op finished
	constexpr char Error      = 'E';	// error (paramValue = error code)
}

constexpr uint32_t MMU2SBaudRate           = 115200;
constexpr uint32_t MMU2SResponseTimeoutMs  = 3000;	 // max wait for initial Accepted response
constexpr uint32_t MMU2SOperationTimeoutMs = 45000;  // overall timeout once polling started; a full load/unload through the 550mm Bowden (FeedingToFinda + FeedingToBondtech) measured >15s
constexpr uint32_t MMU2SStartupTimeoutMs   = 30000;	 // max wait during startup handshake (MMU homing can take ~30s with loaded filament)
constexpr uint32_t MMU2SPollIntervalMs     = 100;	 // Q0 poll interval during long operations
constexpr uint8_t  MMU2SNumSlots         = 5;		// slots 0-4
constexpr size_t   MMU2SMaxResponseLen   = 24;		// max response line length

#endif /* SRC_COMMS_MMU2S_MMU2SPROTOCOL_H_ */
