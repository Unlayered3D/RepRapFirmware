/*
 * MMU2S.h
 *
 * Driver for the Prusa Multi-Material Unit 2S/MMU3 (firmware 3.x).
 * Communicates over UART using the Prusa hex+CRC8 protocol.
 * Configured by M575 P{n} S8 B115200 in config.g.
 */

#ifndef SRC_COMMS_MMU2S_MMU2S_H_
#define SRC_COMMS_MMU2S_MMU2S_H_

#include <RepRapFirmware.h>

#if SUPPORT_MMU2S

#include "MMU2SProtocol.h"
#include <RTOSIface/RTOSIface.h>
#include <Hardware/IoPorts.h>

class AsyncSerial;
class GCodeBuffer;

class MMU2S
{
public:
	static void InitStatic() noexcept;
	static void Spin() noexcept;
	static void Diagnostics(const StringRef& reply) noexcept;

	// Called from AuxDevice::SetMode() when switching to/from mmu2s mode
	static void Enable(AsyncSerial *p_uart) noexcept;
	static void Disable() noexcept;

	// Called from GCode handlers to begin operations; return ok when started, error if already in error state
	static GCodeResult StartToolChange(uint8_t slot) noexcept;
	static GCodeResult StartLoad(uint8_t slot) noexcept;
	static GCodeResult StartUnload() noexcept;
	static GCodeResult StartEject(uint8_t slot) noexcept;
	static GCodeResult StartCut(uint8_t slot) noexcept;
	static GCodeResult StartHome(uint8_t n) noexcept;
	static GCodeResult StartReset() noexcept;
	static GCodeResult StartFindaQuery() noexcept;
	static GCodeResult StartButton(uint8_t btn) noexcept;

	// Polled each Spin() cycle by GCode state machine: returns notFinished, ok, or error
	static GCodeResult CheckOperationComplete(const StringRef& reply) noexcept;

	static bool IsEnabled() noexcept  { return uart != nullptr && operationState != OperationState::notConfigured; }
	static bool IsInError() noexcept  { return operationState == OperationState::error; }
	static bool FindaState() noexcept { return findaState; }
	static bool IsIdle() noexcept     { return operationState == OperationState::idle; }
	static bool SlotHasFilament(uint8_t slot) noexcept { return slot < MMU2SNumSlots && slotFilamentPresent[slot]; }
	static void SetVerboseUart(bool enable) noexcept { verboseUart = enable; }

	// Manually send a filament-sensor update to the MMU (M1750 F0/F1). Bypasses driver state so it
	// can be used to tell the MMU "filament reached the extruder" (f1) to complete a load it is
	// waiting on, or to recover when the driver is latched in error.
	static void SendFilamentSensorState(uint8_t v) noexcept;

	// Configure a real extruder filament-sensor pin (M1750 C"io4.in" or C"!io4.in" for active-low).
	// When configured, the driver reports the REAL sensor state to the MMU during load/unload/toolchange
	// (replacing the no-sensor timer simulation), making the load deterministic.
	static GCodeResult ConfigureFilamentSensor(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	static bool HasFilamentSensor() noexcept { return filamentSensorConfigured; }

	// Live-tunable (via M1750) so the no-sensor fixed-distance load can be dialled in without reflashing
	static void SetFsensorTripDelay(uint32_t ms) noexcept { fsensorTripDelayMs = ms; }
	static uint32_t GetFsensorTripDelay() noexcept { return fsensorTripDelayMs; }
	static void SetFsensorTriggerDelay(uint32_t ms) noexcept { fsensorTriggerDelayMs = ms; }
	static uint32_t GetFsensorTriggerDelay() noexcept { return fsensorTriggerDelayMs; }
	static void SetBowdenLengthMm(uint16_t mm) noexcept { bowdenLengthMm = mm; }
	static uint16_t GetBowdenLengthMm() noexcept { return bowdenLengthMm; }
	static void SetExtraLoadDistance(uint8_t mm) noexcept { extraLoadDistanceMm = mm; }
	static uint8_t GetExtraLoadDistance() noexcept { return extraLoadDistanceMm; }

	static ReadWriteLock mmuLock;		// protects versionString for Object Model access

private:
	enum class DriverState : uint8_t
	{
		idle,
		waitingAccepted,	// sent command, waiting for 'A' or 'R' first response
		polling,			// got 'A' from long op, now polling Q0 until 'F' or 'E'
		responseReceived,	// got final answer ('A' for immediate, 'F' for long op)
		timedOut,
		protocolError,
		mmuError,
		rejected,			// command rejected by MMU (busy with another operation)
		recovering			// operation timed out; sent B1 abort, waiting before re-handshake
	};

	enum class OperationState : uint8_t
	{
		notConfigured,
		idle,
		startupHandshake,
		toolChange,
		load,
		unload,
		eject,
		cut,
		home,
		findaQuery,
		directCommand,
		resetting,			// sent X0, waiting for A, then re-runs startup handshake
		error
	};

	static GCodeResult BeginCommand(char cmd, uint8_t param, OperationState nextState, bool longOp) noexcept;
	static void SendCommand(char cmd, uint8_t param) noexcept;
	static void SendWriteCommand(uint8_t addr, uint16_t value) noexcept;	// W{addr} {value} register write
	static void ParseResponse() noexcept;
	static void DriveStartupHandshake() noexcept;
	static const char *OperationName() noexcept;

	static AsyncSerial    *uart;
	static DriverState     driverState;
	static OperationState  operationState;
	static uint8_t  handshakeStep;		// 0=S0, 1=S1, 2=S2 during startup
	static char     lastParamCode;		// most recent response param code ('A', 'F', 'P', 'E', 'R')
	static char     lastEchoCode;		// echoed request code of the last response (e.g. 'T' during a toolchange, 'f' for an FSensor ack)
	static uint8_t  lastProgressCode;	// value carried by the last 'P' (Processing) response — MMU ProgressCode
	static int32_t  lastNumericResult;	// paramValue from last 'A' or 'F' response
	static int32_t  lastErrorCode;		// paramValue from last 'E' response
	static uint32_t whenCommandSent;	// millis() when last command/poll was sent
	static uint32_t operationStartMs;	// millis() when the current operation began (for overall polling timeout)
	static uint32_t lastPollMs;			// millis() when last Q0 poll was sent
	static bool     isLongOperation;	// true for ops that need Q0 polling after Accepted
	static bool     isRecovering;		// true while auto-recovering from a timeout
	static bool     pendingError;		// set when auto-recovery completes; consumed once by CheckOperationComplete
	static bool     needsInitialQ0;	// true only on first Enable(); false on M1750 R re-handshake
	static bool     findaState;
	static bool     slotFilamentPresent[MMU2SNumSlots];	// filament detected in each slot (updated by load/toolchange)
	static bool     verboseUart;		// when true, log every TX and RX line (toggled via M1750 V1/V0)
	static uint8_t  currentSlot;		// slot number for the in-progress load/toolchange
	static uint8_t  q0PollCount;		// number of Q0 polls sent during startup handshake step 3
	static uint16_t bowdenLengthMm;		// MMU Bowden length written to register 0x22 at init (live-tunable)
	static uint8_t  extraLoadDistanceMm;	// MMU extra load distance written to register 0x0b at init (live-tunable)
	static IoPort   filamentSensorPort;	// optional real extruder filament sensor (M1750 C"...")
	static bool     filamentSensorConfigured;	// true once a real sensor pin is assigned
	static bool     lastReportedSensorPresent;	// last filament-present state we reported to the MMU (f0/f1)
	static uint32_t fsensorTripMs;			// millis() when sensor first tripped (present→true); 0 = not pending
	static uint32_t fsensorTripDelayMs;		// how long to wait after sensor trips before reporting f1 (default 1000ms)
	static uint32_t fsensorTriggerDelayMs;	// delay after entering FeedingToBondtech before we fake f1 (live-tunable)
	static uint32_t fsensorPhaseStartMs;	// millis() when the current load entered FeedingToBondtech
	static bool     fsensorSimReported;	// true once we've sent the simulated f1 for the current operation
	static bool     needSensorBaseline;	// send the current real sensor state to the MMU on the first poll of a filament op
	static char     rxBuf[MMU2SMaxResponseLen];
	static uint8_t  rxLen;
	static char     versionString[24];	// MMU firmware version (built from S0/S1/S2)
};

#endif	// SUPPORT_MMU2S

#endif /* SRC_COMMS_MMU2S_MMU2S_H_ */
