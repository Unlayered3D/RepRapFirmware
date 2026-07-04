/*
 * MMU2S.cpp
 *
 * Driver for the Prusa Multi-Material Unit 2S/MMU3 — firmware 3.x protocol.
 * Protocol: hex-encoded values with CRC8-CCITT checksum.
 * Request:  [CMD][HEX]*[CRC8]\n
 * Response: [CMD][HEX] [PARAMCODE][HEX]*[CRC8]\n
 */

#include "MMU2S.h"

#if SUPPORT_MMU2S

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <AsyncSerial.h>

// Static member definitions
ReadWriteLock          MMU2S::mmuLock;
AsyncSerial           *MMU2S::uart             = nullptr;
MMU2S::DriverState     MMU2S::driverState       = MMU2S::DriverState::idle;
MMU2S::OperationState  MMU2S::operationState    = MMU2S::OperationState::notConfigured;
uint8_t                MMU2S::handshakeStep      = 0;
char                   MMU2S::lastParamCode      = 0;
char                   MMU2S::lastEchoCode       = 0;
uint8_t                MMU2S::lastProgressCode   = 0;
int32_t                MMU2S::lastNumericResult  = -1;
int32_t                MMU2S::lastErrorCode      = 0;
uint32_t               MMU2S::whenCommandSent       = 0;
uint32_t               MMU2S::operationStartMs      = 0;
uint32_t               MMU2S::lastPollMs            = 0;
bool                   MMU2S::isLongOperation       = false;
bool                   MMU2S::isRecovering          = false;
bool                   MMU2S::pendingError          = false;
bool                   MMU2S::needsInitialQ0        = false;
bool                   MMU2S::findaState            = false;
bool                   MMU2S::slotFilamentPresent[MMU2SNumSlots] = {};
bool                   MMU2S::verboseUart           = false;
uint8_t                MMU2S::currentSlot           = MMU2SNumSlots;
uint8_t                MMU2S::q0PollCount           = 0;
uint16_t               MMU2S::bowdenLengthMm        = MMU2SDefaultBowdenLengthMm;
uint8_t                MMU2S::extraLoadDistanceMm   = MMU2SInitExtraLoadDistance;
IoPort                 MMU2S::filamentSensorPort;
bool                   MMU2S::filamentSensorConfigured = false;
bool                   MMU2S::lastReportedSensorPresent = false;
uint32_t               MMU2S::fsensorTripMs          = 0;
uint32_t               MMU2S::fsensorTripDelayMs     = 1000;
uint32_t               MMU2S::fsensorTriggerDelayMs  = MMU2SDefaultFsensorTriggerDelayMs;
uint32_t               MMU2S::fsensorPhaseStartMs   = 0;
bool                   MMU2S::fsensorSimReported    = false;
bool                   MMU2S::needSensorBaseline    = false;
char                   MMU2S::rxBuf[MMU2SMaxResponseLen];
uint8_t                MMU2S::rxLen                 = 0;
char                   MMU2S::versionString[24]     = {};

// CRC8-CCITT (poly 0x07, init 0x00) — matches Prusa MMU firmware implementation
static uint8_t CRC8_update(uint8_t crc, uint8_t b) noexcept
{
	uint8_t data = crc ^ b;
	for (uint8_t i = 0; i < 8; ++i)
	{
		data = (data & 0x80u) ? (uint8_t)((data << 1u) ^ 0x07u) : (uint8_t)(data << 1u);
	}
	return data;
}

// Compute CRC8 over a simple request (code + value byte + two zero bytes for value2)
static uint8_t RequestCRC(char cmd, uint8_t value) noexcept
{
	uint8_t crc = CRC8_update(0, (uint8_t)cmd);
	crc = CRC8_update(crc, value);
	crc = CRC8_update(crc, 0);	// value2 low byte (always 0 for our requests)
	crc = CRC8_update(crc, 0);	// value2 high byte
	return crc;
}

static bool IsHexChar(char c) noexcept
{
	return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

static uint8_t HexNibble(char c) noexcept
{
	if (c >= '0' && c <= '9') { return (uint8_t)(c - '0'); }
	if (c >= 'a' && c <= 'f') { return (uint8_t)(c - 'a' + 10); }
	return (uint8_t)(c - 'A' + 10);
}

void MMU2S::InitStatic() noexcept
{
	// Nothing to do — enabled lazily via M575
}

void MMU2S::Enable(AsyncSerial *p_uart) noexcept
{
	uart = p_uart;
	driverState = DriverState::idle;
	rxLen = 0;
	handshakeStep = 0;
	lastParamCode = 0;
	isLongOperation = false;
	isRecovering = false;
	pendingError = false;
	needsInitialQ0 = false;		// MMU3 3.x firmware does not respond to Q0 during startup; go idle after S0/S1/S2
	currentSlot = MMU2SNumSlots;
	q0PollCount = 0;
	lastReportedSensorPresent = false;		// the init handshake sends f0, so the MMU starts believing "no filament"
	fsensorTripMs = 0;
	memset(slotFilamentPresent, 0, sizeof(slotFilamentPresent));
	versionString[0] = '\0';
	operationState = OperationState::startupHandshake;
	whenCommandSent = millis();
}

void MMU2S::Disable() noexcept
{
	uart = nullptr;
	operationState = OperationState::notConfigured;
	driverState = DriverState::idle;
	rxLen = 0;
}

// Called from RepRap::Spin() — must not block
void MMU2S::Spin() noexcept
{
	if (uart == nullptr) { return; }

	// Drain RX buffer, looking for '\n'-terminated response lines.
	// During auto-recovery we discard all incoming bytes to avoid stale MMU responses
	// confusing the handshake that follows.
	while (uart->available() > 0 && rxLen < sizeof(rxBuf) - 1)
	{
		const char c = (char)uart->read();
		if (c == '\n' || c == '\r')
		{
			if (rxLen > 0 && driverState != DriverState::recovering)
			{
				rxBuf[rxLen] = '\0';
				ParseResponse();
			}
			rxLen = 0;
			break;
		}
		rxBuf[rxLen++] = c;
	}

	if (rxLen >= sizeof(rxBuf) - 1)
	{
		rxBuf[rxLen] = '\0';
		reprap.GetPlatform().MessageF(WarningMessage, "MMU2S RX overflow: '%s'\n", rxBuf);
		rxLen = 0;
	}

	// Auto-recovery: wait 2.5s after sending B1 abort, then restart handshake
	if (driverState == DriverState::recovering)
	{
		if (millis() - whenCommandSent < 2500) { return; }
		rxLen = 0;
		handshakeStep = 0;
		q0PollCount = 0;
		driverState = DriverState::idle;
		operationState = OperationState::startupHandshake;
		// isRecovering stays true — cleared when handshake completes
		return;
	}

	// After a long-operation command gets Accepted, switch from responseReceived to polling
	if (driverState == DriverState::responseReceived && lastParamCode == MMU2SResp::Accepted && isLongOperation)
	{
		driverState = DriverState::polling;
		lastPollMs = millis() - MMU2SPollIntervalMs;	// trigger first Q0 immediately
	}

	// Drive Q0 polling for long operations
	if (driverState == DriverState::polling)
	{
		const uint32_t now = millis();

		// Auto-report the extruder filament-sensor state to the MMU during LOAD/toolchange only.
		// Unload is deliberately excluded: on this machine the sensor sits on the Bowden side of the
		// extruder gears, so the extruder can never pull the filament past it — only the MMU's
		// pull-back can. The tfree macro therefore sends M1750 F0 (sensor clear) before M1750 U so the
		// MMU proceeds to pull the filament all the way back; FINDA (E8002) is the real unload check.
		// If we auto-reported the live pin here, it would re-send f1 (present) and trip E8004.
		const bool inFilamentOp = (operationState == OperationState::load
			|| operationState == OperationState::toolChange);

		if (filamentSensorConfigured)
		{
			// REAL sensor: report state to the MMU during filament operations.
			// f1 (present) is delayed by fsensorTripDelayMs to let filament travel past the sensor
			// before the MMU declares the load done. f0 (clear) is immediate.
			if (inFilamentOp)
			{
				const bool present = filamentSensorPort.ReadDigital();
				if (needSensorBaseline)
				{
					// First poll of this op: report the real sensor state so the MMU and driver agree,
					// even if no transition occurs during the op (e.g. unload after the filament was
					// already retracted past the sensor by tfree).
					needSensorBaseline = false;
					lastReportedSensorPresent = present;
					fsensorTripMs = 0;
					SendCommand(MMU2SCmd::FilamentSensor, present ? 1u : 0u);
					reprap.GetPlatform().MessageF(LoggedGenericMessage,
						"MMU2S: reporting initial filament sensor state f%d\n", present ? 1 : 0);
				}
				else if (present && !lastReportedSensorPresent)
				{
					// Sensor just tripped — start the delay timer if not already running
					if (fsensorTripMs == 0)
					{
						fsensorTripMs = now;
						reprap.GetPlatform().MessageF(LoggedGenericMessage,
							"MMU2S: filament sensor tripped, waiting %" PRIu32 " ms before f1\n", fsensorTripDelayMs);
					}
					else if (now - fsensorTripMs >= fsensorTripDelayMs)
					{
						SendCommand(MMU2SCmd::FilamentSensor, 1u);
						lastReportedSensorPresent = true;
						fsensorTripMs = 0;
						reprap.GetPlatform().Message(LoggedGenericMessage, "MMU2S: filament sensor -> present (f1)\n");
					}
				}
				else if (!present && lastReportedSensorPresent)
				{
					// Filament cleared — report immediately and cancel any pending trip
					fsensorTripMs = 0;
					SendCommand(MMU2SCmd::FilamentSensor, 0u);
					lastReportedSensorPresent = false;
					reprap.GetPlatform().Message(LoggedGenericMessage, "MMU2S: filament sensor -> clear (f0)\n");
				}
				else if (!present && fsensorTripMs != 0)
				{
					// Sensor de-tripped before delay expired (bounce) — cancel
					fsensorTripMs = 0;
					reprap.GetPlatform().Message(LoggedGenericMessage, "MMU2S: filament sensor trip cancelled (bounce)\n");
				}
			}
		}
		else if ((operationState == OperationState::load || operationState == OperationState::toolChange)
			&& !fsensorSimReported)
		{
			// NO-SENSOR fallback: fake the sensor trigger on a timer once the MMU is feeding toward the
			// extruder (FeedingToBondtech or FeedingToFSensor). Latch the timer at the first such phase
			// and fire regardless of the current sub-code. Fragile (timing-dependent) — prefer a real sensor.
			const bool feedingToExtruder = (lastProgressCode == MMU2SProgress::FeedingToBondtech
				|| lastProgressCode == MMU2SProgress::FeedingToFSensor);
			if (fsensorPhaseStartMs == 0 && feedingToExtruder)
			{
				fsensorPhaseStartMs = now;
			}
			if (fsensorPhaseStartMs != 0 && now - fsensorPhaseStartMs >= fsensorTriggerDelayMs)
			{
				SendCommand(MMU2SCmd::FilamentSensor, 1);
				fsensorSimReported = true;
				reprap.GetPlatform().Message(LoggedGenericMessage, "MMU2S: simulated filament-sensor trigger (f1)\n");
			}
		}

		if (now - lastPollMs >= MMU2SPollIntervalMs)
		{
			SendCommand(MMU2SCmd::QueryStatus, 0);
			lastPollMs = now;
			whenCommandSent = now;
		}
	}

	// Timeout check for in-flight commands.
	// During polling we use operationStartMs (set when the operation began) so that the
	// 100ms Q0 resend doesn't continuously reset the timeout window.
	if (driverState == DriverState::waitingAccepted || driverState == DriverState::polling)
	{
		const bool isStartup = (operationState == OperationState::startupHandshake);
		const uint32_t reference = (driverState == DriverState::polling) ? operationStartMs : whenCommandSent;
		const uint32_t timeout   = isStartup                             ? MMU2SStartupTimeoutMs  :
		                           (driverState == DriverState::polling) ? MMU2SOperationTimeoutMs :
		                                                                   MMU2SResponseTimeoutMs;
		if (millis() - reference > timeout)
		{
			if (isStartup)
			{
				// Startup handshake timeout — just report and latch error
				reprap.GetPlatform().Message(ErrorMessage, "MMU2S: startup handshake timed out\n");
				driverState = DriverState::timedOut;
				operationState = OperationState::error;
			}
			else
			{
				// Running operation timeout — mark slot and auto-recover
				if ((operationState == OperationState::load || operationState == OperationState::toolChange)
					&& currentSlot < MMU2SNumSlots)
				{
					slotFilamentPresent[currentSlot] = false;
				}
				reprap.GetPlatform().Message(WarningMessage, "MMU2S: operation timed out, auto-recovering...\n");
				SendCommand(MMU2SCmd::Button, 1);	// middle button = abort/OK on MMU error screen
				driverState = DriverState::recovering;
				whenCommandSent = millis();
				isRecovering = true;
			}
		}
	}

	// Drive the startup handshake state machine
	if (operationState == OperationState::startupHandshake)
	{
		DriveStartupHandshake();
	}
}

// Drive the startup handshake. The Prusa MMU3 starts in a "Start" scope that REJECTS all
// motion commands (T/L/U/H) until the printer completes the init sequence:
//   step 0-2: S0/S1/S2 version queries
//   step 3:   W 0x0b  (Extra Load Distance)   register write
//   step 4:   W 0x14  (Pulley Slow Feedrate)  register write
//   step 5:   W 0x22  (Bowden Length)         register write
//   step 6:   f 0      FilamentSensor update (report printer fsensor state)
//   step 7:   Q 0      query — flips the MMU from "Start" scope into "Idle" scope (READY)
// Only after step 7 does the MMU accept tool/load/unload commands.
void MMU2S::DriveStartupHandshake() noexcept
{
	// Step 7 (Q0): MMU may reply 'P' (still finishing internal startup); keep polling until 'F'/'A'.
	if (handshakeStep == 7 && driverState == DriverState::waitingAccepted && lastParamCode == MMU2SResp::Processing)
	{
		if (millis() - whenCommandSent >= MMU2SPollIntervalMs)
		{
			++q0PollCount;
			if (q0PollCount >= 150)		// 150 × 100ms = 15s max for MMU to finish internal startup
			{
				reprap.GetPlatform().Message(ErrorMessage, "MMU2S: Q0 startup poll timed out\n");
				operationState = OperationState::error;
				driverState = DriverState::timedOut;
				return;
			}
			driverState = DriverState::idle;	// triggers Q0 resend below
		}
		return;
	}

	if (driverState == DriverState::waitingAccepted || driverState == DriverState::polling)
	{
		return;		// waiting for a response
	}

	if (driverState == DriverState::responseReceived)
	{
		switch (handshakeStep)
		{
		case 0:		// response to S0 (major version)
			SafeSnprintf(versionString, sizeof(versionString), "%" PRIi32, lastNumericResult);
			break;
		case 1:		// response to S1 (minor version)
			{
				const size_t pos = strlen(versionString);
				SafeSnprintf(versionString + pos, sizeof(versionString) - pos, ".%" PRIi32, lastNumericResult);
			}
			break;
		case 2:		// response to S2 (revision) — version string complete
			{
				const size_t pos = strlen(versionString);
				SafeSnprintf(versionString + pos, sizeof(versionString) - pos, ".%" PRIi32, lastNumericResult);
			}
			reprap.GetPlatform().MessageF(LoggedGenericMessage, "MMU2S firmware version %s\n", versionString);
			q0PollCount = 0;
			break;
		case 3:		// W 0x0b (extra load distance) acknowledged
		case 4:		// W 0x14 (pulley slow feedrate) acknowledged
		case 5:		// W 0x22 (bowden length) acknowledged
		case 6:		// f 0 acknowledged
			break;
		case 7:		// Q0 acknowledged — MMU is now in Idle scope and READY
			if (isRecovering) { pendingError = true; isRecovering = false; }
			operationState = OperationState::idle;
			driverState = DriverState::idle;
			handshakeStep = 0;
			reprap.GetPlatform().Message(LoggedGenericMessage, "MMU2S ready (idle)\n");
			return;
		default:
			break;
		}
		++handshakeStep;
		driverState = DriverState::idle;
	}

	if (driverState == DriverState::timedOut || driverState == DriverState::mmuError
		|| driverState == DriverState::protocolError || driverState == DriverState::rejected)
	{
		if (handshakeStep == 7 && driverState == DriverState::mmuError)
		{
			// MMU reached operational scope but is reporting a standing error (e.g. E8087 =
			// selector homing failed). Re-handshake won't clear it — the MMU must RETRY the
			// failed operation. Tell the user to clear any jam and press the button to retry.
			reprap.GetPlatform().MessageF(ErrorMessage,
				"MMU2S: MMU reports error E%04" PRIx32 " — clear any jam, then 'M1750 B1' to retry\n", lastErrorCode);
		}
		else
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "MMU2S: init handshake failed at step %u (state %u)\n",
				(unsigned)handshakeStep, (unsigned)driverState);
		}
		operationState = OperationState::error;
		return;
	}

	if (driverState == DriverState::idle)
	{
		isLongOperation = false;
		switch (handshakeStep)
		{
		case 0:
		case 1:
		case 2:
			SendCommand(MMU2SCmd::VersionQ, handshakeStep);
			break;
		case 3:
			SendWriteCommand(MMU2SReg::ExtraLoadDistance, extraLoadDistanceMm);
			break;
		case 4:
			SendWriteCommand(MMU2SReg::PulleySlowFeedrate, MMU2SInitPulleySlowFeedrate);
			break;
		case 5:
			SendWriteCommand(MMU2SReg::BowdenLength, bowdenLengthMm);	// feed distance FINDA->extruder
			break;
		case 6:
			SendCommand(MMU2SCmd::FilamentSensor, 0);	// report: no filament at printer sensor
			break;
		default:	// 7
			SendCommand(MMU2SCmd::QueryStatus, 0);		// flips MMU into Idle scope
			break;
		}
		driverState = DriverState::waitingAccepted;
		whenCommandSent = millis();
	}
}

// Parse a complete response line in rxBuf (without the terminating '\n').
// Response format: [CODE][HEXVAL] [PARAMCODE][HEXVAL]*[CRC]
void MMU2S::ParseResponse() noexcept
{
	if (rxLen == 0) { return; }

	if (verboseUart)
	{
		reprap.GetPlatform().MessageF(LoggedGenericMessage, "MMU2S RX: %s\n", rxBuf);
	}

	uint8_t i = 0;

	// Capture the echoed command code byte. During a long operation the MMU echoes the ACTIVE
	// command code (e.g. 'T' during a tool change), NOT the Q0 we polled with. An async FSensor
	// update we send mid-operation is acked with echo code 'f'.
	if (i >= rxLen) { driverState = DriverState::protocolError; return; }
	lastEchoCode = rxBuf[i];
	++i;

	// Skip echoed request hex value (until space)
	while (i < rxLen && rxBuf[i] != ' ') { ++i; }
	if (i >= rxLen) { driverState = DriverState::protocolError; return; }
	++i;	// skip space

	// Read param code
	if (i >= rxLen) { driverState = DriverState::protocolError; return; }
	lastParamCode = rxBuf[i++];

	// Read param hex value (until '*' or end of buffer)
	uint16_t paramVal = 0;
	while (i < rxLen && rxBuf[i] != '*')
	{
		if (!IsHexChar(rxBuf[i])) { driverState = DriverState::protocolError; return; }
		paramVal = (uint16_t)((paramVal << 4u) | HexNibble(rxBuf[i]));
		++i;
	}
	// CRC validation omitted — add once communication is confirmed working

	// An async FSensor ('f') ack arriving while we are polling a long operation is just an
	// acknowledgement of our simulated sensor update — it must NOT be treated as the operation
	// completing. Leave driverState as polling and ignore it.
	if (driverState == DriverState::polling && lastEchoCode == MMU2SCmd::FilamentSensor)
	{
		return;
	}

	switch (lastParamCode)
	{
	case MMU2SResp::Accepted:
		lastNumericResult = (int32_t)paramVal;
		driverState = DriverState::responseReceived;
		break;

	case MMU2SResp::Finished:
		lastNumericResult = (int32_t)paramVal;
		driverState = DriverState::responseReceived;
		break;

	case MMU2SResp::Processing:
		// Still running — leave driverState unchanged (stays polling). Capture the progress code
		// so the FSensor simulation can detect the FeedingToBondtech phase.
		lastProgressCode = (uint8_t)paramVal;
		break;

	case MMU2SResp::Error:
		lastErrorCode = (int32_t)paramVal;
		driverState = DriverState::mmuError;
		if (operationState != OperationState::startupHandshake)
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "MMU2S error E%04" PRIx32 "\n", lastErrorCode);
		}
		break;

	case MMU2SResp::Rejected:
		driverState = DriverState::rejected;
		break;

	default:
		driverState = DriverState::protocolError;
		break;
	}
}

// Encode and transmit a request.
// Format: [CMD][HEX_VALUE]*[CRC8_HEX]\n
// Uses minimal hex (no leading zeros except "0" for zero), matching MMU firmware's UInt8ToHex.
void MMU2S::SendCommand(char cmd, uint8_t param) noexcept
{
	const uint8_t crc = RequestCRC(cmd, param);
	char buf[12];
	const int len = SafeSnprintf(buf, sizeof(buf), "%c%x*%x\n", cmd, (unsigned)param, (unsigned)crc);
	for (int i = 0; i < len; ++i) { uart->write((uint8_t)buf[i]); }
	if (verboseUart)
	{
		reprap.GetPlatform().MessageF(LoggedGenericMessage, "MMU2S TX: %.*s\n", len - 1, buf);	// -1 excludes trailing '\n'
	}
}

// Encode and transmit a register-write request.
// Format: W{addr_hex} {value_hex}*{CRC8_hex}\n  e.g. "Wb 5*xx\n" writes 5 to register 0x0b.
// CRC8 is computed over (cmd, addr, value_low, value_high) — same scheme as RequestCRC but with a real value2.
void MMU2S::SendWriteCommand(uint8_t addr, uint16_t value) noexcept
{
	uint8_t crc = CRC8_update(0, (uint8_t)MMU2SCmd::WriteReg);
	crc = CRC8_update(crc, addr);
	crc = CRC8_update(crc, (uint8_t)(value & 0xFFu));
	crc = CRC8_update(crc, (uint8_t)(value >> 8u));
	char buf[16];
	const int len = SafeSnprintf(buf, sizeof(buf), "%c%x %x*%x\n",
		MMU2SCmd::WriteReg, (unsigned)addr, (unsigned)value, (unsigned)crc);
	for (int i = 0; i < len; ++i) { uart->write((uint8_t)buf[i]); }
	if (verboseUart)
	{
		reprap.GetPlatform().MessageF(LoggedGenericMessage, "MMU2S TX: %.*s\n", len - 1, buf);
	}
}

// Start a command if the driver is idle
GCodeResult MMU2S::BeginCommand(char cmd, uint8_t param, OperationState nextState, bool longOp) noexcept
{
	if (uart == nullptr || operationState == OperationState::notConfigured)
	{
		return GCodeResult::error;
	}
	if (operationState == OperationState::error)
	{
		return GCodeResult::error;
	}
	if (operationState != OperationState::idle)
	{
		return GCodeResult::error;		// busy
	}
	isLongOperation = longOp;
	// Reset the per-operation filament-sensor state.
	// Snapshot the real sensor so we only react to transitions during this operation,
	// not to filament that was already parked at the sensor before the op started.
	fsensorTripMs = 0;
	fsensorPhaseStartMs = 0;
	fsensorSimReported = false;
	lastReportedSensorPresent = filamentSensorConfigured && filamentSensorPort.ReadDigital();
	// Re-sync the MMU with the real sensor on the first poll of a LOAD/toolchange. The transition
	// logic in Spin() only runs while polling, so a clear->present edge would otherwise be missed.
	// UNLOAD is excluded on purpose (see inFilamentOp note in Spin): the macro hand-sets f0 via
	// M1750 F0 before M1750 U, and we must not override it with the live (still-present) pin.
	needSensorBaseline = filamentSensorConfigured
		&& (nextState == OperationState::load
		 || nextState == OperationState::toolChange);
	lastProgressCode = 0;
	SendCommand(cmd, param);
	driverState = DriverState::waitingAccepted;
	operationState = nextState;
	whenCommandSent = millis();
	operationStartMs = whenCommandSent;
	rxLen = 0;
	return GCodeResult::ok;
}

GCodeResult MMU2S::StartToolChange(uint8_t slot) noexcept
{
	currentSlot = slot;
	return BeginCommand(MMU2SCmd::ToolChange, slot, OperationState::toolChange, true);
}

GCodeResult MMU2S::StartLoad(uint8_t slot) noexcept
{
	currentSlot = slot;
	return BeginCommand(MMU2SCmd::Load, slot, OperationState::load, true);
}

GCodeResult MMU2S::StartUnload() noexcept
{
	return BeginCommand(MMU2SCmd::Unload, 0, OperationState::unload, true);
}

GCodeResult MMU2S::StartEject(uint8_t slot) noexcept
{
	return BeginCommand(MMU2SCmd::Eject, slot, OperationState::eject, true);
}

GCodeResult MMU2S::StartCut(uint8_t slot) noexcept
{
	return BeginCommand(MMU2SCmd::Cut, slot, OperationState::cut, true);
}

GCodeResult MMU2S::StartHome(uint8_t n) noexcept
{
	return BeginCommand(MMU2SCmd::Home, n, OperationState::home, true);
}

GCodeResult MMU2S::StartReset() noexcept
{
	if (uart == nullptr) { return GCodeResult::error; }
	// Clear all state and re-run the startup handshake.
	// We do not send 'X' to the MMU because it reboots immediately without ACKing,
	// causing a guaranteed timeout. Re-handshaking achieves the same re-sync effect.
	driverState = DriverState::idle;
	handshakeStep = 0;
	lastParamCode = 0;
	rxLen = 0;
	versionString[0] = '\0';
	needsInitialQ0 = false;		// MMU already running — skip Q0 boot poll
	operationState = OperationState::startupHandshake;
	return GCodeResult::ok;
}

// Send a raw filament-sensor update to the MMU, regardless of driver state.
// Used by M1750 F1/F0 to manually tell the MMU the (absent) extruder sensor's state — e.g. to
// confirm a load the MMU is waiting on, since this printer has no real sensor.
void MMU2S::SendFilamentSensorState(uint8_t v) noexcept
{
	if (uart == nullptr) { return; }
	SendCommand(MMU2SCmd::FilamentSensor, (v != 0) ? 1u : 0u);
}

// Assign the real extruder filament-sensor pin from M1750 C"...". Use C"!io4.in" for an active-low
// sensor so ReadDigital() returns true when filament is present. Once configured, Spin() reports the
// real sensor to the MMU on every change during load/unload/toolchange (instead of the timer sim).
GCodeResult MMU2S::ConfigureFilamentSensor(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (!filamentSensorPort.AssignPort(gb, reply, PinUsedBy::filamentMonitor, PinAccess::read))
	{
		filamentSensorConfigured = false;
		return GCodeResult::error;
	}
	filamentSensorConfigured = true;
	lastReportedSensorPresent = false;		// assume absent (matches the f0 sent during the init handshake)
	reply.copy("MMU2S filament sensor on ");
	filamentSensorPort.AppendPinName(reply);
	reply.catf(" — currently %s", filamentSensorPort.ReadDigital() ? "filament present" : "no filament");
	return GCodeResult::ok;
}

GCodeResult MMU2S::StartFindaQuery() noexcept
{
	return BeginCommand(MMU2SCmd::FindaQuery, 0, OperationState::findaQuery, false);
}

GCodeResult MMU2S::StartButton(uint8_t btn) noexcept
{
	if (uart == nullptr) { return GCodeResult::error; }
	// Allow button presses from error state so the user can dismiss the MMU error screen
	operationState = OperationState::idle;
	driverState = DriverState::idle;
	return BeginCommand(MMU2SCmd::Button, btn, OperationState::directCommand, false);
}

// Poll whether the current operation has completed.
// Returns notFinished while still waiting, ok on success, error on failure.
GCodeResult MMU2S::CheckOperationComplete(const StringRef& reply) noexcept
{
	switch (driverState)
	{
	case DriverState::waitingAccepted:
	case DriverState::polling:
	case DriverState::recovering:	// auto-recovery in progress
		return GCodeResult::notFinished;

	case DriverState::responseReceived:
		{
			const OperationState completedOp = operationState;
			operationState = OperationState::idle;
			driverState = DriverState::idle;

			// After a successful reset, re-run the startup handshake
			if (completedOp == OperationState::resetting)
			{
				handshakeStep = 0;
				operationState = OperationState::startupHandshake;
				return GCodeResult::ok;
			}

			// Update FINDA state if this was a FINDA query
			if (completedOp == OperationState::findaQuery && lastNumericResult >= 0)
			{
				WriteLocker lock(mmuLock);
				findaState = (lastNumericResult != 0);
				reply.catf("FINDA: %s", findaState ? "filament present" : "no filament");
			}

			// Track which slots have filament after a successful load or tool change
			if ((completedOp == OperationState::load || completedOp == OperationState::toolChange)
				&& currentSlot < MMU2SNumSlots)
			{
				slotFilamentPresent[currentSlot] = true;
			}

			return GCodeResult::ok;
		}

	case DriverState::timedOut:
		reply.copy("MMU2S timeout");
		operationState = OperationState::error;
		driverState = DriverState::idle;
		return GCodeResult::error;

	case DriverState::mmuError:
		reply.catf("MMU2S error E%04" PRIx32, lastErrorCode);
		if ((operationState == OperationState::load || operationState == OperationState::toolChange)
			&& currentSlot < MMU2SNumSlots)
		{
			slotFilamentPresent[currentSlot] = false;
		}
		operationState = OperationState::error;
		driverState = DriverState::idle;
		return GCodeResult::error;

	case DriverState::rejected:
		reply.copy("MMU2S: command rejected (MMU busy or not ready)");
		operationState = OperationState::error;
		driverState = DriverState::idle;
		return GCodeResult::error;

	case DriverState::protocolError:
		reply.copy("MMU2S protocol error");
		operationState = OperationState::error;
		driverState = DriverState::idle;
		return GCodeResult::error;

	case DriverState::idle:
	default:
		// During auto-recovery the re-handshake runs with driverState briefly idle between steps.
		if (isRecovering || operationState == OperationState::startupHandshake)
		{
			return GCodeResult::notFinished;
		}
		// Auto-recovery completed: the timed-out operation failed, but the driver is now
		// ready for new commands (operationState == idle).
		if (pendingError)
		{
			pendingError = false;
			reply.copy("MMU2S: slot likely empty (timed out, no filament detected)");
			return GCodeResult::error;
		}
		// Non-recoverable error (startup failure, protocol error) — requires M1750 R
		if (operationState == OperationState::error)
		{
			return GCodeResult::error;
		}
		return GCodeResult::ok;
	}
}

const char *MMU2S::OperationName() noexcept
{
	switch (operationState)
	{
	case OperationState::notConfigured:   return "not configured";
	case OperationState::idle:            return "idle";
	case OperationState::startupHandshake:return "startup handshake";
	case OperationState::toolChange:      return "tool change";
	case OperationState::load:            return "load";
	case OperationState::unload:          return "unload";
	case OperationState::eject:           return "eject";
	case OperationState::cut:             return "cut";
	case OperationState::home:            return "home";
	case OperationState::findaQuery:      return "FINDA query";
	case OperationState::directCommand:   return "direct command";
	case OperationState::resetting:       return "resetting";
	case OperationState::error:           return "error";
	default:                              return "unknown";
	}
}

void MMU2S::Diagnostics(const StringRef& reply) noexcept
{
	ReadLocker lock(mmuLock);
	reply.lcatf("MMU2S: %s, FINDA: %s, version: %s, slots:",
		OperationName(),
		findaState ? "filament" : "no filament",
		(versionString[0] != '\0') ? versionString : "unknown");
	for (uint8_t i = 0; i < MMU2SNumSlots; ++i)
	{
		reply.catf(" %u:%c", i, slotFilamentPresent[i] ? 'Y' : 'N');
	}
	reply.catf(" | bowden: %u mm, extra-load: %u mm", bowdenLengthMm, extraLoadDistanceMm);
	if (filamentSensorConfigured)
	{
		reply.catf(", sensor: %s (trip delay %" PRIu32 " ms)", filamentSensorPort.ReadDigital() ? "filament" : "clear", fsensorTripDelayMs);
	}
	else
	{
		reply.catf(", no sensor (timer %" PRIu32 " ms)", fsensorTriggerDelayMs);
	}
}

#endif	// SUPPORT_MMU2S

// End
