/*
 * ZProbe.cpp
 *
 *  Created on: 13 Feb 2018
 *      Author: David
 */

#include "ZProbe.h"
#include <Platform/RepRap.h>
#include <Movement/Move.h>
#include <GCodes/GCodes.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Storage/FileStore.h>
#include <Heating/Heat.h>
#include <Math/Matrix.h>
#include <Movement/MoveDebugFlags.h>

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...)							OBJECT_MODEL_FUNC_BODY(ZProbe, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...)			OBJECT_MODEL_FUNC_IF_BODY(ZProbe, _condition, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_ARRAY_IF(_condition, ...)		OBJECT_MODEL_FUNC_ARRAY_IF_BODY(ZProbe, _condition, __VA_ARGS__)
#define OBJECT_MODEL_ARRAY_COUNT(_value)				OBJECT_MODEL_ARRAY_COUNT_BODY(ZProbe, _value)
#define OBJECT_MODEL_ARRAY_VALUE(...)					OBJECT_MODEL_ARRAY_VALUE_BODY(ZProbe, __VA_ARGS__)

constexpr ObjectModelArrayTableEntry ZProbe::objectModelArrayTable[] =
{
	// 0. Offsets
	{
		nullptr,					// no lock needed
		OBJECT_MODEL_ARRAY_COUNT_NOSELF(reprap.GetGCodes().GetVisibleAxes()),
		OBJECT_MODEL_ARRAY_VALUE(self->offsets[context.GetLastIndex()], 2)
	},
	// 1. Speeds
	{
		nullptr,
		OBJECT_MODEL_ARRAY_COUNT((self->type == ZProbeType::scanningAnalog) ? 3 : 2),
		OBJECT_MODEL_ARRAY_VALUE(InverseConvertSpeedToMmPerMin(self->probeSpeeds[context.GetLastIndex()]), 1)
	},
	// 2. Temperature coefficients
	{
		nullptr,
		OBJECT_MODEL_ARRAY_COUNT_NOSELF(ARRAY_SIZE(ZProbe::temperatureCoefficients)),
		OBJECT_MODEL_ARRAY_VALUE(self->temperatureCoefficients[context.GetLastIndex()], 5)
	},
	// 3. Values
	{
		nullptr,
		OBJECT_MODEL_ARRAY_COUNT(self->GetNumReportedValues()),
		OBJECT_MODEL_ARRAY_VALUE(self->GetReportedValue(context.GetLastIndex()))
	},
	// 4. Dive heights
	{
		nullptr,
		OBJECT_MODEL_ARRAY_COUNT_NOSELF(ARRAY_SIZE(ZProbe::diveHeights)),
		OBJECT_MODEL_ARRAY_VALUE(self->diveHeights[context.GetLastIndex()], 1)
	},
#if SUPPORT_SCANNING_PROBES
	// 5. Scanning probe coefficients
	{
		nullptr,
		OBJECT_MODEL_ARRAY_COUNT_NOSELF(ARRAY_SIZE(ZProbe::scanCoefficients)),
		OBJECT_MODEL_ARRAY_VALUE(self->scanCoefficients[context.GetLastIndex()], 7)
	},
#endif
};

DEFINE_GET_OBJECT_MODEL_ARRAY_TABLE(ZProbe)

constexpr ObjectModelTableEntry ZProbe::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. Probe members
	{ "calibrationTemperature",		OBJECT_MODEL_FUNC(self->calibTemperature, 1), 												ObjectModelEntryFlags::none },
	{ "deployedByUser",				OBJECT_MODEL_FUNC(self->isDeployedByUser), 													ObjectModelEntryFlags::none },
	{ "disablesHeaters",			OBJECT_MODEL_FUNC((bool)self->misc.parts.turnHeatersOff), 									ObjectModelEntryFlags::none },
	{ "diveHeight",					OBJECT_MODEL_FUNC(self->diveHeights[0], 1), 												ObjectModelEntryFlags::obsolete },
	{ "diveHeights",				OBJECT_MODEL_FUNC_ARRAY(4), 																ObjectModelEntryFlags::none },
#if SUPPORT_SCANNING_PROBES
	{ "isCalibrated",				OBJECT_MODEL_FUNC_IF(self->IsScanning(), self->isCalibrated), 								ObjectModelEntryFlags::none },
#endif
	{ "lastStopHeight",				OBJECT_MODEL_FUNC(self->lastStopHeight, 3), 												ObjectModelEntryFlags::none },
	{ "maxProbeCount",				OBJECT_MODEL_FUNC((int32_t)self->misc.parts.maxTaps), 										ObjectModelEntryFlags::none },
#if SUPPORT_SCANNING_PROBES
	{ "measuredHeight",				OBJECT_MODEL_FUNC_IF(self->IsScanning() && self->isCalibrated, self->GetLatestHeight()),	ObjectModelEntryFlags::live },
#endif
	{ "offsets",					OBJECT_MODEL_FUNC_ARRAY(0), 																ObjectModelEntryFlags::none },
	{ "recoveryTime",				OBJECT_MODEL_FUNC(self->recoveryTime, 1), 													ObjectModelEntryFlags::none },
#if SUPPORT_SCANNING_PROBES
	{ "scanCoefficients",			OBJECT_MODEL_FUNC_ARRAY_IF(self->IsScanning(), 5), 											ObjectModelEntryFlags::none },
#endif
	{ "speeds",						OBJECT_MODEL_FUNC_ARRAY(1), 																ObjectModelEntryFlags::none },
	{ "temperatureCoefficients",	OBJECT_MODEL_FUNC_ARRAY(2), 																ObjectModelEntryFlags::none },
	{ "threshold",					OBJECT_MODEL_FUNC((int32_t)self->targetAdcValue), 											ObjectModelEntryFlags::none },
	{ "tolerance",					OBJECT_MODEL_FUNC(self->tolerance, 3), 														ObjectModelEntryFlags::none },
#if SUPPORT_SCANNING_PROBES
	{ "touchMode",					OBJECT_MODEL_FUNC_IF(self->IsScanning(), self, 1),											ObjectModelEntryFlags::none },
#endif
	{ "travelSpeed",				OBJECT_MODEL_FUNC(InverseConvertSpeedToMmPerMin(self->travelSpeed), 1), 					ObjectModelEntryFlags::none },
	{ "triggerHeight",				OBJECT_MODEL_FUNC(-self->offsets[Z_AXIS], 3), 												ObjectModelEntryFlags::none },
	{ "type",						OBJECT_MODEL_FUNC((int32_t)self->type), 													ObjectModelEntryFlags::none },
	{ "value",						OBJECT_MODEL_FUNC_ARRAY(3), 																ObjectModelEntryFlags::live },

#if SUPPORT_SCANNING_PROBES
	// 1. probe.touchMode members
	{ "active",						OBJECT_MODEL_FUNC(self->useTouchMode), 														ObjectModelEntryFlags::none },
	{ "speed",						OBJECT_MODEL_FUNC(InverseConvertSpeedToMmPerMin(self->touchModeProbeSpeed), 1),				ObjectModelEntryFlags::none },
	{ "threshold",					OBJECT_MODEL_FUNC(self->touchModeThreshold, 2), 											ObjectModelEntryFlags::none },
	{ "triggerHeight",				OBJECT_MODEL_FUNC(self->touchModeTriggerHeight, 3), 										ObjectModelEntryFlags::none },
#endif
};

constexpr uint8_t ZProbe::objectModelTableDescriptor[] =
{ 	1 + SUPPORT_SCANNING_PROBES,
	17 + 4 * SUPPORT_SCANNING_PROBES,
#if SUPPORT_SCANNING_PROBES
	4
#endif
};

DEFINE_GET_OBJECT_MODEL_TABLE(ZProbe)

ZProbe::ZProbe(unsigned int num, ZProbeType p_type) noexcept : EndstopOrZProbe(Z_AXIS), lastStopHeight(0.0), number(num), isDeployedByUser(false)
{
	SetDefaults();
	type = p_type;
}

void ZProbe::SetDefaults() noexcept
{
	targetAdcValue = DefaultZProbeADValue;
	for (float& offset : offsets)
	{
		offset = 0.0;
	}
	offsets[Z_AXIS] = -DefaultZProbeTriggerHeight;
	calibTemperature = DefaultZProbeTemperature;
	for (float& tc : temperatureCoefficients)
	{
		tc = 0.0;
	}
	diveHeights[0] = diveHeights[1] = DefaultZDive;
	probeSpeeds[0] = probeSpeeds[1] = probeSpeeds[2] = ConvertSpeedFromMmPerSec(DefaultProbingSpeed);
	travelSpeed = ConvertSpeedFromMmPerSec(DefaultZProbeTravelSpeed);
	recoveryTime = 0.0;
	tolerance = DefaultZProbeTolerance;
	misc.parts.maxTaps = DefaultZProbeTaps;
	misc.parts.turnHeatersOff = misc.parts.saveToConfigOverride = misc.parts.probingAway = false;
	type = ZProbeType::none;
	sensor = -1;
#if SUPPORT_DIFFERENTIAL_PROBES
	diffFastMillis = DefaultDiffFastMillis;
	diffSlowMillis = DefaultDiffSlowMillis;
	diffIntMillis = DefaultDiffIntMillis;
	diffHoldMillis = DefaultDiffHoldMillis;
	diffBlankMillis = DefaultDiffBlankMillis;
	diffOrder = DefaultDiffOrder;
	diffHpOrder = 1;							// a steeper bottom skirt eats the wide part of a real contact, so default to one pole
	diffDirection = DefaultDiffDirection;
	diffSource = 1;								// lowpass the raw signal, then subtract the baseline
	RecalcDifferentialShifts();
	ResetDifferential();
#endif
}

// Prepare to use this Z probe
void ZProbe::PrepareForUse(const bool probingAway) noexcept
{
	misc.parts.probingAway = probingAway;

	// We can't read temperature sensors from within the step ISR so calculate the actual trigger height now
	actualTriggerHeight = -offsets[Z_AXIS] + GetTriggerHeightCompensation();

#if SUPPORT_CAN_EXPANSION
	haveTriggerTime = false;
#endif
}

// Return the amount by which the trigger height is increased by temperature compensation
float ZProbe::GetTriggerHeightCompensation() const noexcept
{
	if (sensor >= 0)
	{
		TemperatureError err(TemperatureError::unknownError);
		const float temperature = reprap.GetHeat().GetSensorTemperature(sensor, err);
		if (err == TemperatureError::ok)
		{
			const float dt = temperature - calibTemperature;
			return (dt * temperatureCoefficients[0]) + (fsquare(dt) * temperatureCoefficients[1]);
		}
	}
	return 0.0;
}

// Get the dive height that is in effect for the next tap
float ZProbe::GetDiveHeight(int tapsDone) const noexcept
{
	if (FastThenSlowProbing())
	{
		++tapsDone;
	}
	return diveHeights[(tapsDone < 1) ? 0 : 1];
}

// Get the height we should move to when starting this probe.
// If firstTap is true, always move to the primary dive height.
// Otherwise if the secondary dive height is smaller than the primary, move to the secondary dive height plus the error.
float ZProbe::GetStartingHeight(bool firstTap, float previousHeightError) const noexcept
{
	return ((!firstTap && diveHeights[1] < diveHeights[0]) ? diveHeights[1] + previousHeightError : diveHeights[0])
			+ GetActiveModeTriggerHeight();
}

#if SUPPORT_SCANNING_PROBES

float ZProbe::GetScanningHeight() const noexcept
{
	return GetActualTriggerHeight();
}

float ZProbe::GetActiveModeTriggerHeight() const noexcept
{
	return (type == ZProbeType::scanningAnalog && useTouchMode) ? touchModeTriggerHeight : GetActualTriggerHeight();
}

float ZProbe::GetProbingSpeed(int tapsDone) const noexcept
{
	return (type == ZProbeType::scanningAnalog && useTouchMode) ? touchModeProbeSpeed
			: probeSpeeds[(tapsDone < 0) ? 0 : 1];
}

#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

bool ZProbe::WriteParameters(FileStore *f, unsigned int probeNumber) const noexcept
{
	const char *_ecv_array axisLetters = reprap.GetGCodes().GetAxisLetters();
	const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();
	String<StringLength256> scratchString;
	scratchString.printf("G31 K%u P%" PRIi32, probeNumber, targetAdcValue);
	for (size_t i = 0; i < numTotalAxes; ++i)
	{
		if (axisLetters[i] != 'Z')
		{
			scratchString.catf(" %c%.1f", axisLetters[i], (double)offsets[i]);
		}
	}
	scratchString.catf(" Z%.2f\n", (double)-offsets[Z_AXIS]);
	return f->Write(scratchString.c_str());
}

#endif

int32_t ZProbe::GetReading() const noexcept
{
	int32_t zProbeVal = 0;
#if SUPPORT_CAN_EXPANSION
	if (IsRemote())
	{
		zProbeVal = GetRawReading();
	}
	else
#endif
	{
		const Platform& p = reprap.GetPlatform();
		if (type == ZProbeType::unfilteredDigital || type == ZProbeType::blTouch || (p.GetZProbeOnFilter().IsValid() && p.GetZProbeOffFilter().IsValid()))
		{
			switch (type)
			{
			case ZProbeType::analog:				// Simple or intelligent IR sensor
			case ZProbeType::alternateAnalog:		// Alternate sensor
			case ZProbeType::digital:				// Switch connected to Z probe input
#if SUPPORT_DIFFERENTIAL_PROBES
			case ZProbeType::differentialAnalog:	// keep the boxcar here: the detector uses GetRawReading in the tick, and
													// macros that sample value[0] after a jab need the averaged excursion, not an instant
#endif
				zProbeVal = (int32_t)((p.GetZProbeOnFilter().GetSum() + p.GetZProbeOffFilter().GetSum()) / (int32_t)(2 * ZProbeAverageReadings));
				break;

			case ZProbeType::dumbModulated:		// Dumb modulated IR sensor.
				// We assume that zProbeOnFilter and zProbeOffFilter average the same number of readings.
				// Because of noise, it is possible to get a negative reading, so allow for this.
				zProbeVal = ((int32_t)p.GetZProbeOnFilter().GetSum() - (int32_t)p.GetZProbeOffFilter().GetSum())/(int32_t)ZProbeAverageReadings;
				break;

			case ZProbeType::unfilteredDigital:		// Switch connected to Z probe input, no filtering
			case ZProbeType::blTouch:				// blTouch is now unfiltered too
			case ZProbeType::scanningAnalog:		// scanning analog probes are unfiltered for speed
				zProbeVal = GetRawReading();
				break;

			case ZProbeType::zMotorStall:
#if HAS_STALL_DETECT
				{
					const LocalDriversBitmap zDrivers = reprap.GetMove().GetAxisDriversConfig(Z_AXIS).GetLocalDriversBitmap();
					zProbeVal = (GetStalledDrivers(zDrivers).IsNonEmpty()) ? 1000 : 0;
				}
#else
				return 1000;
#endif
				break;

			default:
				return 1000;
			}
		}
	}

	return zProbeVal;
}

int32_t ZProbe::GetSecondaryValues(int32_t& v1) const noexcept
{
	const Platform& p = reprap.GetPlatform();
	if (p.GetZProbeOnFilter().IsValid() && p.GetZProbeOffFilter().IsValid())
	{
		switch (type)
		{
		case ZProbeType::dumbModulated:		// modulated IR sensor
			v1 = (int)(p.GetZProbeOnFilter().GetSum() / ZProbeAverageReadings);	// pass back the reading with IR turned on
			return 1;
		default:
			break;
		}
	}
	return 0;
}

// Return the number of entries in the object model 'value' array for this probe type.
// Index 0 is always the reading that the trigger test uses. For probes with a local analog input we also report the unfiltered reading and the
// full-resolution ADC reading, because the filtered value hides both the true shape of a fast transient and the real ADC sample rate.
size_t ZProbe::GetNumReportedValues() const noexcept
{
#if SUPPORT_CAN_EXPANSION
	if (IsRemote())
	{
		return 1;											// a remote probe has no local ADC, so the extra values would be meaningless
	}
#endif
	switch (type)
	{
	case ZProbeType::dumbModulated:
		return 2;											// [1] is the reading with the IR emitter turned on

#if SUPPORT_DIFFERENTIAL_PROBES
	case ZProbeType::differentialAnalog:
		return 6;											// [3] delta, [4] filtered, [5] trigger state -1/0/+1
#endif

	case ZProbeType::analog:
	case ZProbeType::alternateAnalog:
	case ZProbeType::scanningAnalog:
		return 3;											// [1] is the unfiltered reading, [2] is the full-resolution ADC reading

	default:
		return 1;
	}
}

// Return entry 'index' of the object model 'value' array. See GetNumReportedValues for which indices are valid for each probe type.
int32_t ZProbe::GetReportedValue(unsigned int index) const noexcept
{
	switch (index)
	{
	case 1:
		if (type == ZProbeType::dumbModulated)
		{
			int32_t v1 = 0;
			(void)GetSecondaryValues(v1);
			return v1;
		}
		return GetRawReading();

	case 2:
		return (int32_t)GetUnscaledAnalogReading();

#if SUPPORT_DIFFERENTIAL_PROBES
	case 3:
		return diffDelta;
	case 4:
		return diffFiltered;
	case 5:
		return diffTrigger;
#endif

	default:
		return GetReading();
	}
}

// Test whether we are at or near the stop. May be called from an ISR as well and from a normal task context.
bool ZProbe::Stopped() const noexcept
{
#if SUPPORT_DIFFERENTIAL_PROBES
	if (type == ZProbeType::differentialAnalog)
	{
		return diffTrigger != 0;				// a single byte, so the step ISR can never see a torn value
	}
#endif
#if SUPPORT_SCANNING_PROBES
	if (type == ZProbeType::scanningAnalog && useTouchMode)
	{
		return touchTriggered;
	}
#endif

	const int32_t zProbeVal = GetReading();
	return zProbeVal >= targetAdcValue;
}

// Check whether the probe is triggered and return the action that should be performed. Called from the step ISR.
EndstopHitDetails ZProbe::CheckTriggered() noexcept
{
	bool b = Stopped();
	if (misc.parts.probingAway)
	{
		b = !b;
	}

	EndstopHitDetails rslt;			// initialised by default constructor
	if (b)
	{
		rslt.SetAction(EndstopHitAction::stopAll
#if SUPPORT_CAN_EXPANSION
						, whenTriggered, haveTriggerTime
#endif
					  );
		rslt.isZProbe = true;
	}
	return rslt;
}

// This is called by the ISR to acknowledge that it is acting on the return from calling CheckTriggered. Called from the step ISR.
// Return true if we have finished with this endstop or probe in this move.
bool ZProbe::Acknowledge(EndstopHitDetails what) noexcept
{
#if SUPPORT_CAN_EXPANSION
	haveTriggerTime = false;
#endif
	return what.GetAction() == EndstopHitAction::stopAll;
}

GCodeResult ZProbe::HandleG31(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	GCodeResult err = GCodeResult::ok;
	bool seen = false;

	// Warning - don't change any values until we know that we are not going to return notFinished!
	// This because users may use the existing values when calculating the new ones (e.g. "make babystepping permanent" macro)

	// Do the temperature coefficient first because it may return notFinished
	int8_t newSensor;
	if (gb.Seen('H'))
	{
		seen = true;
		newSensor = gb.GetIValue();
	}
	else
	{
		newSensor = sensor;
	}

	if (gb.Seen('T'))
	{
		seen = true;
		for (float& tc : temperatureCoefficients)
		{
			tc = 0.0;
		}

		TemperatureError terr(TemperatureError::unknownError);
		const float currentTemperature = reprap.GetHeat().GetSensorTemperature(newSensor, terr);
		if (terr == TemperatureError::unknownSensor)
		{
			reply.printf("Temperature coefficients ignored due to invalid sensor number %d", newSensor);
			err = GCodeResult::warning;
		}
		else
		{
			size_t numValues = ARRAY_SIZE(temperatureCoefficients);
			gb.GetFloatArray(temperatureCoefficients, numValues, false);
			float newCalibTemperature = DefaultZProbeTemperature;

			if (gb.Seen('S'))
			{
				newCalibTemperature = gb.GetFValue();
			}
			else if (terr == TemperatureError::ok)
			{
				newCalibTemperature = currentTemperature;
			}
			else if (!gb.IsTimerRunning())				// the sensor may have only just been configured, so give it 500ms to produce a reading
			{
				gb.StartTimer();
				return GCodeResult::notFinished;
			}
			else if (millis() - gb.WhenTimerStarted() < 500)
			{
				return GCodeResult::notFinished;
			}
			else
			{
				gb.StopTimer();
				reply.printf("Sensor %d did not provide a valid temperature reading, using default", newSensor);
				err = GCodeResult::warning;
			}
			gb.StopTimer();
			calibTemperature = newCalibTemperature;
		}
	}

	// After this we don't return notFinished, so it is safe to amend values directly
	const char *_ecv_array axisLetters = reprap.GetGCodes().GetAxisLetters();
	const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();
	for (size_t i = 0; i < numTotalAxes; ++i)
	{
		if (axisLetters[i] != 'Z')
		{
			gb.TryGetFValue(axisLetters[i], offsets[i], seen);
		}
	}

	{
		float triggerHeight;
		if (gb.TryGetFValue(axisLetters[Z_AXIS], triggerHeight, seen))
		{
			offsets[Z_AXIS] = -triggerHeight;				// logically, the Z offset of the Z probe is the negative of the trigger height
		}
	}

	if (gb.Seen('P'))
	{
		seen = true;
		targetAdcValue = gb.GetIValue();
	}

	if (seen)
	{
		sensor = newSensor;
		reprap.SensorsUpdated();
		if (gb.LatestMachineState().runningM501)
		{
			misc.parts.saveToConfigOverride = true;			// we are loading these parameters from config-override.g, so a subsequent M500 should save them to config-override.g
		}
	}
	else
	{
		const int v0 = GetReading();
		reply.printf("Z probe %u: current reading %d", number, v0);
		int32_t v1;
		if (GetSecondaryValues(v1) == 1)
		{
			reply.catf(" (%" PRIi32 ")", v1);
		}
		reply.catf(", threshold %" PRIi32 ", trigger height %.3f", targetAdcValue, (double)-offsets[Z_AXIS]);
		if (temperatureCoefficients[0] != 0.0 || temperatureCoefficients[1] != 0.0)
		{
			reply.catf(" at %.1f" DEGREE_SYMBOL "C, temperature coefficients [%.1f/" DEGREE_SYMBOL "C, %.1f/" DEGREE_SYMBOL "C^2]",
						(double)calibTemperature, (double)temperatureCoefficients[0], (double)temperatureCoefficients[1]);
		}
		reply.cat(", offsets");
		for (size_t i = 0; i < numTotalAxes; ++i)
		{
			if (axisLetters[i] != 'Z')
			{
				reply.catf(" %c%.1f", axisLetters[i], (double)offsets[i]);
			}
		}
	}
	return err;
}

GCodeResult ZProbe::Configure(GCodeBuffer& gb, const StringRef &reply, bool& seen) THROWS(GCodeException)
{
	if (gb.Seen('H'))										// dive heights
	{
		size_t numHeights = 2;
		gb.GetFloatArray(diveHeights, numHeights, true);
		seen = true;
	}

	if (gb.Seen('F'))										// feed rate i.e. probing speed
	{
		float userProbeSpeeds[3];
		size_t numSpeeds = 3;
		gb.GetFloatArray(userProbeSpeeds, numSpeeds, true);
		probeSpeeds[0] = ConvertSpeedFromMmPerMin(userProbeSpeeds[0]);
		probeSpeeds[1] = ConvertSpeedFromMmPerMin(userProbeSpeeds[1]);
		probeSpeeds[2] = (numSpeeds == 3) ? ConvertSpeedFromMmPerMin(userProbeSpeeds[2]) : probeSpeeds[0];
		seen = true;
	}

	if (gb.Seen('T'))		// travel speed to probe point
	{
		travelSpeed = gb.GetSpeedFromMm(false);
		seen = true;
	}

	if (gb.Seen('B'))
	{
		misc.parts.turnHeatersOff = (gb.GetIValue() == 1);
		seen = true;
	}

	gb.TryGetFValue('R', recoveryTime, seen);				// Z probe recovery time
	gb.TryGetFValue('S', tolerance, seen);					// tolerance when multi-tapping

	if (gb.Seen('A'))
	{
		misc.parts.maxTaps = min<uint32_t>(gb.GetUIValue(), ZProbe::MaxTapsLimit);
		seen = true;
	}

#if SUPPORT_DIFFERENTIAL_PROBES
	if (type == ZProbeType::differentialAnalog)
	{
		const GCodeResult rslt = ConfigureDifferential(gb, seen);
		if (rslt != GCodeResult::ok)
		{
			reply.copy("M558: slow time constant W must be greater than fast time constant V");
			return rslt;
		}
	}
#endif

	if (seen)
	{
		reprap.SensorsUpdated();
		return GCodeResult::ok;
	}

	reply.printf("Z Probe %u: type %u", number, (unsigned int)type);
	const GCodeResult rslt = AppendPinNames(reply);
	reply.catf(", dive heights %.1f,%.1fmm, probe speeds %d,%d",
					(double)diveHeights[0], (double)diveHeights[1],
					(int)InverseConvertSpeedToMmPerMin(probeSpeeds[0]),
					(int)InverseConvertSpeedToMmPerMin(probeSpeeds[1]));
	if (type == ZProbeType::scanningAnalog)
	{
		reply.catf(",%d", (int)InverseConvertSpeedToMmPerMin(probeSpeeds[2]));
	}
	reply.catf("mm/min, travel speed %dmm/min, recovery time %.2f sec, heaters %s, max taps %u, max diff %.2f",
					(int)InverseConvertSpeedToMmPerMin(travelSpeed),
					(double)recoveryTime,
					(misc.parts.turnHeatersOff) ? "suspended" : "normal",
						misc.parts.maxTaps, (double)tolerance);
#if SUPPORT_DIFFERENTIAL_PROBES
	if (type == ZProbeType::differentialAnalog)
	{
		AppendDifferentialReport(reply);
	}
#endif
	return rslt;
}

#if SUPPORT_DIFFERENTIAL_PROBES

// Convert a time constant in milliseconds to a shift count for a single-pole EMA sampled by the 1kHz tick.
// Clamped on the way in, because a shift of 32 or more is undefined behaviour and the value comes from a G-code parameter.
static uint8_t MillisToShift(uint32_t ms) noexcept
{
	if (ms < 1) { ms = 1; }
	if (ms > MaxDiffTimeConstantMillis) { ms = MaxDiffTimeConstantMillis; }
	unsigned int sh = 31u - (unsigned int)__builtin_clz(ms);						// floor(log2(ms))
	if (sh < 12 && (ms - (1u << sh)) > ((1u << (sh + 1)) - ms)) { ++sh; }			// round to nearest power of two
	return (uint8_t)min<unsigned int>(sh, 12);
}

void ZProbe::RecalcDifferentialShifts() noexcept
{
	if (diffOrder < 1) { diffOrder = 1; }
	if (diffOrder > MaxDiffOrder) { diffOrder = MaxDiffOrder; }
	if (diffHpOrder < 1) { diffHpOrder = 1; }
	if (diffHpOrder > MaxDiffOrder) { diffHpOrder = MaxDiffOrder; }
	diffFastShift = MillisToShift(diffFastMillis);
	diffSlowShift = MillisToShift(diffSlowMillis);
	// Each pole of the cascade gets 1/order of the requested time constant, so changing the order changes the rolloff and not the delay
	diffIntShift = MillisToShift(max<uint32_t>(1, diffIntMillis / diffOrder));
}

void ZProbe::ResetDifferential() noexcept
{
	AtomicCriticalSectionLocker lock;
	diffPrimeTicks = DiffNotPrimed;
	diffHoldTicks = 0;
	diffBlankTicks = 0;
	diffDelta = 0;								// separate statements: reading back an assignment to a volatile is deprecated
	diffFiltered = 0;
	diffTrigger = 0;
	diffProbing = false;
	diffArmedBaseline = 0;
	diffFastQ = diffSlowQ = 0;
	for (int32_t& v : diffLpQ) { v = 0; }
	for (int32_t& v : diffHpQ) { v = 0; }
}

// Capture the baseline and start the blanking window. Called from SetProbing, i.e. at standstill immediately before the probing move.
void ZProbe::ArmDifferential(bool isProbing) noexcept
{
	AtomicCriticalSectionLocker lock;
	diffProbing = isProbing;
	diffArmedBaseline = diffFastQ >> DiffFracBits;
	diffBlankTicks = (isProbing) ? diffBlankMillis : 0;
	diffHoldTicks = 0;
	diffTrigger = 0;
}

// The detector. Called from the 1kHz tick ISR, so: integer only (the tick must not use floating point), O(1), and it publishes
// only single naturally-atomic words. Every cycle spent here is step-interrupt latency, once per millisecond, during printing too.
void ZProbe::ProcessDifferentialSample(int32_t raw) noexcept
{
	const int32_t x = raw << DiffFracBits;

	if (diffPrimeTicks == DiffNotPrimed)
	{
		diffFastQ = diffSlowQ = x;
		for (int32_t& v : diffLpQ) { v = (diffSource == 1) ? x : 0; }
		diffHpQ[0] = x;							// stage 0 of the highpass sees DC, later stages see ~0
		for (size_t i = 1; i < MaxDiffOrder; ++i) { diffHpQ[i] = 0; }
		diffPrimeTicks = (uint16_t)min<uint32_t>(4u << diffSlowShift, 8000);		// settle for 4 time constants before we can trigger
		diffHoldTicks = diffBlankTicks = 0;
		diffTrigger = 0;
		return;
	}

	diffFastQ += (x - diffFastQ) >> diffFastShift;
	diffSlowQ += (x - diffSlowQ) >> diffSlowShift;
	const int32_t deltaQ = diffFastQ - diffSlowQ;

	// A bandpass built from two cascades: diffOrder lowpass poles at E/order set the top skirt, diffHpOrder highpass poles at W set the bottom.
	// L1 lowpasses first so the baseline tracker sees the unsmoothed signal and stays free to follow drift; L0 highpasses first, which puts the
	// baseline through the lowpass too and delays it. They are not equivalent in practice even though they would be for ideal LTI blocks.
	int32_t v = diffFastQ;
	if (diffSource == 1)
	{
		for (unsigned int k = 0; k < diffOrder; ++k)   { diffLpQ[k] += (v - diffLpQ[k]) >> diffIntShift; v = diffLpQ[k]; }
		for (unsigned int k = 0; k < diffHpOrder; ++k) { diffHpQ[k] += (v - diffHpQ[k]) >> diffSlowShift; v -= diffHpQ[k]; }
	}
	else
	{
		for (unsigned int k = 0; k < diffHpOrder; ++k) { diffHpQ[k] += (v - diffHpQ[k]) >> diffSlowShift; v -= diffHpQ[k]; }
		for (unsigned int k = 0; k < diffOrder; ++k)   { diffLpQ[k] += (v - diffLpQ[k]) >> diffIntShift; v = diffLpQ[k]; }
	}

	diffDelta = (int16_t)max<int32_t>(-32000, min<int32_t>(32000, deltaQ >> DiffFracBits));
	diffFiltered = (int16_t)max<int32_t>(-32000, min<int32_t>(32000, v >> DiffFracBits));

	if (diffPrimeTicks != 0) { --diffPrimeTicks; return; }
	if (diffBlankTicks != 0) { --diffBlankTicks; return; }							// ride out the probing move's acceleration ramp

	const int32_t f = diffFiltered;
	int8_t sign = 0;
	if (diffDirection != 2 && f >= targetAdcValue) { sign = 1; }
	else if (diffDirection != 1 && f <= -targetAdcValue) { sign = -1; }

	if (sign == 0 && diffProbing)
	{
		// Absolute backstop against the armed baseline, so a contact too slow for the bandpass is still caught.
		// This makes the differential type a superset of the plain analog one rather than a trade.
		const int32_t dev = (diffFastQ >> DiffFracBits) - diffArmedBaseline;
		if (diffDirection != 2 && dev >= targetAdcValue) { sign = 1; }
		else if (diffDirection != 1 && dev <= -targetAdcValue) { sign = -1; }
	}

	if (sign != 0)
	{
		diffHoldTicks = diffHoldMillis;												// retriggerable monostable
		diffTrigger = sign;
	}
	else if (diffHoldTicks != 0 && --diffHoldTicks == 0)
	{
		diffTrigger = 0;
	}
}

// The detector state as PiezoLog sample flags. Tick ISR only, so it reads the ISR-owned counters without locking.
uint16_t ZProbe::GetDiffLogFlags() const noexcept
{
	uint16_t f = 0;
	const int8_t t = diffTrigger;
	if (t != 0) { f |= 1u << 10; }
	if (t < 0) { f |= 1u << 11; }
	if (diffProbing) { f |= 1u << 12; }
	if (diffBlankTicks != 0) { f |= 1u << 13; }
	if (diffPrimeTicks != 0) { f |= 1u << 14; }
	return f;
}

// Parse the differential-probe parameters of M558. Returns error only if the configuration is self-contradictory.
GCodeResult ZProbe::ConfigureDifferential(GCodeBuffer& gb, bool& seen) THROWS(GCodeException)
{
	// Only the filter shape (V W E O J L) invalidates the accumulators. Direction, hold and blanking are
	// decisions made on top of the filter output, so changing them must NOT re-prime - a macro that flips
	// D between probe points would otherwise be blind for ~4*W after every switch.
	bool changed = false, filterChanged = false;
	if (gb.Seen('V')) { diffFastMillis = (uint16_t)gb.GetLimitedUIValue('V', 1, MaxDiffTimeConstantMillis + 1); filterChanged = true; }
	if (gb.Seen('W')) { diffSlowMillis = (uint16_t)gb.GetLimitedUIValue('W', 1, MaxDiffTimeConstantMillis + 1); filterChanged = true; }
	if (gb.Seen('E')) { diffIntMillis = (uint16_t)gb.GetLimitedUIValue('E', 1, MaxDiffTimeConstantMillis + 1); filterChanged = true; }
	if (gb.Seen('O')) { diffOrder = (uint8_t)gb.GetLimitedUIValue('O', 1, MaxDiffOrder + 1); filterChanged = true; }
	if (gb.Seen('J')) { diffHpOrder = (uint8_t)gb.GetLimitedUIValue('J', 1, MaxDiffOrder + 1); filterChanged = true; }
	if (gb.Seen('L')) { diffSource = (uint8_t)gb.GetLimitedUIValue('L', 0, 2); filterChanged = true; }
	if (gb.Seen('D')) { diffDirection = (uint8_t)gb.GetLimitedUIValue('D', 0, 3); changed = true; }
	if (gb.Seen('Q')) { diffHoldMillis = (uint16_t)gb.GetLimitedUIValue('Q', 1, 5001); changed = true; }
	if (gb.Seen('U')) { diffBlankMillis = (uint16_t)gb.GetLimitedUIValue('U', 0, 5001); changed = true; }

	if (filterChanged)
	{
		if (diffSlowMillis <= diffFastMillis)
		{
			return GCodeResult::error;												// otherwise the passband is empty and it can never trigger
		}
		RecalcDifferentialShifts();
		ResetDifferential();														// time constants changed, so the filter state is meaningless
	}
	if (changed || filterChanged)
	{
		seen = true;
	}
	return GCodeResult::ok;
}

void ZProbe::AppendDifferentialReport(const StringRef& reply) const noexcept
{
	reply.catf(", differential: threshold %" PRIi32 ", filter %s, V%ums W%ums E%ums O%u (%udB/oct) J%u (%udB/oct), D%u, Q%ums, U%ums, live delta %d filtered %d trig %d",
					targetAdcValue,
					(diffSource == 1) ? "raw" : "delta",
					diffFastMillis, diffSlowMillis, diffIntMillis, diffOrder, 6u * diffOrder,
					diffHpOrder, 6u * diffHpOrder,
					diffDirection, diffHoldMillis, diffBlankMillis,
					(int)diffDelta, (int)diffFiltered, (int)diffTrigger);
}

#endif

// Default implementation of SendProgram, overridden in some classes
GCodeResult ZProbe::SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply) noexcept
{
	reply.copy("This configuration of Z probe does not support programming");
	return GCodeResult::error;
}

void ZProbe::SetLastStoppedHeight(float h) noexcept
{
	lastStopHeight = h;
	reprap.SensorsUpdated();
}

#if SUPPORT_SCANNING_PROBES

// Scanning support
GCodeResult ZProbe::SetScanningCoefficients(float aParam, float bParam, float cParam) noexcept
{
	scanCoefficients[0] = 0.0;
	scanCoefficients[1] = aParam;
	scanCoefficients[2] = bParam;
	scanCoefficients[3] =  cParam;
	isCalibrated = true;
	reprap.SensorsUpdated();
	return GCodeResult::ok;
}

// Set touch mode parameters (M558.3)
GCodeResult ZProbe::SetTouchModeParameters(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	bool seen = false;
	gb.TryGetBValue('S', useTouchMode, seen);
	gb.TryGetFValue('H', touchModeTriggerHeight, seen);
	gb.TryGetLimitedFValue('V', touchModeThreshold, seen, 0.0, TouchModeMaxThreshold);
	{
		float speed;
		if (gb.TryGetPositiveFValue('F', speed, seen))
		{
			touchModeProbeSpeed = ConvertSpeedFromMmPerMin(speed);
		}
	}

	if (seen)
	{
		reprap.SensorsUpdated();
	}
	else
	{
		reply.printf("Z probe %u touch mode: %s" "active, speed %.1fmm/min, trigger height %.2fmm, threshold %.2f",
						number, (useTouchMode) ? "" : "not ", (double)InverseConvertSpeedToMmPerMin(touchModeProbeSpeed), (double)touchModeTriggerHeight, (double)touchModeThreshold);
	}
	return GCodeResult::ok;
}

float ZProbe::ConvertReadingToHeightDifference(int32_t reading) const noexcept
{
	const float diff = (float)(reading - targetAdcValue);
	return scanCoefficients[0] + diff * (scanCoefficients[1] + diff * (scanCoefficients[2] + diff * scanCoefficients[3]));
}

GCodeResult ZProbe::ReportScanningCoefficients(const StringRef& reply) noexcept
{
	if (isCalibrated)
	{
		reply.printf("Scanning probe offset: %.3fmm, A: %.3e, B: %.3e, C: %.3e", (double)scanCoefficients[0], (double)scanCoefficients[1], (double)scanCoefficients[2], (double)scanCoefficients[3]);
		return GCodeResult::ok;
	}

	reply.copy("Probe has not been calibrated");
	return GCodeResult::error;
}

void ZProbe::CalibrateScanningProbe(const int32_t calibrationReadings[], size_t numCalibrationReadingsTaken, float heightChangePerPoint, const StringRef& reply) noexcept
{
	const int32_t referenceReading = calibrationReadings[numCalibrationReadingsTaken/2];
	FixedMatrix<float, 4, 5> matrix;
	matrix.Fill(0.0);

	// Do a least squares fit of a cubic polynomial to the data
	// Store { N, sum(X), sum(X^2), sum(X^3) sum(Y) } in row 0
	// Store { sum(X), sum(X^2), sum(X^3), sum(X^4), sum(XY) } in row 1
	// Store { sum(X^2), sum(X^3), sum(X^4), sum(X^5), sum(X^2Y) } in row 2
	// Store { sum(X^3), sum(X^4), sum(X^5), sum(X^6), sum(X^3Y) } in row 3
	matrix(0, 0) = (float)numCalibrationReadingsTaken;
	for (size_t i = 0; i < numCalibrationReadingsTaken; ++i)
	{
		const float y = ((int)(numCalibrationReadingsTaken/2) - (int)i) * heightChangePerPoint;		// the height different from the trigger height
		const float x = (float)(calibrationReadings[i] - referenceReading);							// the difference in reading from the target reading at the trigger height
		const float x2 = fsquare(x);
		const float x3 = x * x2;
		const float x4 = fsquare(x2);
		const float x5 = x * x4;
		const float x6 = x3 * x3;
		const float xy = x * y;
		const float x2y = x2 * y;
		const float x3y = x3 * y;
		matrix(0, 1) += x;
		matrix(0, 2) += x2;
		matrix(0, 3) += x3;
		matrix(0, 4) += y;
		matrix(1, 0) += x;
		matrix(1, 1) += x2;
		matrix(1, 2) += x3;
		matrix(1, 3) += x4;
		matrix(1, 4) += xy;
		matrix(2, 0) += x2;
		matrix(2, 1) += x3;
		matrix(2, 2) += x4;
		matrix(2, 3) += x5;
		matrix(2, 4) += x2y;
		matrix(3, 0) += x3;
		matrix(3, 1) += x4;
		matrix(3, 2) += x5;
		matrix(3, 3) += x6;
		matrix(3, 4) += x3y;
	}
	matrix.GaussJordan(4, 5);

	scanCoefficients[0] = matrix(0, 4);
	scanCoefficients[1] = matrix(1, 4);
	scanCoefficients[2] = matrix(2, 4);
	scanCoefficients[3] = matrix(3, 4);
	targetAdcValue = referenceReading;
	isCalibrated = true;
	reprap.SensorsUpdated();
	ReportScanningCoefficients(reply);

	// Calculate the RMS error after subtracting the mean error
	float sumOfErrorSquares = 0.0;
	for (size_t i = 0; i < numCalibrationReadingsTaken; ++i)
	{
		const float readingDiff = (float)(calibrationReadings[i] - referenceReading);
		const float actualHeightDiff = ((int)(numCalibrationReadingsTaken/2) - (int)i) * heightChangePerPoint;	// the height different from the trigger height
		const float predictedHeightDiff = scanCoefficients[0] + readingDiff * (scanCoefficients[1] + readingDiff * (scanCoefficients[2] + readingDiff * scanCoefficients[3]));	// the predicted value from the fitted curve
		sumOfErrorSquares += fsquare(predictedHeightDiff - actualHeightDiff);
	}

	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::ZProbing))
	{
		debugPrintf("NumReadings: %u\nHeight interval: %.5f\nScanned data:", numCalibrationReadingsTaken, (double)heightChangePerPoint);
		for (size_t i = 0; i < numCalibrationReadingsTaken; ++i)
		{
			debugPrintf("%c%" PRIi32, (i == 0) ? ' ' : ',', calibrationReadings[i] - referenceReading);
		}
		debugPrintf("\nFit: %.5f,%.5g,%.5g,%.5g\n", (double)scanCoefficients[0], (double)scanCoefficients[1], (double)scanCoefficients[2], (double)scanCoefficients[3]);
	}

	reply.catf(", reading at trigger height %" PRIi32 ", rms error %.3fmm",
					referenceReading, (double)sqrtf(sumOfErrorSquares/(float)numCalibrationReadingsTaken));
}

#endif

// End
