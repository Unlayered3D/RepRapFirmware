/*
 * InputShaper.h
 *
 *  Created on: 20 Feb 2021
 *      Author: David
 */

#ifndef SRC_MOVEMENT_AXISSHAPER_H_
#define SRC_MOVEMENT_AXISSHAPER_H_

#include <RepRapFirmware.h>
#include <General/NamedEnum.h>
#include <ObjectModel/ObjectModel.h>

// These names must be in alphabetical order and lowercase.
//
// The upstream entries above the marker keep that ordering. The fork's negative shapers are
// appended below it instead of being merged into the alphabetical run, deliberately:
// NamedEnumLookup (RRFLibraries/src/General/NamedEnum.cpp) is a linear strcmp scan, so ordering
// does not affect name lookup, whereas inserting them would renumber none/zvd/zvdd/zvddd and
// would rewrite an upstream-owned block that upstream also appends to - guaranteeing a conflict
// on every future upstream shaper.
NamedEnum(InputShaperType, uint8_t,
	custom,
	ei2,
	ei3,
	mzv,
	none,
	zvd,
	zvdd,
	zvddd,
	// --- Unlayered fork additions, appended (see note above) ---
	nzvum,
	nzvdum,
	neium
);

namespace InputShapingDebugFlags
{
	// Bit numbers in the input shaping debug bitmap
	constexpr unsigned int Errors = 0;
	constexpr unsigned int Retries = 1;
	constexpr unsigned int All = 2;
}

#if SUPPORT_REMOTE_COMMANDS
struct CanMessageSetInputShapingV1;
#endif

class AxisShaper INHERIT_OBJECT_MODEL
{
public:
	AxisShaper() noexcept;

	// Configure input shaping
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// process M593

	size_t GetNumImpulses() const noexcept { return numImpulses; }
	motioncalc_t GetImpulseSize(size_t n) const noexcept { return coefficients[n]; }
	uint32_t GetImpulseDelay(size_t n) const noexcept { return delays[n]; }
	uint32_t GetPrepareAdvanceTime() const noexcept { return prepareAdvanceTime; }
	uint32_t GetShapingTime() const noexcept { return shapingTime; }

#if SUPPORT_REMOTE_COMMANDS
	// Handle a request from the master board to set input shaping parameters
	GCodeResult EutSetInputShaping(const CanMessageSetInputShapingV1& msg, size_t dataLength, const StringRef& reply) noexcept;
#endif

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

private:

#if SUPPORT_CAN_EXPANSION
	GCodeResult UpdateRemoteInputShaping(const StringRef& reply) const noexcept;
#endif

	static constexpr float MinimumInputShapingFrequency = 4.0;
	static constexpr float MaximumInputShapingFrequency = 400.0;
	static constexpr float DefaultFrequency = 40.0;
	static constexpr unsigned int MaxImpulses = 5;
	static constexpr float DefaultDamping = 0.05;

	// Impulse-time polynomials for the fork's negative input shapers. There is no closed-form
	// solution for these, so each row is a least-squares cubic fit in the damping ratio zeta,
	// with the coefficients in ascending powers:
	//
	//     t_N / (1/frequency) = M[0] + M[1]*zeta + M[2]*zeta^2 + M[3]*zeta^3
	//
	// The name is Mt<N><shaper>, where N is the impulse index, so Mt3nzvdum is the normalised
	// time of the third impulse of the nzvdum shaper. Consumed in AxisShaper::Recalc, which
	// multiplies each result by StepClockRate/frequency to get delays[N] in step clocks.
	// Derivation: Time-Optimal Negative Input Shapers,
	// https://asmedigitalcollection.asme.org/dynamicsystems/article/119/2/198/442325
	static constexpr float Mt2nzvum[4] = {0.16724f, 0.27242f, 0.20345f, 0.0f};
	static constexpr float Mt3nzvum[4] = {0.33323f, 0.00533f, 0.17914f, 0.20125f};

	static constexpr float Mt2nzvdum[4] = {0.08945f, 0.28411f, 0.23013f, 0.16401f};
	static constexpr float Mt3nzvdum[4] = {0.36613f, -0.08833f, 0.24048f, 0.17001f};
	static constexpr float Mt4nzvdum[4] = {0.64277f, 0.29103f, 0.23262f, 0.43784f};
	static constexpr float Mt5nzvdum[4] = {0.73228f, 0.00992f, 0.49385f, 0.38633f};

	static constexpr float Mt2neium[4] = {0.09374f, 0.31903f, 0.13582f, 0.65274f};
	static constexpr float Mt3neium[4] = {0.36798f, -0.05894f, 0.13641f, 0.63266f};
	static constexpr float Mt4neium[4] = {0.64256f, 0.28595f, 0.26334f, 0.24999f};
	static constexpr float Mt5neium[4] = {0.73664f, 0.00162f, 0.52749f, 0.19208f};

	// Input shaping parameters input by the user
	InputShaperType type;								// the type of the input shaper, from which we can find its name
	float frequency;									// the undamped frequency in Hz
	float zeta;											// the damping ratio, see https://en.wikipedia.org/wiki/Damping. 0 = undamped, 1 = critically damped.

	// Parameters that fully define the shaping
	unsigned int numImpulses;							// the number of impulses
	motioncalc_t coefficients[MaxImpulses];				// the coefficients of all the impulses, must add up to 1.0
	uint32_t delays[MaxImpulses];						// the start delay in step clocks of each impulse, first one is normally zero
	uint32_t shapingTime;								// how long after its nominal end time the move is still in flight
	uint32_t prepareAdvanceTime;						// how far in advance we need to prepare moves, which depends on input shaping
};

#endif /* SRC_MOVEMENT_AXISSHAPER_H_ */
