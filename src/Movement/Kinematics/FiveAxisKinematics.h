/*
 * FiveAxisKinematics.h
 *
 *  Created on: May 1 2025
 *      Author: Alex Stedman
 */

#ifndef SRC_MOVEMENT_KINEMATICS_FIVEAXISKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_FIVEAXISKINEMATICS_H_

#include <RepRapFirmware.h>

#if SUPPORT_FIVEAXIS

#include "ZLeadscrewKinematics.h"
#include <Math/Matrix.h>

class FiveAxisKinematics: public ZLeadscrewKinematics {
public:
	static Kinematics *_ecv_from _ecv_null Create(const char *_ecv_array _ecv_null name, int legacyNumber) noexcept;

public:
	FiveAxisKinematics(KinematicsType k) noexcept;

	// Overridden base class functions. See Kinematics.h for descriptions.
	bool Configure(unsigned int mCode, GCodeBuffer &gb, const StringRef &reply,
			bool &error) THROWS(GCodeException) override;
	MovementError CartesianToMotorSteps(const float machinePos[],
			const float stepsPerMm[], size_t numVisibleAxes,
			size_t numTotalAxes, int32_t motorPos[],
			bool isCoordinated) const noexcept override;
	void MotorStepsToCartesian(const int32_t motorPos[],
			const float stepsPerMm[], size_t numVisibleAxes,
			size_t numTotalAxes, float machinePos[]) const noexcept override;
	HomingMode GetHomingMode() const noexcept override { return HomingMode::homeCartesianAxes; }
	void LimitSpeedAndAcceleration(DDA &dda,
			const float *_ecv_array normalisedDirectionVector,
			size_t numVisibleAxes,
			bool continuousRotationShortcut) const noexcept override;
	AxesBitmap GetShortestPathRotaryAxes() const noexcept override;
	LogicalDrivesBitmap GetControllingDrives(size_t axis,
			bool forHoming) const noexcept override;
	void ConvertAxisAmountsToLogicalDriveAmounts(float amounts[MaxAxes],
			size_t numVisibleAxes, size_t numTotalAxes) const noexcept override;
	float GetDegreesPerSegment() const noexcept override { return degreesPerSegment; }

protected:DECLARE_OBJECT_MODEL_WITH_ARRAYS

private:
	void Recalc() noexcept;	// recalculate internal variables following a configuration change
	bool HasSharedMotor(size_t axis) const noexcept;// return true if the axis doesn't have a single dedicated motor

	float getRotationMatrixValue(uint8_t num, float cosN, float sinN) const noexcept ;

	// Degrees added to the commanded B before it goes into the arm trig. K15/K16 measure B from the
	// horizontal (B-90 = nozzle straight down). K17 measures it as tilt from vertical, so B0 = straight
	// down and B90 = straight out, which is B-90 and B0 in the K16 frame. Only the trig sees the shift:
	// the linear B->motor mapping is unchanged, and because the arm offsets stay referenced to the same
	// physical pose, XYZ mean exactly what they mean under K16 - only B values and B limits move by 90.
	float BTrigOffset() const noexcept { return (GetLegacyType() == KinematicsType::coreXBYCTilt) ? -90.0f : 0.0f; }
	// Primary parameters
	FixedMatrix<float, MaxAxes, MaxAxes> inverseMatrix;	// maps coordinates to motor positions

	FixedMatrix<uint8_t, MaxAxes, MaxAxes> rotationMatrix1;
	FixedMatrix<uint8_t, MaxAxes, MaxAxes> rotationMatrix2;


	// Derived parameters
	FixedMatrix<float, MaxAxes, MaxAxes> forwardMatrix;	// maps motor positions to coordinates
	LogicalDrivesBitmap controllingDrivers[MaxAxes];// which drives control each axis


	uint8_t firstMotor[MaxAxes], lastMotor[MaxAxes];// first and last motor used by each axis
	uint8_t firstAxis[MaxAxes], lastAxis[MaxAxes];// first and last axis that each motor controls

	bool matrixNeedsInverting;				// true if inverseMatrix has been altered and forwardMatrix must be recomputed

	// Configurable geometry. Every one of these is set by M669; the letter shown is the
	// parameter that sets it (see the TryGetFValue calls in Configure). a5 and d6 are the two
	// nonzero Denavit-Hartenberg parameters; see the DH table at the top of the .cpp.
	// The skews are cross-axis compensation terms written into inverseMatrix by Recalc, so the
	// comment for each names the coupling it introduces rather than a physical dimension.
	float a5;								// (A) DH link length a5, mm
	float d6;								// (D) DH link offset d6, mm
	float bRatio;							// (R) B-axis reduction: 4*bRatio/36 per motor step pair
	float cRatio;							// (Q) C-axis reduction: 4*cRatio/36 per motor step pair
	float xSkew;							// (X) per mm the X axis travels, change Z by this amount
	float ySkew;							// (Y) per mm the Y axis travels, change Z by this amount
	float xzSkew;							// (U) couples the differential X/B pair into Z
	float yzSkew;							// (V) couples the differential Y/C pair into Z
	float xySkew;							// (W) couples the differential Y/C pair into X
	float bSkew;							// (B) B-rotation-dependent correction applied via rotationMatrix2
	float degreesPerSegment;				// (P) max angular change per segment, deg; 0 disables rotary-driven segmentation
};

#endif	// SUPPORT_FIVEAXIS

#endif /* SRC_MOVEMENT_KINEMATICS_FIVEAXISKINEMATICS_H_ */
