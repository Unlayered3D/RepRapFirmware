/*
 * FiveAxisKinematics.h
 *
 *  Created on: May 1 2025
 *      Author: Alex Stedman
 */

#ifndef SRC_MOVEMENT_KINEMATICS_SPLIT5KINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_SPLIT5KINEMATICS_H_

#include "ZLeadscrewKinematics.h"
#include <Math/Matrix.h>

class FiveAxisKinematics: public ZLeadscrewKinematics {
public:
	FiveAxisKinematics(KinematicsType k) noexcept;

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char* _ecv_array GetName(bool forStatusReport) const noexcept
			override;
	bool Configure(unsigned int mCode, GCodeBuffer &gb, const StringRef &reply,
			bool &error) THROWS(GCodeException) override;
	MovementError CartesianToMotorSteps(const float machinePos[],
			const float stepsPerMm[], size_t numVisibleAxes,
			size_t numTotalAxes, int32_t motorPos[],
			bool isCoordinated) const noexcept override;
	void MotorStepsToCartesian(const int32_t motorPos[],
			const float stepsPerMm[], size_t numVisibleAxes,
			size_t numTotalAxes, float machinePos[]) const noexcept override;
	HomingMode GetHomingMode() const noexcept override {
		//for mode 3 let it use individual drives, otherwise home normally.
		//TODO pick a mode and stick with it
		if (GetKinematicsType() == KinematicsType::coreXBYC3) {
			return HomingMode::homeIndividualDrives;
		} else {
			return HomingMode::homeCartesianAxes;
		}
	}
	void LimitSpeedAndAcceleration(DDA &dda,
			const float *_ecv_array normalisedDirectionVector,
			size_t numVisibleAxes,
			bool continuousRotationShortcut) const noexcept override;
	LogicalDrivesBitmap GetControllingDrives(size_t axis,
			bool forHoming) const noexcept override;
	void ConvertAxisAmountsToLogicalDriveAmounts(float amounts[MaxAxes],
			size_t numVisibleAxes, size_t numTotalAxes) const noexcept override;

protected:DECLARE_OBJECT_MODEL_WITH_ARRAYS

private:
	void Recalc() noexcept;	// recalculate internal variables following a configuration change
	bool HasSharedMotor(size_t axis) const noexcept;// return true if the axis doesn't have a single dedicated motor

	float getRotationMatrixValue(uint8_t num, float cosN, float sinN) const noexcept ;
	// Primary parameters
	FixedMatrix<float, MaxAxes, MaxAxes> inverseMatrix;	// maps coordinates to motor positions

	FixedMatrix<uint8_t, MaxAxes, MaxAxes> rotationMatrix1;
	FixedMatrix<uint8_t, MaxAxes, MaxAxes> rotationMatrix2;


	// Derived parameters
	FixedMatrix<float, MaxAxes, MaxAxes> forwardMatrix;	// maps motor positions to coordinates
	LogicalDrivesBitmap controllingDrivers[MaxAxes];// which drives control each axis


	uint8_t firstMotor[MaxAxes], lastMotor[MaxAxes];// first and last motor used by each axis
	uint8_t firstAxis[MaxAxes], lastAxis[MaxAxes];// first and last axis that each motor controls

	bool modified;							// true if matrix has been altered
	float a5, d6, s6;
};

#endif /* SRC_MOVEMENT_KINEMATICS_FiveAxisKinematics_H_ */
