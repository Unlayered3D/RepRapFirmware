/*
 * FiveAxisKinematics.cpp
 *
 *  Created on: May 1 2025
 *      Author: Alex Stedman
 */

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GCodes/GCodes.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Movement/DDA.h>
#include <Movement/Kinematics/FiveAxisKinematics.h>
#include <Movement/Move.h>

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.
// DH PARAMETERS
// d: [0 d2 d3 d4 0 d6];
// theta: [t1 0 3*pi/2 (pi/2 + o3) t5 0];
// a: [0 0 0 0 a5 0];
// alp: [pi/2 pi/2 pi/2 3*pi/2 pi/2 0];

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...)					OBJECT_MODEL_FUNC_BODY(FiveAxisKinematics, __VA_ARGS__)
#define OBJECT_MODEL_ARRAY_COUNT(_value)		OBJECT_MODEL_ARRAY_COUNT_BODY(FiveAxisKinematics, _value)
#define OBJECT_MODEL_ARRAY_VALUE(...)			OBJECT_MODEL_ARRAY_VALUE_BODY(FiveAxisKinematics, __VA_ARGS__)

constexpr ObjectModelArrayTableEntry FiveAxisKinematics::objectModelArrayTable[] =
{
		// 20. Forward matrix elements in a row
		{
				nullptr,					// no lock needed
				OBJECT_MODEL_ARRAY_COUNT_NOSELF(
						reprap.GetGCodes().GetTotalAxes()),
						OBJECT_MODEL_ARRAY_VALUE(
								self->forwardMatrix(context.GetIndex(1),
										context.GetLastIndex()), 3) },
										// 21. Inverse matrix elements in a row
										{
												nullptr,					// no lock needed
												OBJECT_MODEL_ARRAY_COUNT_NOSELF(
														reprap.GetGCodes().GetVisibleAxes()),
														OBJECT_MODEL_ARRAY_VALUE(
																self->inverseMatrix(context.GetIndex(1),
																		context.GetLastIndex()), 3) },
																		// 22. Forward matrix rows
																		{
																				nullptr,					// no lock needed
																				OBJECT_MODEL_ARRAY_COUNT_NOSELF(
																						reprap.GetGCodes().GetVisibleAxes()),
																						OBJECT_MODEL_ARRAY_VALUE(self,
																								20 | (context.GetLastIndex() << 8), true) },
																								// 23. Inverse matrix rows
																								{
																										nullptr,					// no lock needed
																										OBJECT_MODEL_ARRAY_COUNT_NOSELF(
																												reprap.GetGCodes().GetTotalAxes()),
																												OBJECT_MODEL_ARRAY_VALUE(self,
																														21 | (context.GetLastIndex() << 8), true) } };

DEFINE_GET_OBJECT_MODEL_ARRAY_TABLE_WITH_PARENT(FiveAxisKinematics,
		ZLeadscrewKinematics, 20)

constexpr ObjectModelTableEntry FiveAxisKinematics::objectModelTable[] = {
		// Within each group, these entries must be in alphabetical order
		// 0. kinematics members
		{ "forwardMatrix", OBJECT_MODEL_FUNC_ARRAY(22),
				ObjectModelEntryFlags::none }, { "inverseMatrix",
						OBJECT_MODEL_FUNC_ARRAY(23), ObjectModelEntryFlags::none }, {
								"name", OBJECT_MODEL_FUNC(self->GetName(true)),
								ObjectModelEntryFlags::none }, };

constexpr uint8_t FiveAxisKinematics::objectModelTableDescriptor[] = { 1, 3 };

DEFINE_GET_OBJECT_MODEL_TABLE_WITH_PARENT(FiveAxisKinematics,
		ZLeadscrewKinematics)

// Recalculate internal variables following a configuration change
void FiveAxisKinematics::Recalc() noexcept {
	// Calculate the forward differential matrix by inverting the inverse differential matrix
	{
		// Set up a double-width matrix with the inverse matrix in the left half and a unit diagonal matrix in the right half
		FixedMatrix<float, MaxAxes, 2 * MaxAxes> tempMatrix;
		for (size_t i = 0; i < MaxAxes; ++i) {
			for (size_t j = 0; j < MaxAxes; ++j) {
				tempMatrix(i, j) = inverseMatrix(i, j);
			}
			for (size_t j = MaxAxes; j < 2 * MaxAxes; ++j) {
				tempMatrix(i, j) = 0.0;
			}
			tempMatrix(i, i + MaxAxes) = 1.0;
		}

		// Apply the Gauss-Jordan operation to transform the right half into the inverse of the inverse matrix
		//TODO: PLZ USE A PIVOTING STRATEGY (Partial pivoting) to reduce float error
		const bool ok = tempMatrix.GaussJordan(MaxAxes, 2 * MaxAxes);
		if (ok) {
			// Copy the right half to the forward matrix
			for (size_t i = 0; i < MaxAxes; ++i) {
				for (size_t j = 0; j < MaxAxes; ++j) {
					forwardMatrix(i, j) = tempMatrix(i, j + MaxAxes);
				}
			}
		} else {
			forwardMatrix.Fill(0.0);
			reprap.GetPlatform().Message(ErrorMessage,
					"Invalid kinematics matrix\n");
		}




	}

	// Calculate the first and last motors for each axis, and first and last axis controlled by each motor.
	// These are used to optimise calculations and homing behaviour.
	// It doesn't matter if an axis doesn't actually use all the motors from its first to its last inclusive.
	// Also determine which motors are shared by two or more axes.
	for (size_t i = 0; i < MaxAxes; ++i) {
		firstMotor[i] = firstAxis[i] = MaxAxes;
		lastMotor[i] = lastAxis[i] = 0;
		controllingDrivers[i].Clear();
	}

	for (size_t axis = 0; axis < MaxAxes; ++axis) {
		for (size_t motor = 0; motor < MaxAxes; ++motor) {
			if (inverseMatrix(axis, motor) != 0.0)// if this axis needs this motor driven
			{
				if (axis < firstAxis[motor]) {
					firstAxis[motor] = axis;
				}
				if (axis > lastAxis[motor]) {
					lastAxis[motor] = axis;
				}
				controllingDrivers[axis].SetBit(motor);
			}

			if (forwardMatrix(motor, axis) != 0.0)// if this motor affects this axes
			{
				if (motor < firstMotor[axis]) {
					firstMotor[axis] = motor;
				}
				if (motor > lastMotor[axis]) {
					lastMotor[axis] = motor;
				}
				controllingDrivers[axis].SetBit(motor);
			}
		}
	}

	if (reprap.Debug(Module::Kinematics)) {
		PrintMatrix("Inverse", inverseMatrix);
		PrintMatrix("Forward", forwardMatrix);

		String<MediumStringLength> s;
		s.copy("First/last motors:");
		for (size_t axis = 0; axis < MaxAxes; ++axis) {
			s.catf(" %u/%u", firstMotor[axis], lastMotor[axis]);
		}
		debugPrintf("%s\n", s.c_str());

		s.copy("First/last axes:");
		for (size_t motor = 0; motor < MaxAxes; ++motor) {
			s.catf(" %u/%u", firstAxis[motor], lastAxis[motor]);
		}
		debugPrintf("%s\n", s.c_str());
	}
}

// Return true if the axis doesn't have a single dedicated motor
inline bool FiveAxisKinematics::HasSharedMotor(size_t axis) const noexcept {
	return controllingDrivers[axis] != LogicalDrivesBitmap::MakeFromBits(axis);
}
const static uint8_t COS_ID = 2, SIN_ID = 3, N_COS_ID = 4, N_SIN_ID = 5;
FiveAxisKinematics::FiveAxisKinematics(KinematicsType k) noexcept :
										ZLeadscrewKinematics(k), a5(2.5f), d6(46.4f), modified(false) {

	// Start by assuming 1:1 mapping of axes to motors by setting diagonal elements to 1 and other elements to zero
	inverseMatrix.Fill(0.0);
	rotationMatrix1.Fill(0);
	rotationMatrix2.Fill(0);
	for (size_t i = 0; i < MaxAxes; ++i) {
		inverseMatrix(i, i) = 1.0;
		rotationMatrix1(i,i) = 1;
	}

	switch (k) {
	case KinematicsType::cartesian:
	default:
		break;
		//TODO decide which one of these to use
	case KinematicsType::coreXBYC:
		//DIFFERENTIAL MATRIX
			//This matrix maps motor drivers to each axis
			/*   0    1     2  3   4
			 * x 1    0     0  1   0
			 * y 0    1     0  0   1
			 * z 0    0     1  0   0
			 * b -1/3 0     0  1/3 0
			 * c 0    11/18 0  0   -11/18
			 */

			inverseMatrix(0, 3) = 1.0;
			inverseMatrix(1, 4) = 1.0;

			inverseMatrix(3, 0) = 1.0 / 3.0;
			inverseMatrix(3, 3) = -1.0 / 3.0;
			inverseMatrix(4, 1) = 22.0 / 36.0;
			inverseMatrix(4, 4) = -22.0 / 36.0;

			//
			// This matrix deals with rotations about the C axis by rotating X and Y (d2 & d4)
			// Its technically a householder reflection
			//

			/*
			 * 	x  y  z  b  c
			 *  c  s  0  0  0
			 *  s -c  0  0  0
			 *  0  0  1  0  0
			 *  0  0  0  1  0
			 *  0  0  0  0  1
			 */
			rotationMatrix1(0,0) = COS_ID;
			rotationMatrix1(0,1) = SIN_ID;
			rotationMatrix1(1,0) = SIN_ID;
			rotationMatrix1(1,1) = N_COS_ID;


			//
			// This matrix deals with rotations about the B axis by rotating X and Z (d2 & d4)
			// Its technically a householder reflection
			//

			/* stedmans matrix also now applies a cross axis skew to Y based on new constant
			 *  a5 sd d6
			 *  c   0   s
			 *  0   0   0
			 *  s   0  -c
			 *  0   0   0
			 *  0   0   0
			 */
			rotationMatrix2(0,0) = SIN_ID;
			rotationMatrix2(0,2) = COS_ID;
			//rotationMatrix2(1,1) = SIN_ID;
			rotationMatrix2(2,0) = N_COS_ID;
			rotationMatrix2(2,2) = SIN_ID;
		break;
		//for the new kinematics
	case KinematicsType::coreXBYC2:
	case KinematicsType::coreXBYC3:
		//DIFFERENTIAL MATRIX
		//This matrix maps motor drivers to each axis
		/*   0    1     2  3   4
		 * x 1    0     0 -1   0
		 * y 0   -1     0  0   1
		 * z 0    0     1  0   0
		 * b -1/3 0     0  1/3 0
		 * c 0   -11/18 0  0   -11/18
		 */

		inverseMatrix(0, 3) = -1.0;
		inverseMatrix(1, 1) = -1.0;
		inverseMatrix(1, 4) = 1.0;

		inverseMatrix(3, 0) = -1.0 / 3.0;
		inverseMatrix(3, 3) = -1.0 / 3.0;
		inverseMatrix(4, 1) = -22.0 / 36.0;
		inverseMatrix(4, 4) = -22.0 / 36.0;

		//
		// This matrix deals with rotations about the C axis by rotating X and Y (d2 & d4)
		// Its technically a householder reflection
		//

		/*
		 * 	x  y  z  b  c
		 *  c  s  0  0  0
		 *  s  c  0  0  0
		 *  0  0  1  0  0
		 *  0  0  0  1  0
		 *  0  0  0  0  1
		 */
		rotationMatrix1(0,0) = COS_ID;
		rotationMatrix1(0,1) = SIN_ID;
		rotationMatrix1(1,0) = N_SIN_ID;
		rotationMatrix1(1,1) = COS_ID;


		//
		// This matrix deals with rotations about the B axis by rotating X and Z (d2 & d4)
		//
		//

		/* stedmans matrix also now applies a cross axis skew to Y based on new constant
		 *   a5 sd d6
		 * x -s   0   c
		 * y  0   0   0
		 * z -c   0  -s
		 * b  0   0   0
		 * c  0   0   0
		 */
		rotationMatrix2(0,0) = N_SIN_ID;
		rotationMatrix2(0,2) = COS_ID;
		//rotationMatrix2(1,1) = SIN_ID;
		rotationMatrix2(2,0) = N_COS_ID;
		rotationMatrix2(2,2) = N_SIN_ID;

		break;
	}

	Recalc();
}

// Return the name of the current kinematics
const char* _ecv_array FiveAxisKinematics::GetName(
		bool forStatusReport) const noexcept {
	// This reports the original kinematics that was requested. It doesn't allow for the matrix having been patched to change the kinematics.
	switch (GetKinematicsType()) {
	case KinematicsType::cartesian:
		return (forStatusReport) ? "cartesian" : "Cartesian";

	case KinematicsType::coreXBYC:
		return (forStatusReport) ? "coreXBYC" : "CoreXBYC";

	case KinematicsType::coreXBYC2:
		return (forStatusReport) ? "coreXBYC2" : "CoreXBYC2";

	case KinematicsType::coreXBYC3:
		return (forStatusReport) ? "coreXBYC3" : "CoreXBYC3";
	default:
		return "unknown";
	}
}

// Set the parameters from a M665, M666, M667 or M669 command
// Return true if we changed any parameters. Set 'error' true if there was an error, otherwise leave it alone.
// This function is used for CoreXY and CoreXZ kinematics, but it overridden for CoreXYU kinematics
bool FiveAxisKinematics::Configure(unsigned int mCode, GCodeBuffer &gb,
		const StringRef &reply, bool &error) THROWS(GCodeException)
										{
	if (mCode != 669) {
		return ZLeadscrewKinematics::Configure(mCode, gb, reply, error);
	}

	bool seen = gb.Seen('K');

	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	for (size_t axis = 0; axis < numVisibleAxes; ++axis) {
		if (gb.Seen(reprap.GetGCodes().GetAxisLetters()[axis])) {
			seen = true;
			float motorFactors[MaxAxes];
			size_t numMotors = reprap.GetGCodes().GetTotalAxes();
			gb.GetFloatArray(motorFactors, numMotors, false);
			for (size_t m = 0; m < numMotors; ++m) {
				if (inverseMatrix(axis, m) != motorFactors[m]) {
					inverseMatrix(axis, m) = motorFactors[m];
					modified = true;
				}
			}
			for (size_t m = numMotors; m < MaxAxes; ++m) {
				if (inverseMatrix(axis, m) != 0.0) {
					inverseMatrix(axis, m) = 0.0;
					modified = true;
				}
			}
		}
	}

	const bool seenSeg = TryConfigureSegmentation(gb);// configure optional segmentation
	gb.TryGetFValue('A', a5, seen);
	gb.TryGetFValue('D', d6, seen);
	//gb.TryGetFValue('S', s6, seen);
	reply.printf("A is now %.2f, D is now %.2f", (double)a5, (double)d6);

	if (seen) {
		Recalc();
	} else if (!seenSeg) {
		Kinematics::Configure(mCode, gb, reply, error);
		reply.catf(", %smatrix:", ((modified) ? "modified " : ""));
		const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
		const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();
		for (size_t axis = 0; axis < numVisibleAxes; ++axis) {
			for (size_t motor = 0; motor < numTotalAxes; ++motor) {
				reply.cat((motor == 0) ? '\n' : ' ');
				const float val = inverseMatrix(axis, motor);
				if (val == 0.0) {
					reply.cat('0');	// don't print unnecessary decimals, we will probably reach the response buffer limit if we do
				} else {
					reply.catf("%.2f", (double) val);
				}
			}
		}
	}

	return seen;
										}

float FiveAxisKinematics::getRotationMatrixValue(uint8_t num, float cosN, float sinN)  const noexcept {
	switch(num){
	case 1:
		return 1.0;
	case COS_ID:
		return cosN;
	case SIN_ID:
		return sinN;
	case N_COS_ID:
		return -cosN;
	case N_SIN_ID:
		return -sinN;
	default:
		return 0.0;
	}
}
//-----------------------------------------------------------------------------------------
//INVERSE KINEMATICS
//-----------------------------------------------------------------------------------------
// Convert Cartesian coordinates to motor coordinates returning true if successful.
// This is called frequently, so try to keep it efficient.
// If a motor has no visible axes that affect it, leave the old motor coordinate unchanged.
float pastC = 0.0f;
MovementError FiveAxisKinematics::CartesianToMotorSteps(
		const float machinePos[], const float stepsPerMm[],
		size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[],
		bool isCoordinated) const noexcept {
	MovementError rslt = MovementError::ok;
	//TODO apply inverse kinematics

	//initialize the machine pos
	float rotatedMachinePos[] = {0.0, 0.0, 0.0, 0.0, 0.0};

	//get the factors of the current B and C axes. T5 is the B, T1 is the C
	float cosT5 = cos(M_PI/180.0*machinePos[3]);
	float sinT5 = sin(M_PI/180.0*machinePos[3]);
	float cosT1 = cos(M_PI/180.0*machinePos[4]);
	float sinT1 = sin(M_PI/180.0*machinePos[4]);

	//iterate over the axes to calculate real values
	for(size_t i = 0; i < numTotalAxes; ++i){

		for(size_t j = 0; j < numTotalAxes; ++j){

			//run the bed rotation logic in this case. This translation is only applied to x and y in our case.
			rotatedMachinePos[i] += getRotationMatrixValue(rotationMatrix1(i,j), cosT1, sinT1)*machinePos[j];

		}

		//now we offset the X and Z based on the angle of the nozzle. It is also noted that the a5 and d6 offsets are subtracted out with the cos-1.
		//This is important because otherwise the printer will not home properly. It is more efficient to do it this way rather than ...*a5 - a5

		rotatedMachinePos[i] += getRotationMatrixValue(rotationMatrix2(i,0), cosT5-1, sinT5)*a5;
		//rotatedMachinePos[i] += getRotationMatrixValue(rotationMatrix2(i,1), cosT5-1, sinT5)*s6;
		rotatedMachinePos[i] += getRotationMatrixValue(rotationMatrix2(i,2), cosT5-1, sinT5)*d6;

//
	}




	for (size_t motor = 0; motor < numTotalAxes; ++motor) {
		const size_t axisLimit = min<size_t>(numVisibleAxes,
				lastAxis[motor] + 1);
		size_t axis = firstAxis[motor];
		if (axis < axisLimit) {
			//we multiply the new rotated machine pos to the inverse matrix to get the differential.
			float movement = inverseMatrix(axis, motor) * rotatedMachinePos[axis];
			++axis;
			while (axis < axisLimit) {
				movement += inverseMatrix(axis, motor) * rotatedMachinePos[axis];
				++axis;
			}
			RoundToInt32(rslt, movement * stepsPerMm[motor], motorPos[motor]);
		}
	}
	return rslt;
}

//-----------------------------------------------------------------------------------------
//FORWARDS KINEMATICS
//-----------------------------------------------------------------------------------------
// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
void FiveAxisKinematics::MotorStepsToCartesian(const int32_t motorPos[],
		const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes,
		float machinePos[]) const noexcept {
	float rotatedPosition[] = {0.0, 0.0, 0.0, 0.0, 0.0};

	// If there are more motors than visible axes (e.g. CoreXYU which has a V motor), we assume that we can ignore the trailing ones when calculating the machine position
	for (size_t axis = 0; axis < numTotalAxes; ++axis) {
		const size_t motorLimit = min<size_t>(numTotalAxes,
				lastMotor[axis] + 1);
		for (size_t motor = firstMotor[axis]; motor < motorLimit; ++motor) {
			//we know that this only gets us the value before the appropriate rotations are applied
			const float factor = forwardMatrix(motor, axis);
			if (factor != 0.0) {
				rotatedPosition[axis] += factor * (float) motorPos[motor]
																   / stepsPerMm[motor];
			}
		}
	}

	//we now have our rotated position, but we cant get ahead of ourelves and set the machine pos. We have to undo the rotations. :(
	//first thing is to figure out what the rotations actually are. The rotations "rotated" doesnt mean anything...
	//get the factors of the current B and C axes. T5 is the B, T1 is the C
	float cosT5 = cos(M_PI/180.0*rotatedPosition[3]);
	float sinT5 = sin(M_PI/180.0*rotatedPosition[3]);
	float cosT1 = cos(M_PI/180.0*rotatedPosition[4]);
	float sinT1 = sin(M_PI/180.0*rotatedPosition[4]);

	//Rotation matricies are orthagonal. R^-1 = R^T
	//this is great since our matrices are not square lol :skull:
	//iterate over the positions to calculate real values
	for(size_t i = 0; i < numTotalAxes; ++i){

		//now we offset the X and Z based on the angle of the nozzle. It is also noted that the a5 and d6 offsets are subtracted out with the cos-1.
		//This is important because otherwise the printer will not home properly. It is more efficient to do it this way rather than ...*a5 - a5
		//note the negative sign. This does not need an inverse since this is calculated off of rotations and simply applies a cartesian offset
		machinePos[i] = -getRotationMatrixValue(rotationMatrix2(i,0), cosT5-1, sinT5)*a5;
		//machinePos[i] -= getRotationMatrixValue(rotationMatrix2(i,1), cosT5-1, sinT5)*s6;
		machinePos[i] -= getRotationMatrixValue(rotationMatrix2(i,2), cosT5-1, sinT5)*d6;


		for(size_t j = 0; j < numTotalAxes; ++j){

			//notice how j and i are flipped here. This is the same as the transpose
			machinePos[i] += getRotationMatrixValue(rotationMatrix1(j,i), cosT1, sinT1)*rotatedPosition[j];

		}



	}

	//ok i have no idea if this will actually work.

}

// Limit the speed and acceleration of a move to values that the mechanics can handle
// The speeds along individual Cartesian axes have already been limited before this is called, so we need only be concerned with shared motors
void FiveAxisKinematics::LimitSpeedAndAcceleration(DDA &dda,
		const float *_ecv_array normalisedDirectionVector,
		size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept {
	// For each shared motor, calculate how much of the total move it contributes
	float motorMovements[MaxAxes];
	for (float &mm : motorMovements) {
		mm = 0.0;
	}

	for (size_t axis = 0; axis < numVisibleAxes; ++axis) {
		if (HasSharedMotor(axis)) {
			const float dv = normalisedDirectionVector[axis];
			if (dv != 0.0) {
				for (size_t motor = 0; motor < MaxAxes; ++motor) {
					const float factor = inverseMatrix(axis, motor);
					if (factor != 0.0) {
						motorMovements[motor] += factor * dv;
					}
				}
			}
		}
	}

	for (size_t motor = 0; motor < MaxAxes; ++motor) {
		const float mm = fabsf(motorMovements[motor]);
		if (mm != 0.0) {
			dda.LimitSpeedAndAcceleration(
					reprap.GetMove().MaxFeedrate(motor) / mm,
					reprap.GetMove().NormalAcceleration(motor) / mm);
		}
	}
}

// Return a bitmap of the motors that are involved in homing a particular axis or tower. Used for implementing stall detection endstops.
// Usually it is just the corresponding motor (hence this default implementation), but CoreXY and similar kinematics move multiple motors to home an individual axis.
LogicalDrivesBitmap FiveAxisKinematics::GetControllingDrives(size_t axis,
		bool forHoming) const noexcept {
	return (axis < MaxAxes) ?
			controllingDrivers[axis] : LogicalDrivesBitmap::MakeFromBits(axis);
}

// Convert axis movement or speed amounts to logical drive amounts. Only relevant if GetHomingMode() == HomingMode::homeCartesianAxes.
void FiveAxisKinematics::ConvertAxisAmountsToLogicalDriveAmounts(
		float amounts[MaxAxes], size_t numVisibleAxes,
		size_t numTotalAxes) const noexcept {
	float convertedAmounts[MaxAxes];
	for (size_t motor = 0; motor < numTotalAxes; ++motor) {
		const size_t axisLimit = min<size_t>(numVisibleAxes,
				lastAxis[motor] + 1);
		size_t axis = firstAxis[motor];
		if (axis < axisLimit) {
			float total = inverseMatrix(axis, motor) * amounts[axis];
			++axis;
			while (axis < axisLimit) {
				total += inverseMatrix(axis, motor) * amounts[axis];
				++axis;
			}
			convertedAmounts[motor] = total;
		} else {
			convertedAmounts[motor] = 0.0;
		}
	}
	memcpyf(amounts, convertedAmounts, MaxAxes);
}

// End
