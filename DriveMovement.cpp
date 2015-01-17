/*
 * DriveMovement.cpp
 *
 *  Created on: 17 Jan 2015
 *      Author: David
 */

#include "RepRapFirmware.h"

// Prepare this DM for a Cartesian axis move
void DriveMovement::PrepareCartesianAxis(const DDA& dda, const PrepParams& params)
{
	// Acceleration phase parameters
	accelStopStep = (uint32_t)(dda.accelDistance * stepsPerMm) + 1;
	startSpeedTimesCdivA = params.startSpeedTimesCdivA;
	startSpeedTimesCdivAsquared = (uint64_t)startSpeedTimesCdivA * startSpeedTimesCdivA;

	// Constant speed phase parameters
	mmPerStepTimesCdivtopSpeed = (uint32_t)(((float)DDA::stepClockRate * mmPerStepFactor)/(stepsPerMm * dda.topSpeed));
	accelClocksMinusAccelDistanceTimesCdivTopSpeed = params.accelClocksMinusAccelDistanceTimesCdivTopSpeed;

	// Deceleration phase parameters
	// First check whether there is any deceleration at all, otherwise we may get strange results because of rounding errors
	if (dda.decelDistance * stepsPerMm < 0.5)
	{
		decelStartStep = totalSteps + 1;
		topSpeedTimesCdivAPlusDecelStartClocks = 0;
		twoDistanceToStopTimesCsquaredDivA = 0;
	}
	else
	{
		decelStartStep = (uint32_t)(params.decelStartDistance * stepsPerMm) + 1;
		topSpeedTimesCdivAPlusDecelStartClocks = params.topSpeedTimesCdivAPlusDecelStartClocks;
		const uint64_t initialDecelSpeedTimesCdivASquared = (uint64_t)params.topSpeedTimesCdivA * params.topSpeedTimesCdivA;
		twoDistanceToStopTimesCsquaredDivA = initialDecelSpeedTimesCdivASquared + (uint64_t)((params.decelStartDistance * (DDA::stepClockRateSquared * 2))/dda.acceleration);
	}

	// No reverse phase
	reverseStartStep = totalSteps + 1;
	fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
}

// Prepare this DM for a Delta axis move
void DriveMovement::PrepareDeltaAxis(const DDA& dda, const PrepParams& params)
{
	PrepareCartesianAxis(dda, params);			//TODO write this!
}

// Prepare this DM for an extruder move
void DriveMovement::PrepareExtruder(const DDA& dda, const PrepParams& params)
{
	// Calculate the elasticity compensation parameter (not needed for axis movements, but we do them anyway to keep the code simple)
	uint32_t compensationClocks = (uint32_t)(compensationTime * DDA::stepClockRate);
	float accelCompensationDistance = compensationTime * (dda.topSpeed - dda.startSpeed);
	float accelCompensationSteps = accelCompensationDistance * stepsPerMm;

	// Calculate the net total step count to allow for compensation (may be negative)
	// Note that we add totalSteps in floating point mode, to round the number of steps down consistently
	int32_t netSteps = (int32_t)(((dda.endSpeed - dda.startSpeed) * compensationTime * stepsPerMm) + totalSteps);

	// Acceleration phase parameters
	accelStopStep = (uint32_t)((dda.accelDistance * stepsPerMm) + accelCompensationSteps) + 1;
	startSpeedTimesCdivA = params.startSpeedTimesCdivA + compensationClocks;
	startSpeedTimesCdivAsquared = (uint64_t)startSpeedTimesCdivA * startSpeedTimesCdivA;

	// Constant speed phase parameters
	mmPerStepTimesCdivtopSpeed = (uint32_t)(((float)DDA::stepClockRate * mmPerStepFactor)/(stepsPerMm * dda.topSpeed));
	accelClocksMinusAccelDistanceTimesCdivTopSpeed = (int32_t)params.accelClocksMinusAccelDistanceTimesCdivTopSpeed - (int32_t)(compensationClocks * params.compFactor);

	// Deceleration and reverse phase parameters
	// First check whether there is any deceleration at all, otherwise we may get strange results because of rounding errors
	if (dda.decelDistance * stepsPerMm < 0.5)
	{
		totalSteps = netSteps;
		decelStartStep = reverseStartStep = netSteps + 1;
		topSpeedTimesCdivAPlusDecelStartClocks = 0;
		fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
		twoDistanceToStopTimesCsquaredDivA = 0;
	}
	else
	{
		decelStartStep = (uint32_t)((params.decelStartDistance * stepsPerMm) + accelCompensationSteps) + 1;
		const int32_t initialDecelSpeedTimesCdivA = (int32_t)params.topSpeedTimesCdivA - (int32_t)compensationClocks;	// signed because it may be negative and we square it
		const uint64_t initialDecelSpeedTimesCdivASquared = (int64_t)initialDecelSpeedTimesCdivA * initialDecelSpeedTimesCdivA;
		topSpeedTimesCdivAPlusDecelStartClocks = params.topSpeedTimesCdivAPlusDecelStartClocks - compensationClocks;
		twoDistanceToStopTimesCsquaredDivA =
			initialDecelSpeedTimesCdivASquared + (uint64_t)(((params.decelStartDistance + accelCompensationDistance) * (DDA::stepClockRateSquared * 2))/dda.acceleration);

		float initialDecelSpeed = dda.topSpeed - dda.acceleration * compensationTime;
		float reverseStartDistance = (initialDecelSpeed > 0.0) ? initialDecelSpeed * initialDecelSpeed/(2 * dda.acceleration) + params.decelStartDistance : params.decelStartDistance;

		// Reverse phase parameters
		if (reverseStartDistance >= dda.totalDistance)
		{
			// No reverse phase
			totalSteps = netSteps;
			reverseStartStep = netSteps + 1;
			fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
		}
		else
		{
			reverseStartStep = (initialDecelSpeed < 0.0)
									? decelStartStep
									: (twoDistanceToStopTimesCsquaredDivA/twoCsquaredTimesMmPerStepDivA) + 1;
			// Because the step numbers are rounded down, we may sometimes get a situation in which netSteps = 1 and reverseStartStep = 1.
			// This would lead to totalSteps = -1, which must be avoided.
			int32_t overallSteps = (int32_t)(2 * (reverseStartStep - 1)) - netSteps;
			if (overallSteps > 0)
			{
				totalSteps = overallSteps;
				fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA =
						(int64_t)((2 * (reverseStartStep - 1)) * twoCsquaredTimesMmPerStepDivA) - (int64_t)twoDistanceToStopTimesCsquaredDivA;
			}
			else
			{
				totalSteps = (uint)max<int32_t>(netSteps, 0);
				reverseStartStep = totalSteps + 1;
				fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
			}
		}
	}
}

// Calculate the time since the start of the move when the next step for the specified DriveMovement is due
uint32_t DriveMovement::CalcNextStepTime(size_t drive)
{
	uint32_t lastStepTime = nextStepTime;
	++nextStep;
	if (nextStep > totalSteps)
	{
		moving = false;
		return NoStepTime;
	}

	if (nextStep < accelStopStep)
	{
		nextStepTime = isqrt(startSpeedTimesCdivAsquared + (twoCsquaredTimesMmPerStepDivA * nextStep)) - startSpeedTimesCdivA;
	}
	else if (nextStep < decelStartStep)
	{
		nextStepTime = (uint32_t)((int32_t)(((uint64_t)mmPerStepTimesCdivtopSpeed * nextStep)/mmPerStepFactor) + accelClocksMinusAccelDistanceTimesCdivTopSpeed);
	}
	else if (nextStep < reverseStartStep)
	{
		nextStepTime = topSpeedTimesCdivAPlusDecelStartClocks
							- isqrt(twoDistanceToStopTimesCsquaredDivA - (twoCsquaredTimesMmPerStepDivA * nextStep));
	}
	else
	{
		if (nextStep == reverseStartStep)
		{
			reprap.GetPlatform()->SetDirection(drive, !direction, false);
		}
		nextStepTime = topSpeedTimesCdivAPlusDecelStartClocks
							+ isqrt((int64_t)(twoCsquaredTimesMmPerStepDivA * nextStep) - fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA);

	}
	//TODO include delta support

	if ((int32_t)nextStepTime < (int32_t)(lastStepTime + 40))
	{
		stepError = true;
		return NoStepTime;
	}
	return nextStepTime;
}

// Reduce the speed of this movement. Called to reduce the homing speed when we detect we are near the endstop for a drive.
void DriveMovement::ReduceSpeed(float newSpeed, bool isNearEndstop)
{
	// Force the linear motion phase
	decelStartStep = totalSteps + 1;
	accelStopStep = 0;

	// Adjust the speed
	mmPerStepTimesCdivtopSpeed = (uint32_t)(((float)DDA::stepClockRate * mmPerStepFactor)/(stepsPerMm * newSpeed));
	if (isNearEndstop)
	{
		accelClocksMinusAccelDistanceTimesCdivTopSpeed = (int32_t)nextStepTime - (int32_t)(((uint64_t)mmPerStepTimesCdivtopSpeed * nextStep)/mmPerStepFactor);
	}
}

void DriveMovement::DebugPrint(char c) const
{
	if (moving || stepError)
	{
		debugPrintf("DM%c%s dv=%f stepsPerMm=%f totSteps=%u accelStopStep=%u decelStartStep=%u revStartStep=%u nextStep=%u nextStepTime=%u"
					" 2CsqtMmPerStepDivA=%" PRIu64
					"\n",
					c, (stepError) ? " ERR:" : ":", dv, stepsPerMm, totalSteps, accelStopStep, decelStartStep, reverseStartStep, nextStep, nextStepTime,
					twoCsquaredTimesMmPerStepDivA
					);
		debugPrintf(" mmPerStepTimesCdivtopSpeed=%u sstcda=%u sstcda2=%" PRIu64 " adtcdtsmac=%d"
					" tstcdapdsc=%u tstdca2=%" PRIu64 " fmsdmtstdca2=%" PRId64
					"\n",
					mmPerStepTimesCdivtopSpeed, startSpeedTimesCdivA, startSpeedTimesCdivAsquared, accelClocksMinusAccelDistanceTimesCdivTopSpeed,
					topSpeedTimesCdivAPlusDecelStartClocks, twoDistanceToStopTimesCsquaredDivA, fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA
					);
	}
	else
	{
		debugPrintf("DM%c: not moving\n", c);
	}
}

// Fast 64-bit integer square root function
/* static */ uint32_t DriveMovement::isqrt(uint64_t num)
{
//irqflags_t flags = cpu_irq_save();
//uint32_t t2 = Platform::GetInterruptClocks();
	uint32_t numHigh = (uint32_t)(num >> 32);
	if (numHigh != 0)
	{
		uint32_t resHigh = 0;

#define iter64a(N) 								\
		{										\
			uint32_t temp = resHigh + (1 << N);	\
			if (numHigh >= temp << N)			\
			{									\
				numHigh -= temp << N;			\
				resHigh |= 2 << N;				\
			}									\
		}

		// We need to do 16 iterations
		iter64a(15); iter64a(14); iter64a(13); iter64a(12);
		iter64a(11); iter64a(10); iter64a(9); iter64a(8);
		iter64a(7); iter64a(6); iter64a(5); iter64a(4);
		iter64a(3); iter64a(2); iter64a(1); iter64a(0);

		// resHigh is twice the square root of the msw, in the range 0..2^17-1
		uint64_t res = (uint64_t)resHigh << 16;
		uint64_t numAll = ((uint64_t)numHigh << 32) | (uint32_t)num;

#define iter64b(N) 								\
		{										\
			uint64_t temp = res | (1 << N);		\
			if (numAll >= temp << N)			\
			{									\
				numAll -= temp << N;			\
				res |= 2 << N;					\
			}									\
		}

		// We need to do 16 iterations
		iter64b(15); iter64b(14); iter64b(13); iter64b(12);
		iter64b(11); iter64b(10); iter64b(9); iter64b(8);
		iter64b(7); iter64b(6); iter64b(5); iter64b(4);
		iter64b(3); iter64b(2); iter64b(1); iter64b(0);

		uint32_t rslt = (uint32_t)(res >> 1);

//uint32_t t3 = Platform::GetInterruptClocks() - t2; if (t3 < minCalcTime) minCalcTime = t3; if (t3 > maxCalcTime) maxCalcTime = t3;
//cpu_irq_restore(flags);
//uint64_t num3 = (uint64_t)rslt * rslt; if (num3 > num || (num - num3) > 2*rslt) {++sqrtErrors; lastNum = num; lastRes = rslt; }
		return rslt;
	}
	else
	{
		// 32-bit square root
		uint32_t num32 = (uint32_t)num;
		uint32_t res32 = 0;

		// Thanks to Wilco Dijksra for this efficient ARM algorithm
#define iter32(N) 								\
		{										\
			uint32_t temp = res32 | (1 << N);	\
			if (num32 >= temp << N)				\
			{									\
				num32 -= temp << N;				\
				res32 |= 2 << N;				\
			}									\
		}

		// We need to do 16 iterations
		iter32(15); iter32(14); iter32(13); iter32(12);
		iter32(11); iter32(10); iter32(9); iter32(8);
		iter32(7); iter32(6); iter32(5); iter32(4);
		iter32(3); iter32(2); iter32(1); iter32(0);

		res32 >>= 1;

//uint32_t t3 = Platform::GetInterruptClocks() - t2; if (t3 < minCalcTime) minCalcTime = t3; if (t3 > maxCalcTime) maxCalcTime = t3;
//cpu_irq_restore(flags);
//uint64_t num3 = (uint64_t)res32 * res32; if (num3 > num || (num - num3) > 2*res32) {++sqrtErrors; lastNum = num; lastRes = res32; }
		return res32;
	}
}

// End
