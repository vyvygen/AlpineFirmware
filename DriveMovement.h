/*
 * DriveMovement.h
 *
 *  Created on: 17 Jan 2015
 *      Author: David
 */

#ifndef DRIVEMOVEMENT_H_
#define DRIVEMOVEMENT_H_

class DDA;

// Struct for passing parameters to the DriveMovement Prepare methods
struct PrepParams
{
	float decelStartDistance;
	uint32_t startSpeedTimesCdivA;
	uint32_t topSpeedTimesCdivA;
	uint32_t decelStartClocks;
	uint32_t topSpeedTimesCdivAPlusDecelStartClocks;
	uint32_t accelClocksMinusAccelDistanceTimesCdivTopSpeed;
	float compFactor;
};

// This class describes a single movement of one drive
class DriveMovement
{
public:
	uint32_t CalcNextStepTime(size_t drive);
	uint32_t CalcNextStepTimeDelta(size_t drive);
	void PrepareCartesianAxis(const DDA& dda, const PrepParams& params);
	void PrepareDeltaAxis(const DDA& dda, const PrepParams& params, size_t drive);
	void PrepareExtruder(const DDA& dda, const PrepParams& params);
	void ReduceSpeed(float newSpeed, bool isNearEndstop);
	void DebugPrint(char c, bool withDelta) const;

	static uint32_t isqrt(uint64_t num);

	// These values don't depend on how the move is executed, so  are set by Init()
	float dv;									// proportion of total movement for this drive
	float stepsPerMm;							// steps per mm of movement in hypercuboid space
	float compensationTime;						// the elasticity compensation time, in seconds
	float aAplusbB;								// for delta calculations
	float dSquaredMinusAsquaredMinusBsquared;	// for delta calculations
	uint64_t twoCsquaredTimesMmPerStepDivA;		// 2 * clock^2 * mmPerStepInHyperCuboidSpace / acceleration
	int32_t h0MinusZ0;							// the starting step position less the starting Z height, needed for delta calculations
	uint32_t totalSteps;						// total number of steps for this move
	bool moving;								// true if this drive moves in this move, if false then all other values are don't cares
	bool direction;								// true=forwards, false=backwards
	bool stepError;								// for debugging

	// These values depend on how the move is executed, so they are set by Prepare()
	uint32_t accelStopStep;						// the first step number at which we are no longer accelerating
	uint32_t decelStartStep;					// the first step number at which we are decelerating
	uint32_t reverseStartStep;					// the first step number for which we need to reverse direction to to elastic compensation

	uint32_t mmPerStepTimesCdivtopSpeed;		// mmPerStepInHyperCuboidSpace * clock / topSpeed

	// The following only need to be stored per-drive if we are supporting elasticity compensation
	uint32_t startSpeedTimesCdivA;
	uint64_t startSpeedTimesCdivAsquared;
	int32_t accelClocksMinusAccelDistanceTimesCdivTopSpeed;					// this one can be negative
	uint32_t topSpeedTimesCdivAPlusDecelStartClocks;
	uint64_t twoDistanceToStopTimesCsquaredDivA;
	int64_t fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA;		// this one can be negative

	// These values change as the step is executed
	uint32_t nextStep;							// number of steps already done
	uint32_t nextStepTime;						// how many clocks after the start of this move the next step is due

	static const uint32_t NoStepTime = 0xFFFFFFFF;		// value to indicate that no further steps are needed when calculating the next step time
	static const uint32_t mmPerStepFactor = 1024;		// a power of 2 used to multiply the value mmPerStepTimesCdivtopSpeed for better accuracy

	// Given an overall speed, return the signed speed of this drive
	float GetDriveSpeed(float speed) const
	{
		if (moving)
		{
			float res = speed * dv;
			return (direction) ? -res : res;
		}
		return 0.0;
	}
};

#endif /* DRIVEMOVEMENT_H_ */
