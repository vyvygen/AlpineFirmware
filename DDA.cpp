/*
 * DDA.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

//****************************************************************************************************

#include "RepRapFirmware.h"

const uint32_t NoStepTime = 0xFFFFFFFF;		// value to indicate that no further steps are needed when calculating the next step time

const uint32_t mmPerStepFactor = 1024;		// a power of 2 used to multiply the value mmPerStepTimesCdivtopSpeed for better accuracy

void DriveMovement::DebugPrint(char c) const
{
	if (moving)
	{
		debugPrintf("DM%c: dv=%f stepsPerMm=%f totSteps=%u 2CsqtMmPerStepDivA=%" PRIu64 " accelStopStep=%u decelStartStep=%u mmPerStepTimesCdivtopSpeed=%u"
					"\n",
					c, dv, stepsPerMm, totalSteps, twoCsquaredTimesMmPerStepDivA, accelStopStep, decelStartStep, mmPerStepTimesCdivtopSpeed
					);
	}
	else
	{
		debugPrintf("DM%c: not moving\n", c);
	}
}

DDA::DDA(DDA* n) : state(empty), next(n), prev(nullptr)
{
}

void DDA::DebugPrint() const
{
	debugPrintf("DDA: fstep=%u d=%f a=%f reqv=%f"
				" topv=%f startv=%f tstopa=%f tstartd=%f ttotal=%f daccel=%f ddecel=%f",
				firstStepTime, totalDistance, acceleration, requestedSpeed,
				topSpeed, startSpeed, accelStopTime, decelStartTime, totalTime, accelDistance, decelDistance);
	debugPrintf(" sstcda=%u sstcda2=%" PRIu64 " tstcdapdsc=%u tstdca2=%" PRIu64
				" adtcdtsmac=%u"
				"\n",
				startSpeedTimesCdivA, startSpeedTimesCdivAsquared, topSpeedTimesCdivAPlusDecelStartClocks, topSpeedTimesCdivAsquaredPlusTwoDecelStartDistanceTimesCsquareddDivA,
				accelClocksMinusAccelDistanceTimesCdivTopSpeed);
	ddm[0].DebugPrint('x');
	ddm[1].DebugPrint('y');
	ddm[2].DebugPrint('z');
}

// Set up a dummy move at the start of the ring so that Move can find the last move endpoint
void DDA::Init()
{
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		endPoint[drive] = 0;
	}
	state = empty;
}

// Set up a real move. Return true if it represents real movement, alse false.
bool DDA::Init(const float nextMove[], EndstopChecks ce)
{
	// 1. Computer the new endpoints and the movement vector
	bool realMove = false;
	const int32_t *positionNow = prev->MachineCoordinates();
	float normalisedDirectionVector[DRIVES];	// Used to hold a unit-length vector in the direction of motion
	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		int32_t ep = EndPointToMachine(drive, nextMove[drive]);
		endPoint[drive] = ep;
		int32_t delta = (drive < AXES) ? ep - positionNow[drive] : ep;
		normalisedDirectionVector[drive] = (float)delta/reprap.GetPlatform()->DriveStepsPerUnit(drive);
		DriveMovement& dm = ddm[drive];
		dm.moving = (delta != 0);
		if (dm.moving)
		{
			realMove = true;
			dm.totalSteps = labs(delta);
			dm.direction = (delta >= 0);
		}
	}

	// 2. Throw it away if there's no real movement.
	if (!realMove)
	{
		return false;
	}

	// 3. Store some values
	endStopsToCheck = ce;

	// 4. Compute the direction and amount of motion, moved to the positive hyperquadrant
	Move::Absolute(normalisedDirectionVector, DRIVES);
	totalDistance = Move::Normalise(normalisedDirectionVector, DRIVES);

	// 5. Compute the maximum acceleration available
	acceleration = Move::VectorBoxIntersection(normalisedDirectionVector, reprap.GetPlatform()->Accelerations(), DRIVES);

	// Set the speed to the smaller of the requested and maximum speed.
	// Note: this assumes that the requested feedrate is the Cartesian diagonal of all the drive moves (as in the RRP 0.78 release).
	// This will cause the actual feedrate to be slightly lower than requested if an XYZ move includes extrusion, however the effect is only a few percent.
	requestedSpeed = min<float>(nextMove[DRIVES], Move::VectorBoxIntersection(normalisedDirectionVector, reprap.GetPlatform()->MaxFeedrates(), DRIVES));

	for (size_t i = 0; i < DRIVES; ++i)
	{
		DriveMovement& dm = ddm[i];
		if (dm.moving)
		{
			dm.nextStep = 0;
			dm.dv = normalisedDirectionVector[i];
			dm.stepsPerMm = reprap.GetPlatform()->DriveStepsPerUnit(i) * dm.dv;
			dm.twoCsquaredTimesMmPerStepDivA = (uint64_t)(((float)stepClockRate * (float)stepClockRate)/(dm.stepsPerMm * acceleration)) * 2;
			dm.elasticComp = reprap.GetPlatform()->GetElasticComp(i) * dm.dv;
		}
	}

	// 2. Calculate the accelerate and decelerate distances and the top speed
	if (prev->state != provisional)
	{
		// There is no previous move that we can adjust, so this move must start at zero speed, and for now it must also end at zero speed.
		// Calculate the distance required to accelerate to the requested speed: v^2 = u^2 + 2as
		startSpeed = 0.0;
		float distanceToReqSpeed = (requestedSpeed * requestedSpeed)/(2.0 * acceleration);
		if (distanceToReqSpeed * 2 >= totalDistance)
		{
			// we can't reach the requested speed, so just accelerate and decelerate
			accelDistance = decelDistance = totalDistance * 0.5;
			topSpeed = sqrt(accelDistance * acceleration * 2.0);
			//moveType = accelDecel;
		}
		else
		{
			accelDistance = decelDistance = distanceToReqSpeed;
			topSpeed = requestedSpeed;
		}
	}
	else
	{
		// Assuming that this move ends with zero speed, calculate the maximum possible starting speed: u^2 = v^2 - 2as
		float maxStartSpeed = sqrt(acceleration * totalDistance * 2.0);

		// Determine the ideal starting speed, given that the end speed is constrained to zero until we have another move to follow it
		float idealStartSpeed = min<float>(maxStartSpeed, requestedSpeed);

		// Try to meld this move to the previous move to avoid stop/start
		startSpeed = prev->AdjustEndSpeed(idealStartSpeed);
		accelDistance = ((requestedSpeed * requestedSpeed) - (startSpeed * startSpeed))/(2.0 * acceleration);
		decelDistance = (requestedSpeed * requestedSpeed)/(2.0 * acceleration);		// end speed is zero
		if (accelDistance + decelDistance >= totalDistance)
		{
			// It's an accelerate-decelerate move. If V is the peak speed, then (V^2 - u^2)/2a + (V^2 - 0)/2a = distance.
			// So (2V^2 - u^2)/2a = distance
			// So V^2 = a * distance - 0.5u^2
			float vsquared = (acceleration * totalDistance) - 0.5 * startSpeed * startSpeed;
			// Calculate accelerate distance from: V^2 = u^2 + 2as
			accelDistance = (vsquared - (startSpeed * startSpeed))/(2.0 * acceleration);
			decelDistance = totalDistance - accelDistance;
			topSpeed = sqrt(vsquared);
		}
		else
		{
			topSpeed = requestedSpeed;
		}
	}

	state = provisional;
	return true;
}

float DDA::MachineToEndPoint(size_t drive) const
{
	return ((float)(endPoint[drive]))/reprap.GetPlatform()->DriveStepsPerUnit(drive);
}

// Adjust the end speed of this move, if we can.
// Prior to calling this, the caller has checked that its state is 'provisional'.
float DDA::AdjustEndSpeed(float targetStartSpeed)
//pre(state == provisional)
{
//	debugPrintf("Adjusting, %f\n", targetStartSpeed);
	// Decide what speed we would really like to start at. There are several possibilities:
	// 1. If the top speed is already the requested speed, use the requested speed.
	// 2. Else if this is a deceleration-only move and the previous move is not frozen, we may be able to increase the start speed,
	//    so use the requested speed again.
	// 3. Else the start speed must be pinned, so use the lower of the maximum speed we can accelerate to and the requested speed.
	bool canAdjustPreviousMove = (decelDistance == totalDistance && prev->state == provisional);
	float targetEndSpeed =
			(topSpeed == requestedSpeed || canAdjustPreviousMove)
			? requestedSpeed
			: min<float>(sqrt((startSpeed * startSpeed) + (2 * acceleration * totalDistance)), requestedSpeed);

	// We may need 2 goes at this
	bool repeat;
	do
	{
//		debugPrintf(" Repeat\n");
		// We may have to make multiple passes, because reducing one of the speeds may solve some problems but actually make matters worse on another axis.
		bool limited;
		do
		{
//			debugPrintf("  Pass, start=%f end=%f\n", targetStartSpeed, targetEndSpeed);
			limited = false;
			for (size_t drive = 0; drive < DRIVES; ++drive)
			{
				const DriveMovement& thisMoveDm = ddm[drive];
				const DriveMovement& nextMoveDm = next->ddm[drive];
				if (thisMoveDm.moving || nextMoveDm.moving)
				{
					float thisMoveSpeed = thisMoveDm.GetDriveSpeed(targetEndSpeed);
					float nextMoveSpeed = nextMoveDm.GetDriveSpeed(targetStartSpeed);
					float idealDeltaV = fabsf(thisMoveSpeed - nextMoveSpeed);
					float maxDeltaV = reprap.GetPlatform()->InstantDv(drive);
					if (idealDeltaV > maxDeltaV)
					{
						// This drive can't change speed fast enough, so reduce the start and/or end speeds
						// This algorithm sometimes converges very slowly, requiring many passes.
						// To ensure it converges at all, and to speed up convergence, we over-adjust the speed to achieve an even lower deltaV.
						maxDeltaV *= 0.8;
						if (thisMoveDm.direction == nextMoveDm.direction)
						{
							// Drives moving in the same direction, so we must reduce the faster one
							if (fabsf(thisMoveSpeed) > fabsf(nextMoveSpeed))
							{
								targetEndSpeed = (fabsf(nextMoveSpeed) + maxDeltaV)/thisMoveDm.dv;
							}
							else
							{
								targetStartSpeed = (fabsf(thisMoveSpeed) + maxDeltaV)/nextMoveDm.dv;
							}
						}
						else if (fabsf(thisMoveSpeed) * 2 < maxDeltaV)
						{
							targetStartSpeed = (maxDeltaV - fabsf(thisMoveSpeed))/nextMoveDm.dv;
						}
						else if (fabsf(nextMoveSpeed) * 2 < maxDeltaV)
						{
							targetEndSpeed = (maxDeltaV - fabsf(nextMoveSpeed))/thisMoveDm.dv;
						}
						else
						{
							targetStartSpeed = maxDeltaV/(2 * nextMoveDm.dv);
							targetEndSpeed = maxDeltaV/(2 * thisMoveDm.dv);
						}
						limited = true;
						// Most conflicts are between X and Y. So if we just did Y, start another pass immediately to save time.
						if (drive == 1)
						{
							break;
						}
					}
				}
			}
		} while (limited);

		repeat = false;

		// If we made the assumption that we can increase the end speed of the previous move, see whether we need to do so
		if (canAdjustPreviousMove)
		{
			float maxStartSpeed = min<float>(sqrt((targetEndSpeed * targetEndSpeed) + (2 * acceleration * totalDistance)), requestedSpeed);
			debugPrintf("Recursion start\n");
			startSpeed = prev->AdjustEndSpeed(maxStartSpeed);		// TODO eliminate this recursive call! (but it's very rare)
			debugPrintf("Recursion end\n");
			float maxEndSpeed = sqrt((startSpeed * startSpeed) + (2 * acceleration * totalDistance));
			if (maxEndSpeed < targetEndSpeed)
			{
				// Oh dear, we were too optimistic! Have another go.
				targetEndSpeed = maxEndSpeed;
				canAdjustPreviousMove = false;						// we have already adjusted it as much as we can
				repeat = true;
			}
		}
	} while (repeat);

	// Recalculate this move
	accelDistance = ((requestedSpeed * requestedSpeed) - (startSpeed * startSpeed))/(2.0 * acceleration);
	decelDistance = ((requestedSpeed * requestedSpeed) - (targetEndSpeed * targetEndSpeed))/(2.0 * acceleration);
	if (accelDistance + decelDistance >= totalDistance)
	{
		// It's an accelerate-decelerate move. If V is the peak speed, then (V^2 - u^2)/2a + (V^2 - v^2)/2a = distance.
		// So (2V^2 - u^2 - v^2)/2a = distance
		// So V^2 = a * distance - 0.5(u^2 + v^2)
		float vsquared = (acceleration * totalDistance) - 0.5 * ((startSpeed * startSpeed) + (targetEndSpeed * targetEndSpeed));
		// Calculate accelerate distance from: V^2 = u^2 + 2as
		if (vsquared >= 0.0)
		{
			accelDistance = (vsquared - (startSpeed * startSpeed))/(2.0 * acceleration);
			decelDistance = totalDistance - accelDistance;
			topSpeed = sqrt(vsquared);
		}
		else if (startSpeed < targetEndSpeed)
		{
			// This would ideally never happen, but might because of rounding errors
			accelDistance = totalDistance;
			decelDistance = 0.0;
			topSpeed = targetEndSpeed;
		}
		else
		{
			// This would ideally never happen, but might because of rounding errors
			accelDistance = 0.0;
			decelDistance = totalDistance;
			topSpeed = startSpeed;
		}

	}
	else
	{
		topSpeed = requestedSpeed;
	}

//	debugPrintf("Complete, %f\n", targetStartSpeed);
	return targetStartSpeed;
}

// Force an end point
void DDA::SetDriveCoordinate(int32_t a, size_t drive)
{
	endPoint[drive] = a;
}

/*static*/ int32_t DDA::EndPointToMachine(size_t drive, float coord)
{
	return (int32_t)roundf(coord * reprap.GetPlatform()->DriveStepsPerUnit(drive));
}


// The remaining functions are speed-critical, so use full optimisation
#pragma GCC optimize ("O3")

// Prepare this DDA for execution
void DDA::Prepare()
{
	// Now convert the accelerate/decelerate distances to times
	float decelStartDistance = totalDistance - decelDistance;
	accelStopTime = sqrt((2.0 * accelDistance)/acceleration);
	decelStartTime = accelStopTime + (decelStartDistance - accelDistance)/topSpeed;
	totalTime = decelStartTime + sqrt((2.0 * decelDistance)/acceleration);

	startSpeedTimesCdivA = (uint32_t)((startSpeed * (float)stepClockRate)/acceleration);
	startSpeedTimesCdivAsquared = (uint64_t)startSpeedTimesCdivA * startSpeedTimesCdivA;
	uint32_t topSpeedTimesCdivA = (uint32_t)((topSpeed * (float)stepClockRate)/acceleration);
	uint32_t decelStartClocks = decelStartTime * (float)stepClockRate;
	topSpeedTimesCdivAPlusDecelStartClocks = topSpeedTimesCdivA + decelStartClocks;
	topSpeedTimesCdivAsquaredPlusTwoDecelStartDistanceTimesCsquareddDivA =
		((uint64_t)topSpeedTimesCdivA * topSpeedTimesCdivA) + ((uint64_t)(((float)stepClockRate * (float)stepClockRate * decelStartDistance)/acceleration) * 2);
	uint32_t accelClocks = (uint32_t)(accelStopTime * (float)stepClockRate);
	accelClocksMinusAccelDistanceTimesCdivTopSpeed = accelClocks - (uint32_t)((accelDistance * (float)stepClockRate)/topSpeed);
	timeNeeded = (uint32_t)(totalTime * (float)stepClockRate);

	firstStepTime = NoStepTime;
	for (size_t i = 0; i < DRIVES; ++i)
	{
		DriveMovement& dm = ddm[i];
		if (dm.moving)
		{
			dm.accelStopStep = (uint32_t)(accelDistance * dm.stepsPerMm) + 1;
			dm.decelStartStep = (uint32_t)(decelStartDistance * dm.stepsPerMm) + 1;
			dm.mmPerStepTimesCdivtopSpeed = (uint32_t)(((float)stepClockRate * mmPerStepFactor)/(dm.stepsPerMm * topSpeed));
			uint32_t st = CalcNextStepTime(dm);
			if (st < firstStepTime)
			{
				firstStepTime = st;
			}
		}
	}

//	DebugPrint();
	state = frozen;					// must do this last so that the ISR doesn't start executing it before we have finished setting it up
}

// Start executing the move, returning true if Step() needs to be called immediately.
bool DDA::Start(uint32_t tim)
//pre(state == frozen)
{
	moveStartTime = tim;
	moveCompletedTime = 0;
	state = executing;

	if (firstStepTime == NoStepTime)
	{
		// No steps are pending. This should not happen!
		state = completed;
		return false;
	}
	else
	{
		for (size_t i = 0; i < DRIVES; ++i)
		{
			DriveMovement& dm = ddm[i];
			if (dm.moving)
			{
				reprap.GetPlatform()->SetDirection(i, dm.direction, true);
			}
		}
		return reprap.GetPlatform()->ScheduleInterrupt(firstStepTime + moveStartTime);
	}
}

extern uint32_t /*maxStepTime,*/ maxCalcTime, minCalcTime, maxReps, sqrtErrors, lastRes; extern uint64_t lastNum;	//DEBUG

inline uint32_t DDA::CalcNextStepTime(DriveMovement& dm)
{
	++dm.nextStep;
	if (dm.nextStep > dm.totalSteps)
	{
		dm.moving = false;
		return NoStepTime;
	}

	if (dm.nextStep < dm.accelStopStep)
	{
		dm.nextStepTime = (isqrt(startSpeedTimesCdivAsquared + (dm.twoCsquaredTimesMmPerStepDivA * dm.nextStep)) - startSpeedTimesCdivA);
	}
	else if (dm.nextStep < dm.decelStartStep)
	{
		dm.nextStepTime = (uint32_t)(((uint64_t)dm.mmPerStepTimesCdivtopSpeed * dm.nextStep)/mmPerStepFactor) + accelClocksMinusAccelDistanceTimesCdivTopSpeed;
	}
	else
	{
		uint64_t temp = dm.twoCsquaredTimesMmPerStepDivA * dm.nextStep;
		dm.nextStepTime = (temp > topSpeedTimesCdivAsquaredPlusTwoDecelStartDistanceTimesCsquareddDivA)
								? topSpeedTimesCdivAPlusDecelStartClocks
								: topSpeedTimesCdivAPlusDecelStartClocks - isqrt(topSpeedTimesCdivAsquaredPlusTwoDecelStartDistanceTimesCsquareddDivA - temp);
	}
	//TODO include Bowden elasticity compensation and delta support
	return dm.nextStepTime;
}

bool DDA::Step()
{
	if (state != executing)
	{
		return false;
	}

	bool repeat;
uint32_t numReps = 0;
	do
	{
++numReps;
if (numReps > maxReps) maxReps = numReps;
		uint32_t now = Platform::GetInterruptClocks() - moveStartTime;		// how long since the move started
		uint32_t nextInterruptTime = NoStepTime;
		for (size_t drive = 0; drive < DRIVES; ++drive)
		{
			DriveMovement& dm = ddm[drive];
			if (dm.moving)
			{
				// Hit anything?
				if ((endStopsToCheck & (1 << drive)) != 0)
				{
					switch(reprap.GetPlatform()->Stopped(drive))
					{
					case lowHit:
						reprap.GetMove()->HitLowStop(drive, this);
						moveCompletedTime = now + settleClocks;
						state = completed;
						break;

					case highHit:
						reprap.GetMove()->HitHighStop(drive, this);
						moveCompletedTime = now + settleClocks;
						state = completed;
						break;

					case lowNear:
						{
							// This code assumes that only one endstop is enabled at a time, which is the usual case.
							// Also, at present only the Z axis can give a LowNear indication anyway.
							float instantDv = reprap.GetPlatform()->InstantDv(drive);
							if (instantDv < topSpeed)
							{
								ReduceHomingSpeed(instantDv, drive);
							}
						}
						break;

					default:
						break;
					}
				}

				uint32_t st0 = dm.nextStepTime;
				if (now + minInterruptInterval >= st0)
				{
					if (st0 > moveCompletedTime)
					{
						moveCompletedTime = st0;							// save in case the move has finished
					}
//uint32_t t1 = Platform::GetInterruptClocks();
					reprap.GetPlatform()->StepHigh(drive);
//uint32_t t2 = Platform::GetInterruptClocks();
//if (t2 - t1 > maxStepTime) maxStepTime = t2 - t1;
					uint32_t st1 = CalcNextStepTime(dm);
					if (st1 < nextInterruptTime)
					{
						nextInterruptTime = st1;
					}
					reprap.GetPlatform()->StepLow(drive);
//uint32_t t3 = Platform::GetInterruptClocks() - t2;
//if (t3 > maxCalcTime) maxCalcTime = t3;
//if (t3 < minCalcTime) minCalcTime = t3;
				}
				else if (st0 < nextInterruptTime)
				{
					nextInterruptTime = st0;
				}
			}
		}

		if (nextInterruptTime == NoStepTime)
		{
			state = completed;
		}
		if (state == completed)
		{
			// Schedule the next move immediately - we put the spaces between moves at the start of moves, not the end
			return reprap.GetMove()->StartNextMove(moveStartTime + moveCompletedTime);
		}
		repeat = reprap.GetPlatform()->ScheduleInterrupt(nextInterruptTime + moveStartTime);
	} while (repeat);
	return false;
}

// This is called when we abort a move because we have hit and endstop.
// It adjusts the end points of the current move to account for how far through the move we got.
// The end point coordinate of the axis whose endstop we hit generally doesn't matter, because we will reset it,
// however if we are using compensation then the other coordinates do matter.
void DDA::MoveAborted()
{
	for (size_t drive = 0; drive < AXES; ++drive)
	{
		const DriveMovement& dm = ddm[drive];
		if (dm.moving)
		{
			int32_t stepsLeft = dm.totalSteps - dm.nextStep;
			if (dm.direction)
			{
				endPoint[drive] += stepsLeft;			// we were going backwards
			}
			else
			{
				endPoint[drive] -= stepsLeft;			// we were going forwards
			}
		}
	}
}

// Reduce the speed of this move to the indicated speed.
// 'drive' is the drive whose near-endstop we hit, so it should be the one with the major movement.
// This is called from the ISR, so interrupts are disabled and nothing else can mess with us.
// As this is only called for homing moves and with very low speeds, we assume that we don't need acceleration or deceleration phases.
void DDA::ReduceHomingSpeed(float newSpeed, size_t endstopDrive)
{
	topSpeed = newSpeed;
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		DriveMovement& dm = ddm[drive];
		if (dm.moving)
		{
			// Force the linear motion phase
			dm.decelStartStep = dm.totalSteps + 1;
			dm.accelStopStep = 0;

			// Adjust the speed
			dm.mmPerStepTimesCdivtopSpeed = (uint32_t)(((float)stepClockRate * mmPerStepFactor)/(dm.stepsPerMm * newSpeed));
			if (drive == endstopDrive)
			{
				accelClocksMinusAccelDistanceTimesCdivTopSpeed = dm.nextStepTime - (uint32_t)(((uint64_t)dm.mmPerStepTimesCdivtopSpeed * dm.nextStep)/mmPerStepFactor);
			}
		}
	}
}

// Fast 64-bit integer square root function
/* static */ uint32_t DDA::isqrt(uint64_t num)
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
