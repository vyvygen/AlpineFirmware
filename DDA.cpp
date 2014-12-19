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
	debugPrintf("DDA: irqc=%u d=%f a=%f reqv=%f"
				" topv=%f startv=%f tstopa=%f tstartd=%f ttotal=%f dstopa=%f dstartd=%f",
				firstStepTime, totalDistance, acceleration, requestedSpeed,
				topSpeed, startSpeed, accelStopTime, decelStartTime, totalTime, accelDistance, decelStartDistance);
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

// Set up a real move
void DDA::Init(
		int32_t ep[],						// the endpoints of this move (for XYZ) or the amount of movement (for extruders)
		float distanceMoved,				// the total distance moved in mm
		float reqSpeed,						// the requested speed in mm/sec
		float acc,							// the acceleration in mm/sec^2
		const float *directionVector,		// the unit direction vector normalised to the positive hyperquadrant
		EndstopChecks ce,					// which axis endstops we must check in this move
		const DDA *limitDda)				// the earliest of the preceding DDAs whose end speed we can adjust, or nullptr if we can't adjust any of them
{
	// 1. Store some values
	endStopsToCheck = ce;
	totalDistance = distanceMoved;
	requestedSpeed = reqSpeed;
	acceleration = acc;

	const int32_t *startPosition = prev->MachineCoordinates();

	for (size_t i = 0; i < DRIVES; ++i)
	{
		int32_t thisEndpoint = ep[i];
		endPoint[i] = thisEndpoint;
		int32_t delta = (i < AXES) ? thisEndpoint - startPosition[i] : thisEndpoint;
		DriveMovement& dm = ddm[i];
		dm.moving = (delta != 0);
		if (dm.moving)
		{
			dm.nextStep = 0;
			dm.totalSteps = labs(delta);
			dm.direction = (delta >= 0);
			dm.dv = directionVector[i];
			dm.stepsPerMm = reprap.GetPlatform()->DriveStepsPerUnit(i) * dm.dv;
			dm.twoCsquaredTimesMmPerStepDivA = (uint64_t)(((float)stepClockRate * (float)stepClockRate)/(dm.stepsPerMm * acceleration)) * 2;
			dm.elasticComp = reprap.GetPlatform()->GetElasticComp(i) * dm.dv;
		}
	}

	startPinned = false;

	// 2. Calculate the accelerate and decelerate distances and the top speed
	float decelDistance;
	if (limitDda == nullptr)
	{
		// There is no previous move, so this move must start at zero speed, and for now it must also end at zero speed.
		// Calculate the distance required to accelerate to the requested speed: v^2 = u^2 + 2as
		startSpeed = 0.0;
		float distanceToReqSpeed = (reqSpeed * reqSpeed)/(2.0 * acceleration);
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
		startPinned = true;			// can't adjust the starting speed of this move
	}
	else
	{
		// Assuming that this move ends with zero speed, calculate the maximum possible starting speed: u^2 = v^2 - 2as
		float maxStartSpeed = sqrt(acceleration * totalDistance * 2.0);

		// Determine the ideal starting speed, given that the end speed is constrained to zero until we have another move to follow it
		float idealStartSpeed = min<float>(maxStartSpeed, requestedSpeed);

		// Try to meld this move to the previous move to avoid stop/start
		startSpeed = prev->AdjustEndSpeed(idealStartSpeed, directionVector);
		if (startSpeed < idealStartSpeed || startSpeed == requestedSpeed)
		{
			startPinned = true;
		}

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

	// Now convert the accelerate/decelerate distances to times
	decelStartDistance = totalDistance - decelDistance;
	accelStopTime = sqrt((2.0 * accelDistance)/acceleration);
	decelStartTime = accelStopTime + (decelStartDistance - accelDistance)/topSpeed;
	totalTime = decelStartTime + sqrt((2.0 * decelDistance)/acceleration);

	state = provisional;
}

float DDA::MachineToEndPoint(size_t drive) const
{
	return ((float)(endPoint[drive]))/reprap.GetPlatform()->DriveStepsPerUnit(drive);
}

float DDA::AdjustEndSpeed(float idealStartSpeed, const float *directionVector)
{
	return 0.0;		//TODO implement this
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
	startSpeedTimesCdivA = (uint32_t)((startSpeed * (float)stepClockRate)/acceleration);
	startSpeedTimesCdivAsquared = (uint64_t)startSpeedTimesCdivA * startSpeedTimesCdivA;
	uint32_t topSpeedTimesCdivA = (uint32_t)((topSpeed * (float)stepClockRate)/acceleration);
	uint32_t decelStartClocks = decelStartTime * (float)stepClockRate;
	topSpeedTimesCdivAPlusDecelStartClocks = topSpeedTimesCdivA + decelStartClocks;
	topSpeedTimesCdivAsquaredPlusTwoDecelStartDistanceTimesCsquareddDivA =
		((uint64_t)topSpeedTimesCdivA * topSpeedTimesCdivA) + ((uint64_t)(((float)stepClockRate * (float)stepClockRate * decelStartDistance)/acceleration) * 2);
	uint32_t accelClocks = (uint32_t)(accelStopTime * (float)stepClockRate);
	accelClocksMinusAccelDistanceTimesCdivTopSpeed = accelClocks - (uint32_t)((accelDistance * (float)stepClockRate)/topSpeed);

	firstStepTime = NoStepTime;
	for (size_t i = 0; i < DRIVES; ++i)
	{
		DriveMovement& dm = ddm[i];
		if (dm.moving)
		{
			dm.accelStopStep = (uint32_t)(accelDistance * dm.stepsPerMm) + 1;
			dm.decelStartStep = (uint32_t)(decelStartDistance * dm.stepsPerMm) + 1;
			dm.mmPerStepTimesCdivtopSpeed = (uint32_t)(((float)stepClockRate * (float)mmPerStepFactor)/(dm.stepsPerMm * topSpeed));
			uint32_t st = CalcNextStepTime(dm);
			if (st < firstStepTime)
			{
				firstStepTime = st;
			}
		}
	}

	state = frozen;
}

// Start executing the move, returning true if Step() needs to be called immediately.
bool DDA::Start(uint32_t tim)
{
	if (state == provisional)
	{
		// This should preferably not happen, because if Start() is called from the step interrupt,
		// then calling Prepare() may take too long and the next steps may be late.
		// Also, method Spin() might be trying to update this DDA.
		Prepare();
	}

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
					//TODO implement this
//					velocity = instantDv;		// slow down because we are getting close
					break;
				default:
					break;
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

// Fast 64-bit integer square root function
/* static */ uint32_t DDA::isqrt(uint64_t num)
{
irqflags_t flags = cpu_irq_save();
uint32_t t2 = Platform::GetInterruptClocks();
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

uint32_t t3 = Platform::GetInterruptClocks() - t2; if (t3 < minCalcTime) minCalcTime = t3; if (t3 > maxCalcTime) maxCalcTime = t3;
cpu_irq_restore(flags);
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

uint32_t t3 = Platform::GetInterruptClocks() - t2; if (t3 < minCalcTime) minCalcTime = t3; if (t3 > maxCalcTime) maxCalcTime = t3;
cpu_irq_restore(flags);
//uint64_t num3 = (uint64_t)res32 * res32; if (num3 > num || (num - num3) > 2*res32) {++sqrtErrors; lastNum = num; lastRes = res32; }
		return res32;
	}
}

void DDA::MoveAborted()
{
//TODO implement
}

// End
