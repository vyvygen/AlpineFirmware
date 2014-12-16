/*
 * DDA.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

//****************************************************************************************************

#include "RepRapFirmware.h"
//#include <cinttypes>
#define PRIu64	"%llu"

const uint32_t NoStepTime = 0xFFFFFFFF;		// value to indicate that no further steps are needed when calculating the next step time

void DriveMovement::DebugPrint() const
{
	if (moving)
	{
		debugPrintf("DM: dv=%f stepsPerMm=%f totSteps=%u 2CsqtMmPerStepDivA=" PRIu64 " accelStopStep=%u decelStartStep=%u mmPerStepTimesCdivtopSpeed=%u"
					"\n",
					dv, stepsPerMm, totalSteps, twoCsquaredTimesMmPerStepDivA, accelStopStep, decelStartStep, mmPerStepTimesCdivtopSpeed
					);
	}
	else
	{
		debugPrintf("DM: not moving\n");
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
	debugPrintf(" sstcda=%u sstcda2=" PRIu64 " tstcda=%u tstdca2=" PRIu64
				" adtcdtsmac=%u 2dsdtc2diva=" PRIu64 " dsc=%u"
				"\n",
				startSpeedTimesCdivA, startSpeedTimesCdivAsquared, topSpeedTimesCdivA, topSpeedTimesCdivAsquared,
				accelClocksMinusAccelDistanceTimesCdivTopSpeed, twoDecelStartDistanceTimesCsquareddDivA, decelStartClocks);
	ddm[1].DebugPrint();
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


// The remaining functions are speed-critical, so use full optimization
#pragma GCC optimize ("O3")

// Prepare this DDA for execution
void DDA::Prepare()
{
	startSpeedTimesCdivA = (uint32_t)((startSpeed * (float)stepClockRate)/acceleration);
	startSpeedTimesCdivAsquared = (uint64_t)startSpeedTimesCdivA * startSpeedTimesCdivA;
	topSpeedTimesCdivA = (uint32_t)((topSpeed * (float)stepClockRate)/acceleration);
	topSpeedTimesCdivAsquared = (uint64_t)topSpeedTimesCdivA * topSpeedTimesCdivA;
	uint32_t accelClocks = (uint32_t)(accelStopTime * (float)stepClockRate);
	accelClocksMinusAccelDistanceTimesCdivTopSpeed = accelClocks - (uint32_t)((accelDistance * (float)stepClockRate)/topSpeed);
	twoDecelStartDistanceTimesCsquareddDivA = (uint64_t)(((float)stepClockRate * (float)stepClockRate * decelStartDistance)/acceleration) * 2;
	decelStartClocks = decelStartTime * (float)stepClockRate;

	firstStepTime = NoStepTime;
	for (size_t i = 0; i < DRIVES; ++i)
	{
		DriveMovement& dm = ddm[i];
		if (dm.moving)
		{
			dm.accelStopStep = (uint32_t)(accelDistance * dm.stepsPerMm) + 1;
			dm.decelStartStep = (uint32_t)(decelStartDistance * dm.stepsPerMm) + 1;
			dm.mmPerStepTimesCdivtopSpeed = (uint32_t)((float)stepClockRate/(dm.stepsPerMm * topSpeed));
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
				reprap.GetPlatform()->SetDirection(i, dm.direction);
			}
		}
		return reprap.GetPlatform()->ScheduleInterrupt(firstStepTime + moveStartTime);
	}
}

extern uint32_t maxStepTime, maxCalcTime;	//DEBUG

bool DDA::Step()
{
	if (state != executing)
	{
		return false;
	}

	bool repeat;
	do
	{
		uint32_t now = Platform::GetInterruptClocks() - moveStartTime;
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
						moveCompletedTime = st0;			// save in case the move has finished
					}
uint32_t t1 = Platform::GetInterruptClocks();
					reprap.GetPlatform()->Step(drive);
uint32_t t2 = Platform::GetInterruptClocks();
if (t2 - t1 > maxStepTime) maxStepTime = t2 - t1;
					uint32_t st1 = CalcNextStepTime(dm);
					if (st1 < nextInterruptTime)
					{
						nextInterruptTime = st1;
					}
uint32_t t3 = Platform::GetInterruptClocks() - t2;
if (t3 > maxCalcTime) maxCalcTime = t3;
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

uint32_t DDA::CalcNextStepTime(DriveMovement& dm)
{
	++dm.nextStep;
	if (dm.nextStep > dm.totalSteps)
	{
		dm.moving = false;
		return NoStepTime;
	}

#if 1
	if (dm.nextStep < dm.accelStopStep)
	{
		dm.nextStepTime = (isqrt(startSpeedTimesCdivAsquared + (dm.twoCsquaredTimesMmPerStepDivA * dm.nextStep)) - startSpeedTimesCdivA);
	}
	else if (dm.nextStep < dm.decelStartStep)
	{
		dm.nextStepTime = (dm.mmPerStepTimesCdivtopSpeed * dm.nextStep) + accelClocksMinusAccelDistanceTimesCdivTopSpeed;
	}
	else
	{
		dm.nextStepTime = topSpeedTimesCdivA - isqrt(topSpeedTimesCdivAsquared - ((dm.twoCsquaredTimesMmPerStepDivA * dm.nextStep) - twoDecelStartDistanceTimesCsquareddDivA)) + decelStartClocks;
	}
	//TODO include Bowden elasticity compensation and delta support
#else
	float distanceMoved = dm.mmPerStep * (float)(dm.nextStep);		// we could accumulate this instead of multiplying, but we might then get unacceptable rounding errors
	float tempStepTime;
	if (distanceMoved < accelDistance)
	{
		tempStepTime = (sqrt((startSpeed * startSpeed) + (acceleration * distanceMoved * 2.0)) - startSpeed) * recipAccel;
	}
	else if (distanceMoved < decelStartDistance)
	{
		tempStepTime = (distanceMoved - accelDistance)/topSpeed + accelStopTime;
	}
	else
	{
		tempStepTime = (topSpeed - sqrt((topSpeed * topSpeed) - ((distanceMoved - decelStartDistance) * acceleration * 2.0))) * recipAccel + decelStartTime;
	}
	//TODO include Bowden elasticity compensation and delta support
	dm.nextStepTime = (uint32_t)(tempStepTime * (float)stepClockRate);
#endif
	return dm.nextStepTime;
}

void DDA::MoveAborted()
{
//TODO implement
}

// Return the highest power of four <= the argument
inline uint32_t GetBit(uint32_t num)
{
#if 1
	if (num >= (1 << 16))
	{
		if (num >= (1 << 24))
		{
			if (num >= (1 << 28))
			{
				if (num >= (1 << 30))
				{
					return 1 << 30;
				}
				else
				{
					return 1 << 28;
				}
			}
			else if (num >= (1 << 26))
			{
				return 1 << 26;
			}
			else
			{
				return 1 << 24;
			}
		}
		else if (num >= (1 << 20))
		{
			if (num >= (1 << 22))
			{
				return 1 << 22;
			}
			else
			{
				return 1 << 20;
			}
		}
		else if (num >= (1 << 18))
		{
			return 1 << 18;
		}
		else
		{
			return 1 << 16;
		}
	}
	else if (num >= (1 << 8))
	{
		if (num >= (1 << 12))
		{
			if (num >= (1 << 14))
			{
				return 1 << 14;
			}
			else
			{
				return 1 << 12;
			}
		}
		else if (num >= (1 << 10))
		{
			return 1 << 10;
		}
		else
		{
			return 1 << 8;
		}
	}
	else if (num >= (1 << 4))
	{
		if (num >= (1 << 6))
		{
			return 1 << 6;
		}
		else
		{
			return 1 << 4;
		}
	}
	else if (num >= (1 >> 2))
	{
		return 1 << 2;
	}
	else
	{
		return 1;
	}
#else
	uint32_t bit = 1 << 30;
	while (bit > num)
	{
		bit >>= 2;
	}
	return bit;
#endif
}

// Fast 64-bit integer square root function
/* static */ uint32_t DDA::isqrt(uint64_t num)
{
	uint32_t numHigh = num >> 32;
	if (numHigh != 0)
	{
		uint64_t res = 0;
		uint64_t bit = static_cast<uint64_t>(GetBit(numHigh)) << 32;	// "bit" starts at the highest power of four <= the argument

		if (num >= bit)
		{
			num -=  bit;
			res = bit;
		}
		bit >>= 2;

		do
		{
			if (num >= res + bit)
			{
				num -= res + bit;
				res = (res >> 1) + bit;
			}
			else
			{
				res >>= 1;
			}
			bit >>= 2;
		} while (bit != 0);
		return (uint32_t)res;
	}
	else
	{
		// 32-bit square root
		uint32_t num32 = (uint32_t)num;
		uint32_t bit = GetBit(num32);
		uint32_t res = 0;

		while (bit != 0)
		{
			if (num32 >= res + bit)
			{
				num32 -= res + bit;
				res = (res >> 1) + bit;
			}
			else
			{
				res >>= 1;
			}
			bit >>= 2;
		}
		return res;
	}
}

// End
