/*
 * DDA.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

//****************************************************************************************************

#include "RepRapFirmware.h"

DDA::DDA(DDA* n) : state(empty), next(n), prev(nullptr)
{
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
	recipAccel = 1.0/acc;

	const int32_t *positionNow = prev->MachineCoordinates();

	for (size_t i = 0; i < DRIVES; ++i)
	{
		int32_t thisEndpoint = ep[i];
		endPoint[i] = thisEndpoint;
		int32_t delta = (i < AXES) ? thisEndpoint - positionNow[i] : thisEndpoint;
		DriveMovement& dm = ddm[i];
		dm.moving = (delta != 0);
		if (dm.moving)
		{
			dm.nextStep = 0;
			dm.totalSteps = labs(delta);
			dm.direction = (delta >= 0);
			float dv = directionVector[i];
			dm.mmPerStep = 1.0/(reprap.GetPlatform()->DriveStepsPerUnit(i) * dv);
			dm.elasticComp = reprap.GetPlatform()->GetElasticComp(i) * dv;
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
			accelStopDistance = decelDistance = 0.5 * totalDistance;
			topSpeed = sqrt((2.0 * accelStopDistance)/acceleration);
			//moveType = accelDecel;
		}
		else
		{
			accelStopDistance = decelDistance = distanceToReqSpeed;
			topSpeed = requestedSpeed;
		}
		startPinned = true;			// can't adjust the starting speed of this move
	}
	else
	{
		// Assuming that this move ends with zero speed, calculate the maximum possible starting speed: u^2 = v^2 - 2as
		float maxStartSpeed = sqrt(2.0 * acceleration * totalDistance);

		// Determine the ideal starting speed, given that the end speed is constrained to zero until we have another move to follow it
		float idealStartSpeed = min<float>(maxStartSpeed, requestedSpeed);

		// Try to meld this move to the previous move to avoid stop/start
		startSpeed = prev->AdjustEndSpeed(idealStartSpeed, directionVector);
		if (startSpeed < idealStartSpeed || startSpeed == requestedSpeed)
		{
			startPinned = true;
		}

		accelStopDistance = ((requestedSpeed * requestedSpeed) - (startSpeed * startSpeed))/(2.0 * acceleration);
		decelDistance = (requestedSpeed * requestedSpeed)/(2.0 * acceleration);		// end speed is zero
		if (accelStopDistance + decelDistance >= totalDistance)
		{
			// It's an accelerate-decelerate move. If V is the peak speed, then (V^2 - u^2)/2a + (V^2 - 0)/2a = distance.
			// So (2V^2 - u^2)/2a = distance
			// So V^2 = a * distance - 0.5u^2
			float vsquared = (acceleration * totalDistance) - 0.5 * startSpeed * startSpeed;
			// Calculate accelerate distance from: V^2 = u^2 + 2as
			accelStopDistance = (vsquared - (startSpeed * startSpeed))/(2.0 * acceleration);
			decelDistance = totalDistance - accelStopDistance;
			topSpeed = sqrt(vsquared);
		}
		else
		{
			topSpeed = requestedSpeed;
		}
	}

	// Now convert the accelerate/decelerate distances to times
	decelStartDistance = totalDistance - decelDistance;
	accelStopTime = sqrt((2.0 * accelStopDistance)/acceleration);
	decelStartTime = accelStopTime + (decelStartDistance - accelStopDistance)/topSpeed;
	totalTime = decelStartTime + sqrt((2.0 * decelDistance)/acceleration);

	state = ready;
}

float DDA::MachineToEndPoint(size_t drive) const
{
	return ((float)(endPoint[drive]))/reprap.GetPlatform()->DriveStepsPerUnit(drive);
}

float DDA::AdjustEndSpeed(float idealStartSpeed, const float *directionVector)
{
	return 0.0;		//TODO implement this
}

// Start executing the move, returning true if Step() needs to be called immediately.
bool DDA::Start(uint32_t tim)
{
	moveStartTime = tim;
	moveCompletedTime = 0;
	state = executing;
	uint32_t firstInterruptTime = 0xFFFFFFFF;
	for (size_t i = 0; i < DRIVES; ++i)
	{
		DriveMovement& dm = ddm[i];
		if (dm.moving)
		{
			reprap.GetPlatform()->SetDirection(i, dm.direction);
			uint32_t st = CalcNextStepTime(dm);
			if (st < firstInterruptTime)
			{
				firstInterruptTime = st;
			}
		}
	}
	debugPrintf("DDA: startc=%u irqc=%u d=%f a=%f 1/a=%f reqv=%f topv=%f startv=%f tstopa=%f tstartd=%f dstopa=%f dstartd=%f, ttotal=%f\n",
					moveStartTime, firstInterruptTime, totalDistance, acceleration, recipAccel, requestedSpeed,
					topSpeed, startSpeed, accelStopTime, decelStartTime, accelStopDistance, decelStartDistance, totalTime);
	if (firstInterruptTime == 0xFFFFFFFF)
	{
		// No steps are pending. This should not happen!
		state = completed;
		return false;
	}
	else
	{
		return reprap.GetPlatform()->ScheduleInterrupt(firstInterruptTime + moveStartTime);
	}
}

bool DDA::Step()
{
	if (state != executing)
	{
		return false;
	}

	bool repeat;
	do
	{
		uint32_t now = reprap.GetPlatform()->GetInterruptClocks() - moveStartTime;
		uint32_t nextInterruptTime = 0xFFFFFFFF;
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
					reprap.GetPlatform()->Step(drive);
					uint32_t st1 = CalcNextStepTime(dm);
					if (st1 < nextInterruptTime)
					{
						nextInterruptTime = st1;
					}
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

		if (nextInterruptTime == 0xFFFFFFFF)
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
		return 0xFFFFFFFF;
	}
	float distanceMoved = dm.mmPerStep * (float)(dm.nextStep);		// we could accumulate this instead of multiplying, but we might then get unacceptable rounding errors
	float tempStepTime;
	if (distanceMoved < accelStopDistance)
	{
		tempStepTime = (sqrt((startSpeed * startSpeed) + (acceleration * distanceMoved * 2.0)) - startSpeed) * recipAccel;
	}
	else if (distanceMoved < decelStartDistance)
	{
		tempStepTime = (distanceMoved - accelStopDistance)/topSpeed + accelStopTime;
	}
	else
	{
		tempStepTime = (topSpeed - sqrt((topSpeed * topSpeed) - ((distanceMoved - decelStartDistance) * acceleration * 2.0))) * recipAccel + decelStartTime;
	}
	//TODO include Bowden elasticity compensation and delta support
	dm.nextStepTime = (uint32_t)(tempStepTime * (float)stepClockRate);
	return dm.nextStepTime;
}

void DDA::MoveAborted()
{
//TODO implement
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

// End
