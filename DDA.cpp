/*
 * DDA.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#include "RepRapFirmware.h"


DDA::DDA(DDA* n) : state(empty), next(n), prev(nullptr)
{
	memset(ddm, 0, sizeof(ddm));	//DEBUG to clear stepError field
}

// Return the number of clocks this DDA still needs to execute.
// This could be slightly negative, if the move is overdue for completion.
int32_t DDA::GetTimeLeft() const
//pre(state == executing || state == frozen || state == completed)
{
	return (state == completed) ? 0
			: (state == executing) ? (int32_t)(moveStartTime + timeNeeded - Platform::GetInterruptClocks())
			: (int32_t)timeNeeded;
}

void DDA::DebugPrint() const
{
	debugPrintf("DDA: fstep=%u d=%f a=%f reqv=%f"
				" topv=%f startv=%f endv=%f tstopa=%f tstartd=%f ttotal=%f daccel=%f ddecel=%f"
				"\n",
				firstStepTime, totalDistance, acceleration, requestedSpeed,
				topSpeed, startSpeed, endSpeed, accelStopTime, decelStartTime, totalTime, accelDistance, decelDistance);
	reprap.GetPlatform()->GetLine()->Flush();
	ddm[0].DebugPrint('x');
	ddm[1].DebugPrint('y');
	ddm[2].DebugPrint('z');
	ddm[3].DebugPrint('1');
	ddm[4].DebugPrint('2');
	reprap.GetPlatform()->GetLine()->Flush();
}

// This is called by Move to initialize all DDAs
void DDA::Init()
{
	// Set the endpoints to zero, because Move asks for them
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		endPoint[drive] = 0;
	}
	state = empty;
	endCoordinatesValid = false;
}

// Set up a real move. Return true if it represents real movement, else false.
bool DDA::Init(const float nextMove[], EndstopChecks ce, bool doDeltaMapping)
{
	// 1. Compute the new endpoints and the movement vector
	bool deltaAxisMovement = false;										// will be set true if we are doing an XYZ move on a delta printer
	const int32_t *positionNow = prev->DriveCoordinates();
	if (doDeltaMapping)
	{
		reprap.GetMove()->DeltaTransform(nextMove, endPoint);			// transform the axis coordinates if on a delta printer
		deltaAxisMovement = (endPoint[X_AXIS] != positionNow[X_AXIS]) || (endPoint[Y_AXIS] != positionNow[Y_AXIS]) || (endPoint[Z_AXIS] != positionNow[Z_AXIS]);
	}

	bool realMove = false, xyMoving = false;
	float normalisedDirectionVector[DRIVES];	// Used to hold a unit-length vector in the direction of motion
	float accelerations[DRIVES];
	const float *normalAccelerations = reprap.GetPlatform()->Accelerations();
	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		if (drive < AXES)
		{
			endCoordinates[drive] = nextMove[drive];
		}
		if (drive >= AXES || !doDeltaMapping)
		{
			endPoint[drive] = Move::MotorEndPointToMachine(drive, nextMove[drive]);
		}
		int32_t delta = (drive < AXES) ? endPoint[drive] - positionNow[drive] : endPoint[drive];
		normalisedDirectionVector[drive] = (float)delta/reprap.GetPlatform()->DriveStepsPerUnit(drive);
		DriveMovement& dm = ddm[drive];
		dm.moving = (delta != 0 || (drive < AXES && deltaAxisMovement));	// on a delta printer, if one tower moves then we assume they all do
		accelerations[drive] = normalAccelerations[drive];
		if (dm.moving)
		{
			dm.totalSteps = labs(delta);
			dm.direction = (delta >= 0);
			realMove = true;
			if (drive < Z_AXIS)
			{
				xyMoving = true;
			}
			if (drive >= AXES && xyMoving)
			{
				dm.compensationTime = reprap.GetPlatform()->GetElasticComp(drive);
				if (dm.compensationTime > 0.0)
				{
					// Compensation causes instant velocity changes equal to acceleration * k, so we may need to limit the acceleration
					accelerations[drive] = min<float>(accelerations[drive], reprap.GetPlatform()->ConfiguredInstantDv(drive)/dm.compensationTime);
				}
			}
			else
			{
				dm.compensationTime = 0;
			}
		}
	}

	// 2. Throw it away if there's no real movement.
	if (!realMove)
	{
		return false;
	}

	// 3. Store some values
	endStopsToCheck = ce;
	endSpeed = 0.0;					// until the next move asks us to adjust it
	endCoordinatesValid = doDeltaMapping;

	// 4. Compute the direction and amount of motion, moved to the positive hyperquadrant
	Move::Absolute(normalisedDirectionVector, DRIVES);
	totalDistance = Move::Normalise(normalisedDirectionVector, DRIVES);

	// 5. Compute the maximum acceleration available
	acceleration = Move::VectorBoxIntersection(normalisedDirectionVector, accelerations, DRIVES);

	// Set the speed to the smaller of the requested and maximum speed.
	// Note: this assumes that the requested feedrate is the Cartesian diagonal of all the drive moves (as in the RRP 0.78 release).
	// This will cause the actual feedrate to be slightly lower than requested if an XYZ move includes extrusion, however the effect is only a few percent.
	requestedSpeed = min<float>(nextMove[DRIVES], Move::VectorBoxIntersection(normalisedDirectionVector, reprap.GetPlatform()->MaxFeedrates(), DRIVES));

	// 6. Set up the parameters for the individual drives
	for (size_t i = 0; i < DRIVES; ++i)
	{
		DriveMovement& dm = ddm[i];
		if (dm.moving)
		{
			dm.dv = normalisedDirectionVector[i];
			dm.stepsPerMm = reprap.GetPlatform()->DriveStepsPerUnit(i) * dm.dv;
			dm.twoCsquaredTimesMmPerStepDivA = (uint64_t)(((float)stepClockRate * (float)stepClockRate)/(dm.stepsPerMm * acceleration)) * 2;
		}
	}

	// 7. Calculate the accelerate and decelerate distances and the top speed
	if (prev->state != provisional)
	{
		// There is no previous move that we can adjust, so this move must start at zero speed, and for now it must also end at zero speed.
		// Calculate the distance required to accelerate to the requested speed: v^2 = u^2 + 2as
		startSpeed = 0.0;
	}
	else
	{
		// Try to meld this move to the previous move to avoid stop/start
		// Assuming that this move ends with zero speed, calculate the maximum possible starting speed: u^2 = v^2 - 2as
		float maxStartSpeed = sqrt(acceleration * totalDistance * 2.0);

		// Determine the ideal starting speed, given that the end speed is constrained to zero until we have another move to follow it
		prev->targetNextSpeed = min<float>(maxStartSpeed, requestedSpeed);
		DoLookahead(prev);
		startSpeed = prev->targetNextSpeed;
	}

	RecalculateMove();
	state = provisional;
	return true;
}

float DDA::GetMotorPosition(size_t drive) const
{
	return Move::MotorEndpointToPosition(endPoint[drive], drive);
}

void DDA::DoLookahead(DDA *laDDA)
//pre(state == provisional)
{
//	if (reprap.Debug(moduleDda)) debugPrintf("Adjusting, %f\n", laDDA->targetNextSpeed);
	unsigned int laDepth = 0;
	bool goingUp = true;

	for(;;)					// this loop is used to nest lookahead without making recursive calls
	{
		bool recurse = false;
		if (goingUp)
		{
			// We have been asked to adjust the end speed of this move to targetStartSpeed
			if (laDDA->topSpeed == laDDA->requestedSpeed)
			{
				// This move already reaches its top speed, so just need to adjust the deceleration part
				laDDA->endSpeed = laDDA->requestedSpeed;
				laDDA->CalcNewSpeeds();
			}
			else if (laDDA->decelDistance == laDDA->totalDistance && laDDA->prev->state == provisional)
			{
				// This move doesn't reach its requested speed, so we may have to adjust the previous move as well to get optimum behaviour
				laDDA->endSpeed = laDDA->requestedSpeed;
				laDDA->CalcNewSpeeds();
				laDDA->prev->targetNextSpeed = min<float>(sqrt((laDDA->endSpeed * laDDA->endSpeed) + (2 * laDDA->acceleration * laDDA->totalDistance)), laDDA->requestedSpeed);
				recurse = true;
			}
			else
			{
				// This move doesn't reach its requested speed, but we can't adjust the previous one
				laDDA->endSpeed = min<float>(sqrt((laDDA->startSpeed * laDDA->startSpeed) + (2 * laDDA->acceleration * laDDA->totalDistance)), laDDA->requestedSpeed);
				laDDA->CalcNewSpeeds();
			}
		}
		else
		{
			laDDA->startSpeed = laDDA->prev->targetNextSpeed;
			float maxEndSpeed = sqrt((laDDA->startSpeed * laDDA->startSpeed) + (2 * laDDA->acceleration * laDDA->totalDistance));
			if (maxEndSpeed < laDDA->endSpeed)
			{
				// Oh dear, we were too optimistic! Have another go.
				laDDA->endSpeed = maxEndSpeed;
				laDDA->CalcNewSpeeds();
			}
		}

		if (recurse)
		{
			laDDA = laDDA->prev;
			++laDepth;
			if (reprap.Debug(moduleDda)) debugPrintf("Recursion start %u\n", laDepth);
		}
		else
		{
			laDDA->RecalculateMove();

			if (laDepth == 0)
			{
//				if (reprap.Debug(moduleDda)) debugPrintf("Complete, %f\n", laDDA->targetNextSpeed);
				return;
			}

			laDDA = laDDA->next;
			--laDepth;
			goingUp = false;
		}
	}
}

// Recalculate the top speed, acceleration distance and deceleration distance
void DDA::RecalculateMove()
{
	accelDistance = ((requestedSpeed * requestedSpeed) - (startSpeed * startSpeed))/(2.0 * acceleration);
	decelDistance = ((requestedSpeed * requestedSpeed) - (endSpeed * endSpeed))/(2.0 * acceleration);
	if (accelDistance + decelDistance >= totalDistance)
	{
		// It's an accelerate-decelerate move. If V is the peak speed, then (V^2 - u^2)/2a + (V^2 - v^2)/2a = distance.
		// So (2V^2 - u^2 - v^2)/2a = distance
		// So V^2 = a * distance + 0.5(u^2 + v^2)
		float vsquared = (acceleration * totalDistance) + 0.5 * ((startSpeed * startSpeed) + (endSpeed * endSpeed));
		// Calculate accelerate distance from: V^2 = u^2 + 2as
		if (vsquared >= 0.0)
		{
			accelDistance = max<float>((vsquared - (startSpeed * startSpeed))/(2.0 * acceleration), 0.0);
			decelDistance = totalDistance - accelDistance;
			topSpeed = sqrt(vsquared);
		}
		else if (startSpeed < endSpeed)
		{
			// This would ideally never happen, but might because of rounding errors
			accelDistance = totalDistance;
			decelDistance = 0.0;
			topSpeed = endSpeed;
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
}

void DDA::CalcNewSpeeds()
{
	// Decide what speed we would really like to start at. There are several possibilities:
	// 1. If the top speed is already the requested speed, use the requested speed.
	// 2. Else if this is a deceleration-only move and the previous move is not frozen, we may be able to increase the start speed,
	//    so use the requested speed again.
	// 3. Else the start speed must be pinned, so use the lower of the maximum speed we can accelerate to and the requested speed.

	// We may have to make multiple passes, because reducing one of the speeds may solve some problems but actually make matters worse on another axis.
	bool limited;
	do
	{
//		debugPrintf("  Pass, start=%f end=%f\n", targetStartSpeed, endSpeed);
		limited = false;
		for (size_t drive = 0; drive < DRIVES; ++drive)
		{
			const DriveMovement& thisMoveDm = ddm[drive];
			const DriveMovement& nextMoveDm = next->ddm[drive];
			if (thisMoveDm.moving || nextMoveDm.moving)
			{
				float thisMoveSpeed = thisMoveDm.GetDriveSpeed(endSpeed);
				float nextMoveSpeed = nextMoveDm.GetDriveSpeed(targetNextSpeed);
				float idealDeltaV = fabsf(thisMoveSpeed - nextMoveSpeed);
				float maxDeltaV = reprap.GetPlatform()->ActualInstantDv(drive);
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
							endSpeed = (fabsf(nextMoveSpeed) + maxDeltaV)/thisMoveDm.dv;
						}
						else
						{
							targetNextSpeed = (fabsf(thisMoveSpeed) + maxDeltaV)/nextMoveDm.dv;
						}
					}
					else if (fabsf(thisMoveSpeed) * 2 < maxDeltaV)
					{
						targetNextSpeed = (maxDeltaV - fabsf(thisMoveSpeed))/nextMoveDm.dv;
					}
					else if (fabsf(nextMoveSpeed) * 2 < maxDeltaV)
					{
						endSpeed = (maxDeltaV - fabsf(nextMoveSpeed))/thisMoveDm.dv;
					}
					else
					{
						targetNextSpeed = maxDeltaV/(2 * nextMoveDm.dv);
						endSpeed = maxDeltaV/(2 * thisMoveDm.dv);
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
}

bool DDA::FetchEndPosition(int32_t ep[DRIVES], float endCoords[AXES])
{
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		ep[drive] = endPoint[drive];
	}
	if (endCoordinatesValid)
	{
		for (size_t axis = 0; axis < AXES; ++axis)
		{
			endCoords[axis] = endCoordinates[axis];
		}
	}
	return endCoordinatesValid;
}

void DDA::SetPositions(const float move[DRIVES])
{
	reprap.GetMove()->EndPointToMachine(move, endPoint, DRIVES);
	for (size_t axis = 0; axis < AXES; ++axis)
	{
		endCoordinates[axis] = move[axis];
	}
	endCoordinatesValid = true;
}

// Get a Cartesian end coordinate from this move
float DDA::GetEndCoordinate(size_t drive, bool disableDeltaMapping)
{
	if (disableDeltaMapping)
	{
		return Move::MotorEndpointToPosition(endPoint[drive], drive);
	}
	else
	{
		if (drive < AXES && !endCoordinatesValid)
		{
			reprap.GetMove()->MachineToEndPoint(endPoint, endCoordinates, AXES);
			endCoordinatesValid = true;
		}
		return endCoordinates[drive];
	}
}

// The remaining functions are speed-critical, so use full optimisation
#pragma GCC optimize ("O3")

// Prepare this DDA for execution
void DDA::Prepare()
{
//debugPrintf("Prep\n");
//reprap.GetPlatform()->GetLine()->Flush();

	PrepParams params;
	params.decelStartDistance = totalDistance - decelDistance;

	// Convert the accelerate/decelerate distances to times
	accelStopTime = (topSpeed - startSpeed)/acceleration;
	decelStartTime = accelStopTime + (params.decelStartDistance - accelDistance)/topSpeed;
	totalTime = decelStartTime + (topSpeed - endSpeed)/acceleration;
	timeNeeded = (uint32_t)(totalTime * stepClockRate);

	params.startSpeedTimesCdivA = (uint32_t)((startSpeed * stepClockRate)/acceleration);
	params.topSpeedTimesCdivA = (uint32_t)((topSpeed * stepClockRate)/acceleration);
	params.decelStartClocks = decelStartTime * stepClockRate;
	params.topSpeedTimesCdivAPlusDecelStartClocks = params.topSpeedTimesCdivA + params.decelStartClocks;
	params.accelClocksMinusAccelDistanceTimesCdivTopSpeed = (uint32_t)((accelStopTime - (accelDistance/topSpeed)) * stepClockRate);
	params.compFactor = 1.0 - startSpeed/topSpeed;

	firstStepTime = DriveMovement::NoStepTime;
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		DriveMovement& dm = ddm[drive];
		if (dm.moving)
		{
			if (drive >= AXES)
			{
				dm.PrepareExtruder(*this, params);
			}
			else if (reprap.GetMove()->IsDeltaMode())
			{
				dm.PrepareDeltaAxis(*this, params);
			}
			else
			{
				dm.PrepareCartesianAxis(*this, params);
			}

			// Check for sensible values, print them if they look dubious
			if (reprap.Debug(moduleDda)
				&& (   dm.totalSteps > 1000000
					|| dm.reverseStartStep < dm.decelStartStep
					|| (dm.reverseStartStep <= dm.totalSteps && dm.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA > (int64_t)(dm.twoCsquaredTimesMmPerStepDivA * dm.reverseStartStep))
				  )
			   )
			{
				DebugPrint();
				reprap.GetPlatform()->GetLine()->Flush();
			}

			// Prepare for the first step
			dm.nextStep = 0;
			dm.nextStepTime = 0;
			uint32_t st = dm.CalcNextStepTime(drive);
			if (st < firstStepTime)
			{
				firstStepTime = st;
			}
		}
	}

	if (reprap.Debug(moduleDda) && reprap.Debug(moduleMove))	// temp show the prepared DDA if debug enabled for both modules
	{
		DebugPrint();
		reprap.GetPlatform()->GetLine()->Flush();
	}
//debugPrintf("Done\n");
//reprap.GetPlatform()->GetLine()->Flush();

	state = frozen;					// must do this last so that the ISR doesn't start executing it before we have finished setting it up
}

// Start executing the move, returning true if Step() needs to be called immediately.
bool DDA::Start(uint32_t tim)
//pre(state == frozen)
{
	moveStartTime = tim;
	moveCompletedTime = 0;
	state = executing;

	if (firstStepTime == DriveMovement::NoStepTime)
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
		uint32_t nextInterruptTime = DriveMovement::NoStepTime;
		for (size_t drive = 0; drive < DRIVES; ++drive)
		{
			DriveMovement& dm = ddm[drive];
			if (dm.moving && !dm.stepError)
			{
				// Hit anything?
				if ((endStopsToCheck & (1 << drive)) != 0)
				{
					switch(reprap.GetPlatform()->Stopped(drive))
					{
					case lowHit:
						endStopsToCheck &= ~(1 << drive);					// clear this check so that we can check for more
						ddm[drive].moving = false;							// stop this drive
						reprap.GetMove()->HitLowStop(drive, this);
						if (endStopsToCheck == 0)							// if no more endstops to check
						{
							moveCompletedTime = now + settleClocks;
							MoveAborted();
						}
						break;

					case highHit:
						endStopsToCheck &= ~(1 << drive);					// clear this check so that we can check for more
						ddm[drive].moving = false;							// stop this drive
						reprap.GetMove()->HitHighStop(drive, this);
						if (endStopsToCheck == 0)							// if no more endstops to check
						{
							moveCompletedTime = now + settleClocks;
							MoveAborted();
						}
						break;

					case lowNear:
						{
							// This code assumes that only one endstop that supports the 'near' function is enabled at a time.
							// Typically, only the Z axis can give a LowNear indication anyway.
							float instantDv = reprap.GetPlatform()->ConfiguredInstantDv(drive);
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
					uint32_t st1 = dm.CalcNextStepTime(drive);
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

		if (nextInterruptTime == DriveMovement::NoStepTime)
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
				endPoint[drive] -= stepsLeft;			// we were going forwards
			}
			else
			{
				endPoint[drive] += stepsLeft;			// we were going backwards
			}
		}
	}
	state = completed;
}

// As MoveAborted, but just do the Z axis, unconditionally
void DDA::SetStoppedHeight()
{
	const DriveMovement& dm = ddm[Z_AXIS];
	int32_t stepsLeft = dm.totalSteps - dm.nextStep;
	if (dm.direction)
	{
		endPoint[Z_AXIS] -= stepsLeft;					// we were going forwards
	}
	else
	{
		endPoint[Z_AXIS] += stepsLeft;					// we were going backwards
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
			dm.ReduceSpeed(newSpeed, drive == endstopDrive);
		}
	}
}

void DDA::PrintIfHasStepError()
{
	bool printed = false;
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		if (ddm[drive].stepError)
		{
			if (!printed)
			{
				DebugPrint();
				printed = true;
			}
			ddm[drive].stepError = false;
		}
	}
}

// End
