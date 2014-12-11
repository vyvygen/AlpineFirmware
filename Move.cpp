/*
 * Move.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#include "RepRapFirmware.h"

Move::Move(Platform* p, GCodes* g) : currentDda(NULL)
{
	active = false;

	// Build the DDA ring
	ddaRingAddPointer = new DDA(NULL);
	DDA *dda = ddaRingAddPointer;
	DDA *oldDda;
	for(unsigned int i = 1; i < DdaRingLength; i++)
	{
		oldDda = dda;
		dda = new DDA(dda);
		oldDda->SetPrevious(dda);
	}
	ddaRingAddPointer->SetNext(dda);
	dda->SetPrevious(oldDda);
}

void Move::Init()
{
	for(int8_t i = 0; i < DRIVES; i++)
	{
		reprap.GetPlatform()->SetDirection(i, FORWARDS);
	}

	// Empty the ring (TODO: set all status to empty)
	ddaRingGetPointer = ddaRingAddPointer;
	ddaRingAddPointer->GetPrevious()->Init();			// set the initial positions
	currentDda = nullptr;

	ddaRingLocked = false;
	addNoMoreMoves = false;

	// Put the origin on the lookahead ring with default velocity in the previous
	// position to the first one that will be used.
	for(unsigned int i = 0; i < DRIVES; i++)
	{
		liveCoordinates[i] = 0.0;
	}

	int8_t slow = reprap.GetPlatform()->SlowestDrive();
	liveCoordinates[DRIVES] = reprap.GetPlatform()->HomeFeedRate(slow);

	currentFeedrate = -1.0;

	SetIdentityTransform();
	tanXY = 0.0;
	tanYZ = 0.0;
	tanXZ = 0.0;

	lastZHit = 0.0;
	zProbing = false;

	for(uint8_t point = 0; point < NUMBER_OF_PROBE_POINTS; point++)
	{
		xBedProbePoints[point] = (0.3 + 0.6*(float)(point%2))*reprap.GetPlatform()->AxisMaximum(X_AXIS);
		yBedProbePoints[point] = (0.0 + 0.9*(float)(point/2))*reprap.GetPlatform()->AxisMaximum(Y_AXIS);
		zBedProbePoints[point] = 0.0;
		probePointSet[point] = unset;
	}

	lastTime = reprap.GetPlatform()->Time();
	longWait = lastTime;

    deltaMode = false;
	deltaDiagonal = 0.0;
    deltaRadius = 0.0;
    for (size_t axis = 0; axis < AXES; ++axis)
    {
    	deltaEndstopAdjustments[axis] = 0.0;
    }

	active = true;
}

void Move::Exit()
{
	reprap.GetPlatform()->Message(BOTH_MESSAGE, "Move class exited.\n");
	active = false;
}

void Move::Spin()
{
	if (!active)
	{
		return;
	}

	// If we either don't want to, or can't, add to the DDA ring, go home.
	if (addNoMoreMoves || ddaRingAddPointer->GetState() != DDA::empty)
	{
		reprap.GetPlatform()->ClassReport("Move", longWait);
		return;
	}

	// If there's a G Code move available, add it to the DDA ring for processing.
	float nextMove[DRIVES + 1];
	EndstopChecks endStopsToCheck;
    float normalisedDirectionVector[DRIVES];	// Used to hold a unit-length vector in the direction of motion
	if (reprap.GetGCodes()->ReadMove(nextMove, endStopsToCheck))
	{
		Transform(nextMove);
		currentFeedrate = nextMove[DRIVES];		// might be G1 with just an F field

		bool noMove = true;
		DDA *newDda = ddaRingAddPointer;
		DDA *prevDda = newDda->GetPrevious();
		for (size_t drive = 0; drive < DRIVES; drive++)
		{
			nextMachineEndPoints[drive] = EndPointToMachine(drive, nextMove[drive]);
			if (drive < AXES)
			{
				if (nextMachineEndPoints[drive] - prevDda->MachineCoordinates()[drive] != 0)
				{
					noMove = false;
				}
				normalisedDirectionVector[drive] = nextMove[drive] - prevDda->MachineToEndPoint(drive);
			}
			else
			{
				if (nextMachineEndPoints[drive] != 0)
				{
					noMove = false;
				}
				normalisedDirectionVector[drive] = nextMove[drive];
			}
		}

		// Throw it away if there's no real movement.
		if (noMove)
		{
			reprap.GetPlatform()->ClassReport("Move", longWait);
			return;
		}

		// Compute the direction of motion, moved to the positive hyperquadrant
		Absolute(normalisedDirectionVector, DRIVES);
		float distanceMoved = Normalise(normalisedDirectionVector, DRIVES);

		// Real move - record its feedrate with it, not here.
		currentFeedrate = -1.0;

		// Get the acceleration
		float acceleration = VectorBoxIntersection(normalisedDirectionVector, reprap.GetPlatform()->Accelerations(), DRIVES);

		// Set the speed to the smaller of the requested and maximum speed.
		// Note: this assumes that the requested feedrate is the Cartesian diagonal of all the drive moves (as in the RRP 0.79 release).
		// This will cause the actual feedrate to be slightly lower than requested if an XYZ move includes extrusion, however the effect is only a few percent.
		float speed = min<float>(nextMove[DRIVES], VectorBoxIntersection(normalisedDirectionVector, reprap.GetPlatform()->MaxFeedrates(), DRIVES));

		// See which already-prepared moves we can adjust. The ISR could change the DDAs while we are doing this, so we disable interrupts for a short while.
		cpu_irq_disable();
		const DDA *limitDda = currentDda;
		uint32_t timeToRun = (limitDda == nullptr) ? 0 : limitDda->GetTimeLeft();
		limitDda = ddaRingGetPointer;
		while (limitDda->GetState() != DDA::empty && timeToRun < freezeTime)
		{
			timeToRun += limitDda->GetTimeNeeded();
			limitDda = limitDda->next;
			if (limitDda == newDda)
			{
				limitDda = nullptr;
				break;
			}
		}
		cpu_irq_enable();

		// We are safe to adjust all previous DDAs between limitDDA and the DDA we are about to fill in,
		// because we should have finished adjusting them before the ISR starts working on them.
		if (limitDda == ddaRingAddPointer)
		{
			limitDda = nullptr;		// there are no previous DDAs we can adjust
		}

		newDda->Init(nextMachineEndPoints, distanceMoved, speed, acceleration, normalisedDirectionVector, endStopsToCheck, limitDda);
	}
	reprap.GetPlatform()->ClassReport("Move", longWait);
}

// Take a unit positive-hyperquadrant vector, and return the factor needed to obtain
// length of the vector as projected to touch box[].

float Move::VectorBoxIntersection(const float v[], const float box[], int8_t dimensions)
{
	// Generate a vector length that is guaranteed to exceed the size of the box

	float biggerThanBoxDiagonal = 2.0*Magnitude(box, dimensions);
	float magnitude = biggerThanBoxDiagonal;
	for (size_t d = 0; d < dimensions; d++)
	{
		if (biggerThanBoxDiagonal*v[d] > box[d])
		{
			float a = box[d]/v[d];
			if (a < magnitude)
			{
				magnitude = a;
			}
		}
	}
	return magnitude;
}

// Normalise a vector, and also return its previous magnitude
// If the vector is of 0 length, return a negative magnitude

float Move::Normalise(float v[], int8_t dimensions)
{
	float magnitude = Magnitude(v, dimensions);
	if (magnitude <= 0.0)
	{
		return -1.0;
	}
	Scale(v, 1.0/magnitude, dimensions);
	return magnitude;
}

// Return the magnitude of a vector

float Move::Magnitude(const float v[], int8_t dimensions)
{
	float magnitude = 0.0;
	for (size_t d = 0; d < dimensions; d++)
	{
		magnitude += v[d]*v[d];
	}
	magnitude = sqrt(magnitude);
	return magnitude;
}

// Multiply a vector by a scalar

void Move::Scale(float v[], float scale, int8_t dimensions)
{
	for(size_t d = 0; d < dimensions; d++)
	{
		v[d] = scale*v[d];
	}
}

// Move a vector into the positive hyperquadrant

void Move::Absolute(float v[], int8_t dimensions)
{
	for(size_t d = 0; d < dimensions; d++)
	{
		v[d] = fabs(v[d]);
	}
}

#if 0
// These are the actual numbers we want in the positions, so don't transform them.
void Move::SetPositions(float move[])
{
	for(uint8_t drive = 0; drive < DRIVES; drive++)
	{
		lastMove->SetDriveCoordinate(move[drive], drive);
	}
	lastMove->SetFeedRate(move[DRIVES]);
}

void Move::SetFeedrate(float feedRate)
{
	lastMove->SetFeedRate(feedRate);
}
#endif

void Move::Diagnostics()
{
	reprap.GetPlatform()->AppendMessage(BOTH_MESSAGE, "Move Diagnostics:\n");

#if 0
  if(active)
    platform->Message(HOST_MESSAGE, " active\n");
  else
    platform->Message(HOST_MESSAGE, " not active\n");

  platform->Message(HOST_MESSAGE, " look ahead ring count: ");
  snprintf(scratchString, STRING_LENGTH, "%d\n", lookAheadRingCount);
  platform->Message(HOST_MESSAGE, scratchString);
  if(dda == NULL)
    platform->Message(HOST_MESSAGE, " dda: NULL\n");
  else
  {
    if(dda->Active())
      platform->Message(HOST_MESSAGE, " dda: active\n");
    else
      platform->Message(HOST_MESSAGE, " dda: not active\n");

  }
  if(ddaRingLocked)
    platform->Message(HOST_MESSAGE, " dda ring is locked\n");
  else
    platform->Message(HOST_MESSAGE, " dda ring is not locked\n");
  if(addNoMoreMoves)
    platform->Message(HOST_MESSAGE, " addNoMoreMoves is true\n\n");
  else
    platform->Message(HOST_MESSAGE, " addNoMoreMoves is false\n\n");
#endif
}

// Returns steps from units (mm) for a particular drive
int32_t Move::EndPointToMachine(int8_t drive, float coord)
{
	return (int32_t)roundf(coord * reprap.GetPlatform()->DriveStepsPerUnit(drive));
}

// Do the Axis transform BEFORE the bed transform
void Move::AxisTransform(float xyzPoint[]) const
{
	xyzPoint[X_AXIS] = xyzPoint[X_AXIS] + tanXY*xyzPoint[Y_AXIS] + tanXZ*xyzPoint[Z_AXIS];
	xyzPoint[Y_AXIS] = xyzPoint[Y_AXIS] + tanYZ*xyzPoint[Z_AXIS];
}

// Invert the Axis transform AFTER the bed transform
void Move::InverseAxisTransform(float xyzPoint[]) const
{
	xyzPoint[Y_AXIS] = xyzPoint[Y_AXIS] - tanYZ*xyzPoint[Z_AXIS];
	xyzPoint[X_AXIS] = xyzPoint[X_AXIS] - (tanXY*xyzPoint[Y_AXIS] + tanXZ*xyzPoint[Z_AXIS]);
}


void Move::Transform(float xyzPoint[]) const
{
	AxisTransform(xyzPoint);
	BedTransform(xyzPoint);
}

void Move::InverseTransform(float xyzPoint[]) const
{
	InverseBedTransform(xyzPoint);
	InverseAxisTransform(xyzPoint);
}

void Move::BedTransform(float xyzPoint[]) const
{
	//TODO implement bed compensation
}

void Move::InverseBedTransform(float xyzPoint[]) const
{
	//TODO implement bed compensation
}

void Move::SetIdentityTransform()
{
	identityBedTransform = true;
	//TODO anything else?
}

float Move::AxisCompensation(int8_t axis) const
{
	switch(axis)
	{
		case X_AXIS:
			return tanXY;

		case Y_AXIS:
			return tanYZ;

		case Z_AXIS:
			return tanXZ;

		default:
			reprap.GetPlatform()->Message(HOST_MESSAGE, "Axis compensation requested for non-existent axis.");
	}
	return 0.0;
}

void Move::SetAxisCompensation(int8_t axis, float tangent)
{
	switch(axis)
	{
	case X_AXIS:
		tanXY = tangent;
		break;
	case Y_AXIS:
		tanYZ = tangent;
		break;
	case Z_AXIS:
		tanXZ = tangent;
		break;
	default:
		reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "SetAxisCompensation: dud axis.\n");
	}
}

void Move::GetDeltaParameters(float &diagonal, float &radius) const
{
	diagonal = deltaDiagonal;
	radius = deltaRadius;
}

void Move::SetDeltaParameters(float diagonal, float radius)
{
	deltaDiagonal = diagonal;
	deltaRadius = radius;
	deltaMode = (radius != 0.0 && diagonal != 0.0);
}

const float *Move::GetDeltaEndstopAdjustments() const
{
	return deltaEndstopAdjustments;
}

void Move::SetDeltaEndstopAdjustments(float x, float y, float z)
{
	deltaEndstopAdjustments[X_AXIS] = x;
	deltaEndstopAdjustments[Y_AXIS] = y;
	deltaEndstopAdjustments[Z_AXIS] = z;
}

// End
