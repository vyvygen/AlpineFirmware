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
	DDA *dda = new DDA(NULL);
	ddaRingGetPointer = ddaRingAddPointer = dda;
	for(unsigned int i = 1; i < DdaRingLength; i++)
	{
		DDA *oldDda = dda;
		dda = new DDA(dda);
		oldDda->SetPrevious(dda);
	}
	ddaRingAddPointer->SetNext(dda);
	dda->SetPrevious(ddaRingAddPointer);
}

void Move::Init()
{
	// Put the origin on the lookahead ring with default velocity in the previous
	// position to the first one that will be used.
	for(unsigned int i = 0; i < DRIVES; i++)
	{
		liveCoordinates[i] = 0.0;
		nextMachineEndPoints[i] = 0;
		reprap.GetPlatform()->SetDirection(i, FORWARDS, false);
	}

	// Empty the ring
	ddaRingGetPointer = ddaRingAddPointer;
	DDA *dda = ddaRingAddPointer;
	do
	{
		dda->Init();
		dda = dda->GetNext();
	} while (dda != ddaRingAddPointer);

	currentDda = nullptr;

	addNoMoreMoves = false;

	int8_t slow = reprap.GetPlatform()->SlowestDrive();
	currentFeedrate = liveCoordinates[DRIVES] = reprap.GetPlatform()->HomeFeedRate(slow);

	SetIdentityTransform();
	tanXY = 0.0;
	tanYZ = 0.0;
	tanXZ = 0.0;

	lastZHit = 0.0;
	zProbing = false;

	for(size_t point = 0; point < NUMBER_OF_PROBE_POINTS; point++)
	{
		xBedProbePoints[point] = (0.3 + 0.6*(float)(point%2))*reprap.GetPlatform()->AxisMaximum(X_AXIS);
		yBedProbePoints[point] = (0.0 + 0.9*(float)(point/2))*reprap.GetPlatform()->AxisMaximum(Y_AXIS);
		zBedProbePoints[point] = 0.0;
		probePointSet[point] = unset;
	}

	xRectangle = 1.0/(0.8*reprap.GetPlatform()->AxisMaximum(X_AXIS));
	yRectangle = xRectangle;

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

	// See if we can add another move to the ring
	if (!addNoMoreMoves && ddaRingAddPointer->GetState() == DDA::empty)
	{
		ddaRingAddPointer->PrintIfHasStepError();
		// If there's a G Code move available, add it to the DDA ring for processing.
		float nextMove[DRIVES + 1];
		EndstopChecks endStopsToCheck;
		if (reprap.GetGCodes()->ReadMove(nextMove, endStopsToCheck))
		{
			currentFeedrate = nextMove[DRIVES];		// might be G1 with just an F field
			Transform(nextMove);
			if (ddaRingAddPointer->Init(nextMove, endStopsToCheck))
			{
				const int32_t* machineCoords = ddaRingAddPointer->MachineCoordinates();
				for (size_t drive = 0; drive < DRIVES; ++drive)
				{
					nextMachineEndPoints[drive] = machineCoords[drive];
				}
				ddaRingAddPointer = ddaRingAddPointer->GetNext();
			}
		}
	}

	// See whether we need to kick off a move
	DDA *cdda = currentDda;										// currentDda is volatile, so copy it
	if (cdda == nullptr)
	{
		// No DDA is executing, so start executing a new one if possible
		DDA *dda = ddaRingGetPointer;
		if (dda->GetState() == DDA::provisional)
		{
			dda->Prepare();
		}
		if (StartNextMove(Platform::GetInterruptClocks()))		// start the next move if none is executing already
		{
			cpu_irq_disable();
			Interrupt();
			cpu_irq_enable();
		}
	}
	else
	{
		// See whether we need to prepare any moves
		int32_t preparedTime = 0;
		DDA::DDAState st;
		while ((st = cdda->GetState()) == DDA:: completed || st == DDA::executing || st == DDA::frozen)
		{
			preparedTime += cdda->GetTimeLeft();
			cdda = cdda->GetNext();
		}

		// If the number of prepared moves will execute in less than the minimum time, prepare another move
		while (st == DDA::provisional && preparedTime < (int32_t)(DDA::stepClockRate/8))		// prepare moves one eighth of a second ahead of when they will be needed
		{
			cdda->Prepare();
			preparedTime += cdda->GetTimeLeft();
			cdda = cdda->GetNext();
			st = cdda->GetState();
		}
	}

	reprap.GetPlatform()->ClassReport("Move", longWait, moduleMove);
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
		v[d] = fabsf(v[d]);
	}
}


// These are the actual numbers we want in the positions, so don't transform them.
void Move::SetPositions(float move[])
{
	if (DDARingEmpty())
	{
		DDA *lastMove = ddaRingAddPointer->GetPrevious();
		for (size_t drive = 0; drive < DRIVES; drive++)
		{
			float coord = DDA::EndPointToMachine(drive, move[drive]);
			lastMove->SetDriveCoordinate(coord, drive);
			nextMachineEndPoints[drive] = coord;
		}
	}
	else
	{
		reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "SetPositions called when DDA ring not empty\n");
	}
}

void Move::SetFeedrate(float feedRate)
{
	if (DDARingEmpty())
	{
		DDA *lastMove = ddaRingAddPointer->GetPrevious();
		currentFeedrate = feedRate;
		lastMove->SetFeedRate(feedRate);
	}
	else
	{
		reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "SetFeedrate called when DDA ring not empty\n");
	}
}

uint32_t maxStepTime=0, maxCalcTime=0, minCalcTime = 999, maxReps = 0, sqrtErrors = 0, lastRes = 0; uint64_t lastNum = 0;	//DEBUG

void Move::Diagnostics()
{
	reprap.GetPlatform()->AppendMessage(BOTH_MESSAGE, "Move Diagnostics:\n");

	reprap.GetPlatform()->AppendMessage(BOTH_MESSAGE, "MaxStepClocks: %u, minCalcClocks: %u, maxCalcClocks: %u, maxReps: %u, sqrtErrors: %u, lastRes: %u, lastNum: %" PRIu64 "\n",
										maxStepTime, minCalcTime, maxCalcTime, maxReps, sqrtErrors, lastRes, lastNum);
	maxStepTime = maxCalcTime = maxReps = 0;
	minCalcTime = 999;

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

// Do the bed transform AFTER the axis transform
void Move::BedTransform(float xyzPoint[]) const
{
	if (!identityBedTransform)
	{
		switch(NumberOfProbePoints())
		{
		case 0:
			return;

		case 3:
			xyzPoint[Z_AXIS] = xyzPoint[Z_AXIS] + aX*xyzPoint[X_AXIS] + aY*xyzPoint[Y_AXIS] + aC;
			break;

		case 4:
			xyzPoint[Z_AXIS] = xyzPoint[Z_AXIS] + SecondDegreeTransformZ(xyzPoint[X_AXIS], xyzPoint[Y_AXIS]);
			break;

		case 5:
			xyzPoint[Z_AXIS] = xyzPoint[Z_AXIS] + TriangleZ(xyzPoint[X_AXIS], xyzPoint[Y_AXIS]);
			break;

		default:
			reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "BedTransform: wrong number of sample points.");
		}
	}
}

// Invert the bed transform BEFORE the axis transform
void Move::InverseBedTransform(float xyzPoint[]) const
{
	if (!identityBedTransform)
	{
		switch(NumberOfProbePoints())
		{
		case 0:
			return;

		case 3:
			xyzPoint[Z_AXIS] = xyzPoint[Z_AXIS] - (aX*xyzPoint[X_AXIS] + aY*xyzPoint[Y_AXIS] + aC);
			break;

		case 4:
			xyzPoint[Z_AXIS] = xyzPoint[Z_AXIS] - SecondDegreeTransformZ(xyzPoint[X_AXIS], xyzPoint[Y_AXIS]);
			break;

		case 5:
			xyzPoint[Z_AXIS] = xyzPoint[Z_AXIS] - TriangleZ(xyzPoint[X_AXIS], xyzPoint[Y_AXIS]);
			break;

		default:
			reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "InverseBedTransform: wrong number of sample points.");
		}
	}
}

void Move::SetIdentityTransform()
{
	identityBedTransform = true;
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
			reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "Axis compensation requested for non-existent axis.\n");
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

void Move::BarycentricCoordinates(int8_t p1, int8_t p2, int8_t p3, float x, float y, float& l1, float& l2, float& l3) const
{
	float y23 = baryYBedProbePoints[p2] - baryYBedProbePoints[p3];
	float x3 = x - baryXBedProbePoints[p3];
	float x32 = baryXBedProbePoints[p3] - baryXBedProbePoints[p2];
	float y3 = y - baryYBedProbePoints[p3];
	float x13 = baryXBedProbePoints[p1] - baryXBedProbePoints[p3];
	float y13 = baryYBedProbePoints[p1] - baryYBedProbePoints[p3];
	float iDet = 1.0 / (y23 * x13 + x32 * y13);
	l1 = (y23 * x3 + x32 * y3) * iDet;
	l2 = (-y13 * x3 + x13 * y3) * iDet;
	l3 = 1.0 - l1 - l2;
}

/*
 * Interpolate on a triangular grid.  The triangle corners are indexed:
 *
 *   ^  [1]      [2]
 *   |
 *   Y      [4]
 *   |
 *   |  [0]      [3]
 *      -----X---->
 *
 */
float Move::TriangleZ(float x, float y) const
{
	float l1, l2, l3;
	int8_t j;
	for(int8_t i = 0; i < 4; i++)
	{
		j = (i + 1) % 4;
		BarycentricCoordinates(i, j, 4, x, y, l1, l2, l3);
		if(l1 > TRIANGLE_0 && l2 > TRIANGLE_0 && l3 > TRIANGLE_0)
		{
			return l1 * baryZBedProbePoints[i] + l2 * baryZBedProbePoints[j] + l3 * baryZBedProbePoints[4];
		}
	}
	reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "Triangle interpolation: point outside all triangles!\n");
	return 0.0;
}

void Move::SetProbedBedEquation(StringRef& reply)
{
	switch(NumberOfProbePoints())
	{
	case 3:
		/*
		 * Transform to a plane
		 */

		{
			float x10 = xBedProbePoints[1] - xBedProbePoints[0];
			float y10 = yBedProbePoints[1] - yBedProbePoints[0];
			float z10 = zBedProbePoints[1] - zBedProbePoints[0];
			float x20 = xBedProbePoints[2] - xBedProbePoints[0];
			float y20 = yBedProbePoints[2] - yBedProbePoints[0];
			float z20 = zBedProbePoints[2] - zBedProbePoints[0];
			float a = y10 * z20 - z10 * y20;
			float b = z10 * x20 - x10 * z20;
			float c = x10 * y20 - y10 * x20;
			float d = -(xBedProbePoints[1] * a + yBedProbePoints[1] * b + zBedProbePoints[1] * c);
			aX = -a / c;
			aY = -b / c;
			aC = -d / c;
			identityBedTransform = false;
		}
		break;

	case 4:
		/*
		 * Transform to a ruled-surface quadratic.  The corner points for interpolation are indexed:
		 *
		 *   ^  [1]      [2]
		 *   |
		 *   Y
		 *   |
		 *   |  [0]      [3]
		 *      -----X---->
		 *
		 *   These are the scaling factors to apply to x and y coordinates to get them into the
		 *   unit interval [0, 1].
		 */
		xRectangle = 1.0 / (xBedProbePoints[3] - xBedProbePoints[0]);
		yRectangle = 1.0 / (yBedProbePoints[1] - yBedProbePoints[0]);
		identityBedTransform = false;
		break;

	case 5:
		for(int8_t i = 0; i < 4; i++)
		{
			float x10 = xBedProbePoints[i] - xBedProbePoints[4];
			float y10 = yBedProbePoints[i] - yBedProbePoints[4];
			float z10 = zBedProbePoints[i] - zBedProbePoints[4];
			baryXBedProbePoints[i] = xBedProbePoints[4] + 2.0 * x10;
			baryYBedProbePoints[i] = yBedProbePoints[4] + 2.0 * y10;
			baryZBedProbePoints[i] = zBedProbePoints[4] + 2.0 * z10;
		}
		baryXBedProbePoints[4] = xBedProbePoints[4];
		baryYBedProbePoints[4] = yBedProbePoints[4];
		baryZBedProbePoints[4] = zBedProbePoints[4];
		identityBedTransform = false;
		break;

	default:
		reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "Attempt to set bed compensation before all probe points have been recorded.");
	}

	reply.copy("Bed equation fits points");
	for(int8_t point = 0; point < NumberOfProbePoints(); point++)
	{
		reply.catf(" [%.1f, %.1f, %.3f]", xBedProbePoints[point], yBedProbePoints[point], zBedProbePoints[point]);
	}
}

/*
 * Transform to a ruled-surface quadratic.  The corner points for interpolation are indexed:
 *
 *   ^  [1]      [2]
 *   |
 *   Y
 *   |
 *   |  [0]      [3]
 *      -----X---->
 *
 *   The values of x and y are transformed to put them in the interval [0, 1].
 */
float Move::SecondDegreeTransformZ(float x, float y) const
{
	x = (x - xBedProbePoints[0])*xRectangle;
	y = (y - yBedProbePoints[0])*yRectangle;
	return (1.0 - x)*(1.0 - y)*zBedProbePoints[0] + x*(1.0 - y)*zBedProbePoints[3] + (1.0 - x)*y*zBedProbePoints[1] + x*y*zBedProbePoints[2];
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

// This is the function that's called by the timer interrupt to step the motors.
void Move::Interrupt()
{
	bool again = true;
	while (again && currentDda != nullptr)
	{
		again = currentDda->Step();
	}
}

bool Move::StartNextMove(uint32_t startTime)
{
	if (currentDda != nullptr)
	{
		for (size_t i = 0; i < DRIVES; ++i)
		{
			liveCoordinates[i] = currentDda->MachineToEndPoint(i);
		}
		currentDda->Release();
		currentDda = nullptr;
		ddaRingGetPointer = ddaRingGetPointer->GetNext();
	}

	if (ddaRingGetPointer->GetState() == DDA::frozen)
	{
		currentDda = ddaRingGetPointer;
		return currentDda->Start(startTime);
	}
	else
	{
		return false;
	}
}

// This is called from the step ISR. Any variables it modifies that are also read by code outside the ISR must be declared 'volatile'.
void Move::HitLowStop(size_t drive, DDA* hitDDA)
{
	hitDDA->MoveAborted();
	float hitPoint = reprap.GetPlatform()->AxisMinimum(drive);
	if(drive == Z_AXIS)
	{
		if(zProbing)
		{
			// Executing G32, so record the Z position at which we hit the end stop
			if (reprap.GetGCodes()->GetAxisIsHomed(drive))
			{
				// Z-axis has already been homed, so just record the height of the bed at this point
				lastZHit = hitDDA->MachineToEndPoint(drive) - reprap.GetPlatform()->ZProbeStopHeight();
				return;
			}
			else
			{
				// Z axis has not yet been homed, so treat this probe as a homing command
				lastZHit = 0.0;
				hitPoint = reprap.GetPlatform()->ZProbeStopHeight();
			}
		}
		else
		{
			// Executing G30, so set the current Z height to the value at which the end stop is triggered
			// Transform it first so that the height is correct in user coordinates
			float xyzPoint[DRIVES + 1];
			LiveCoordinates(xyzPoint);
			xyzPoint[Z_AXIS] = lastZHit = reprap.GetPlatform()->ZProbeStopHeight();
			Transform(xyzPoint);
			hitPoint = xyzPoint[Z_AXIS];
		}
	}
	int32_t coord = EndPointToMachine(drive, hitPoint);
	hitDDA->SetDriveCoordinate(coord, drive);
	const int32_t* pos = hitDDA->MachineCoordinates();
	for (size_t i = 0; i < AXES; ++i)
	{
		nextMachineEndPoints[i] = pos[i];
	}
	reprap.GetGCodes()->SetAxisIsHomed(drive);
}

// This is called from the step ISR. Any variables it modifies that are also read by code outside the ISR must be declared 'volatile'.
void Move::HitHighStop(size_t drive, DDA* hitDDA)
{
	hitDDA->MoveAborted();
	int32_t coord = EndPointToMachine(drive, reprap.GetPlatform()->AxisMaximum(drive));
	hitDDA->SetDriveCoordinate(coord, drive);
	const int32_t* pos = hitDDA->MachineCoordinates();
	for (size_t i = 0; i < AXES; ++i)
	{
		nextMachineEndPoints[i] = pos[i];
	}
	reprap.GetGCodes()->SetAxisIsHomed(drive);
}

// Return the untransformed machine coordinates
void Move::GetCurrentMachinePosition(float m[]) const
{
	for (size_t i = 0; i < DRIVES; i++)
	{
		if (i < AXES)
		{
			m[i] = ((float)nextMachineEndPoints[i])/reprap.GetPlatform()->DriveStepsPerUnit(i);
		}
		else
		{
			m[i] = 0.0; //FIXME This resets extruders to 0.0, even the inactive ones (is this behaviour desired?)
			//m[i] = lastMove->MachineToEndPoint(i); //FIXME TEST alternative that does not reset extruders to 0
		}
	}
	m[DRIVES] = currentFeedrate;
}

// Return the transformed machine coordinates
void Move::GetCurrentUserPosition(float m[]) const
{
	GetCurrentMachinePosition(m);
	InverseTransform(m);
}

void Move::SetXBedProbePoint(int index, float x)
{
	if(index < 0 || index >= NUMBER_OF_PROBE_POINTS)
	{
		reprap.GetPlatform()->Message(BOTH_MESSAGE, "Z probe point  X index out of range.\n");
		return;
	}
	xBedProbePoints[index] = x;
	probePointSet[index] |= xSet;
}

void Move::SetYBedProbePoint(int index, float y)
{
	if(index < 0 || index >= NUMBER_OF_PROBE_POINTS)
	{
		reprap.GetPlatform()->Message(BOTH_MESSAGE, "Z probe point Y index out of range.\n");
		return;
	}
	yBedProbePoints[index] = y;
	probePointSet[index] |= ySet;
}

void Move::SetZBedProbePoint(int index, float z)
{
	if(index < 0 || index >= NUMBER_OF_PROBE_POINTS)
	{
		reprap.GetPlatform()->Message(BOTH_MESSAGE, "Z probe point Z index out of range.\n");
		return;
	}
	zBedProbePoints[index] = z;
	probePointSet[index] |= zSet;
}

float Move::XBedProbePoint(int index) const
{
	return xBedProbePoints[index];
}

float Move::YBedProbePoint(int index) const
{
	return yBedProbePoints[index];
}

float Move::ZBedProbePoint(int index) const
{
	return zBedProbePoints[index];
}

void Move::SetZProbing(bool probing)
{
	zProbing = probing;
}

float Move::GetLastProbedZ() const
{
	return lastZHit;
}

bool Move::AllProbeCoordinatesSet(int index) const
{
	return probePointSet[index] == (xSet | ySet | zSet);
}

bool Move::XYProbeCoordinatesSet(int index) const
{
	return (probePointSet[index]  & xSet) && (probePointSet[index]  & ySet);
}

int Move::NumberOfProbePoints() const
{
	for(int i = 0; i < NUMBER_OF_PROBE_POINTS; i++)
	{
		if(!AllProbeCoordinatesSet(i))
		{
			return i;
		}
	}
	return NUMBER_OF_PROBE_POINTS;
}

int Move::NumberOfXYProbePoints() const
{
	for(int i = 0; i < NUMBER_OF_PROBE_POINTS; i++)
	{
		if(!XYProbeCoordinatesSet(i))
		{
			return i;
		}
	}
	return NUMBER_OF_PROBE_POINTS;
}

// For debugging
void Move::PrintCurrentDda() const
{
	if (currentDda != nullptr)
	{
		currentDda->DebugPrint();
		reprap.GetPlatform()->GetLine()->Flush();
	}
}

// End
