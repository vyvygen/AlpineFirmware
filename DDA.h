/*
 * DDA.h
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#ifndef DDA_H_
#define DDA_H_

// This class describes a single movement of one drive
class DriveMovement
{
public:
	// These values don't depend on how the move is executed, so  are set by Init()
	float dv;									// proportion of total movement for this drive
	float stepsPerMm;							// steps per mm of movement in hypercuboid space
//	float mmPerStep;							// the distance we move in hypercuboid space per step of this drive
	float elasticComp;							// the amount of elasticity compensation to apply
	uint32_t totalSteps;						// total number of steps for this move
	uint64_t twoCsquaredTimesMmPerStepDivA;		// 2 * clock^2 * mmPerStepInHyperCuboidSpace / acceleration
	bool moving;								// true if this drive moves in this move, if false then all other values are don't cares
	bool direction;								// forwards or backwards?

	// These values depend on how the move is executed, so they are set by Prepare()
	uint32_t accelStopStep;						// the first step number at which we are no longer accelerating
	uint32_t decelStartStep;					// the first step number at which we are decelerating
	uint32_t mmPerStepTimesCdivtopSpeed;		// mmPerStepInHyperCuboidSpace * clock / topSpeed

	// These values change as the step is executed
	uint32_t nextStep;							// number of steps already done
	uint32_t nextStepTime;						// how many clocks after the start of this move the next step is due

	void DebugPrint(char c) const;

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

/**
 * This defines a single linear movement of the print head
 */
class DDA
{
//	friend class Move;		// TODO eliminate this

#if 0
	// Type of a DDA movement
	enum MovementType
	{
		unset = 0,
		accel = 1,
		decel = 2,
		accelDecel = 3,
		trapezoidal = 4
	};
#endif

public:

	enum DDAState
	{
		empty,				// empty or being filled in
		provisional,		// ready, but could be subject to modifications
		frozen,				// ready, no further modifications allowed
		executing,			// steps are currently being generated for this DDA
		completed			// move has been completed or aborted
	};

	DDA(DDA* n);

	bool Init(const float nextMove[], EndstopChecks ce);			// Set up a new move, returning true if it represents real movement
	void Init();													// Set up initial positions for machine startup
	bool Start(uint32_t tim);										// Start executing the DDA, i.e. move the move.
	bool Step();													// Take one step of the DDA, called by timed interrupt.
	void SetNext(DDA *n) { next = n; }
	void SetPrevious(DDA *p) { prev = p; }
	void Release() { state = empty; }
	void Prepare();													// Calculate all the values and freeze this DDA

	DDAState GetState() const { return state; }
	DDA* GetNext() const { return next; }
	DDA* GetPrevious() const { return prev; }
	int32_t GetTimeLeft() const;
	float MachineToEndPoint(size_t drive) const;					// Convert a move endpoint to real mm coordinates
	static int32_t EndPointToMachine(size_t drive, float coord);
	const int32_t *MachineCoordinates() const { return endPoint; }	// Get endpoints of a move in machine coordinates
	void MoveAborted();
	void SetDriveCoordinate(int32_t a, size_t drive);				// Force an end point
	void SetFeedRate(float rate) { requestedSpeed = rate; }
	void DebugPrint() const;

	static uint32_t isqrt(uint64_t num);

	static const uint32_t stepClockRate = VARIANT_MCK/32;			// the frequency of the clock used for stepper pulse timing (using TIMER_CLOCK3), about 0.38us resolution

private:
	static const uint32_t minInterruptInterval = 6;					// about 2us minimum interval between interrupts, in clocks
	static const uint32_t settleClocks = stepClockRate/50;			// settling time after hitting an endstop (20ms)

	float AdjustEndSpeed(float idealStartSpeed);					// adjust the end speed to match the following move
	uint32_t CalcNextStepTime(DriveMovement& dm);

	DDA* next;								// The next one in the ring
	DDA *prev;								// The previous one in the ring
	volatile DDAState state;				// what state this DDA is in

	// These remain the same regardless of how we execute a move
	int32_t endPoint[DRIVES];  				// Machine coordinates of the endpoint
	EndstopChecks endStopsToCheck;			// Which endstops we are checking on this move
    float totalDistance;					// How long is the move in hypercuboid distance
	float acceleration;						// The acceleration to use
    float requestedSpeed;

    // These vary depending on how we connect the move with its predecessor and successor, but remain constant while the move is being executed
	float startSpeed;
	float topSpeed;
	float accelDistance;
	float decelDistance;
	float accelStopTime;
	float decelStartTime;
	float totalTime;

	// These are calculated from the above and used in the ISR, so they are set up by Prepare()
	uint32_t startSpeedTimesCdivA;
	uint64_t startSpeedTimesCdivAsquared;
	uint32_t topSpeedTimesCdivAPlusDecelStartClocks;
	uint64_t topSpeedTimesCdivAsquaredPlusTwoDecelStartDistanceTimesCsquareddDivA;
	uint32_t accelClocksMinusAccelDistanceTimesCdivTopSpeed;
	uint32_t decelStartClocks;

	uint32_t timeNeeded;					// in clocks
	uint32_t moveStartTime;					// clock count at which the move was started
	uint32_t firstStepTime;					// in clocks, relative to the start of the move
	uint32_t moveCompletedTime;				// in clocks, relative to the start of the move

	DriveMovement ddm[DRIVES];				// These describe the state of each drive movement
};

// Return the number of clocks this DDA still needs to execute.
// This could be slightly negative, if the move is overdue for completion.
inline int32_t DDA::GetTimeLeft() const
//pre(state == executing || state == frozen || state == completed)
{
	return (state == completed) ? 0
			: (state == executing) ? (int32_t)(moveStartTime + timeNeeded - Platform::GetInterruptClocks())
			: (int32_t)timeNeeded;
}

#endif /* DDA_H_ */
