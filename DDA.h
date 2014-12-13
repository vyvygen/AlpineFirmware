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
	float mmPerStep;			// the distance we move in hypercuboid space per step of this drive
	float elasticComp;			// the amount of elasticity compensation to apply
	uint32_t totalSteps;		// total number of steps for this move
	uint32_t nextStep;			// number of steps already done
	uint32_t nextStepTime;		// how many clocks after the start of this move the next step is due
	bool moving;				// true if this drive moves in this move, if false then all other values are don't cares
	bool direction;				// forwards or backwards?
};

/**
 * This defines a single linear movement of the print head
 */
class DDA
{
	friend class Move;		// TODO eliminate this

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
		ready,				// ready, but could be subject to modifications
		executing,			// steps are currently being generated for this DDA
		completed			// move has been completed or aborted
	};

	DDA(DDA* n);

	void Init(int32_t ep[], float distanceMoved, float reqSpeed, float acc,	// Set up this move
			const float *directionVector, EndstopChecks ce, const DDA *limitDda);
	void Init();													// Set up initial positions for machine startup
	bool Start(uint32_t tim);										// Start executing the DDA, i.e. move the move.
	bool Step();													// Take one step of the DDA, called by timed interrupt.
	void SetNext(DDA *n) { next = n; }
	void SetPrevious(DDA *p) { prev = p; }
	void Release() { state = empty; }

	DDAState GetState() const { return state; }
	DDA* GetNext() const { return next; }
	DDA* GetPrevious() const { return prev; }
	uint32_t GetTimeNeeded() const { return timeNeeded; }
	uint32_t GetTimeLeft() const { return 0; }						//TODO implement this properly
	float MachineToEndPoint(size_t drive) const;					// Convert a move endpoint to real mm coordinates
	static int32_t EndPointToMachine(size_t drive, float coord);
	const int32_t *MachineCoordinates() const { return endPoint; }	// Get endpoints of a move in machine coordinates
	void MoveAborted();
	void SetDriveCoordinate(int32_t a, size_t drive);				// Force an end point
	void SetFeedRate(float rate) { requestedSpeed = rate; }

private:
	static const uint32_t stepClockRate = VARIANT_MCK/2;			// the frequency of the clock used for stepper pulse timing
	static const uint32_t minInterruptInterval = stepClockRate/500000u;		// 2us minimum interval between interrupts, in clocks
	static const uint32_t settleClocks = stepClockRate/50;			// settling time after hitting an endstop (20ms)

	float AdjustEndSpeed(float idealStartSpeed, const float *directionVector);	// adjust the end speed to match the following move
	uint32_t CalcNextStepTime(DriveMovement& dm);

	DDA* next;								// The next one in the ring
	DDA *prev;								// The previous one in the ring
	volatile DDAState state;				// what state this DDA is in

	// These remain the same regardless of how we execute a move
	int32_t endPoint[DRIVES];  				// Machine coordinates of the endpoint
	EndstopChecks endStopsToCheck;			// Which endstops we are checking on this move
    float totalDistance;					// How long is the move in hypercuboid distance
	float acceleration;						// The acceleration to use
	float recipAccel;
    float requestedSpeed;

    // These vary depending on how we connect the move with its predecessor and successor, but remain constant while the move is being executed
	float topSpeed;
//	float idealStartSpeed;
	float startSpeed;
	float accelStopTime;
	float decelStartTime;
	float accelStopDistance;
	float decelStartDistance;
	float totalTime;
	uint32_t timeNeeded;
	uint32_t moveStartTime;
	uint32_t moveCompletedTime;

	DriveMovement ddm[DRIVES];				// These describe the state of each drive movement

	bool startPinned;
};

#endif /* DDA_H_ */
