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
	float startSpeed;
	float topSpeed;
	float accelStopDistance;
	float decelStartDistance;
	float accel;
	float recipAccel;
	float mmPerStep;
	uint32_t totalSteps;
	uint32_t stepsDone;
	float nextStepTime;
	bool moving;
	bool direction;				// Forwards or backwards?

	void InitNotMoving();
	void InitMoving(int32_t delta, float dv, float acc, float driveStepsPerMm);
	void Prepare(float dv, float accelDist, float startDecelDist, float moveStartSpeed, float moveTopSpeed);
	uint32_t CalcNextStepTime(float accelStopTime, float decelStartTime);
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
		empty,				// empty of being filled in
		ready,				// ready, but could be subject to modifications
		executing			// steps are currently being generated for this DDA
	};

	DDA(DDA* n);

	void Init(int32_t ep[], float distanceMoved, float reqSpeed, float acc,	// Set up this move
			const float *directionVector, EndstopChecks ce, const DDA *limitDda);
	void Init();													// Set up initial positions for machine startup
	void Start();													// Start executing the DDA, i.e. move the move.
	void Step();													// Take one step of the DDA, called by timed interrupt.
	void SetNext(DDA *n) { next = n; }
	void SetPrevious(DDA *p) { prev = p; }

	DDAState GetState() const { return state; }
	DDA* GetNext() const { return next; }
	DDA* GetPrevious() const { return prev; }
	uint32_t GetTimeNeeded() const { return timeNeeded; }
	uint32_t GetTimeLeft() const { return 0; }						//TODO implement this properly
	float MachineToEndPoint(int8_t drive) const;					// Convert a move endpoint to real mm coordinates
	const int32_t *MachineCoordinates() const { return endPoint; }	// Get endpoints of a move in machine coordinates

private:
	static const uint32_t minInterruptInterval = F_CPU/100000UL;	// 10us minimum interval between interrupts, in clocks

	float AdjustEndSpeed(float idealStartSpeed, const float *directionVector);	// adjust the end speed to match the following move
	void ScheduleInterrupt(uint32_t time);
	void MoveComplete();

	DDA* next;								// The next one in the ring
	DDA *prev;								// The previous one in the ring
	volatile DDAState state;				// what state this DDA is in
	uint32_t timeNeeded;
	int32_t endPoint[DRIVES];  				// Machine coordinates of the endpoint
	EndstopChecks endStopsToCheck;			// Which endstops we are checking on this move
    float totalDistance;					// How long is the move in real distance
    float requestedSpeed;
	float acceleration;						// The acceleration to use
	float idealStartSpeed;
	float accelStopTime;
	float decelStartTime;
	float totalTime;
	bool startPinned;

	DriveMovement ddm[DRIVES];
};

#endif /* DDA_H_ */
