/*
 * LiftSystem.h
 *
 *  Created on: Feb 15, 2015
 *      Author: mll
 */

#ifndef LIFTSYSTEM_H_
#define LIFTSYSTEM_H_

#include "WPILib.h"
#include "Constants.h"

class LiftSystem {
public:
	LiftSystem(CANSpeedController *pForkMotor, CANSpeedController *pLiftMotorBack, CANSpeedController *pLiftMotorFront, CANSpeedController *pLeftIntakeMotor, CANSpeedController *pRightIntakeMotor,
			DigitalInput *pForkLimitInner, DigitalInput *pForkLimitOuter, DigitalInput *pLiftLimitLow, DigitalInput *pLiftLimitHigh,
			Joystick *pLiftSysJoystick);
	virtual ~LiftSystem();
	void Update();

private:
// local motors, switches and joysticks
	CANSpeedController	*forkMotor, *liftMotorBack, *liftMotorFront, *leftIntakeMotor, *rightIntakeMotor;
	DigitalInput *forkLimitInner, *forkLimitOuter, *liftLimitLow, *liftLimitHigh;
	Joystick    *liftSysJoystick;

// other private variables
	float liftDir; //value from the joystick to determine the lift direction (i.e. forward or backwards or stopped)
	bool intakesOn;
	bool forksIn;
	bool forksOut;

	bool atTop;
	struct timespec startTimeAtTop;
	struct timespec curTime;
	struct timespec timeDiff;
	double timeDiffInSec;

	//prototypes
	bool GetForkLimitSwitchInner();
	bool GetForkLimitSwitchOuter();
	bool GetLiftLimitSwitchLow();
	bool GetLiftLimitSwitchHigh();
	void SetForkMotor(float val);
	void SetLiftMotor(float val);
	bool CheckForkMotorCurrentSpike();
	bool CheckInakeMotorsCurrentSpike();
	bool IsButtonPressed(int button);
	void TurnIntakesOn();
	void TurnIntakesOff();
};

#endif /* LIFTSYSTEM_H_ */
