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
			Encoder *pLiftEncoder, PIDController *pControlLiftBack, PIDController *pControlLiftFront,
			Joystick *pLiftSysJoystick);
	virtual ~LiftSystem();
	void Update();
	void StartForksInAuto();
	void StartForksOutAuto();
	bool CheckForksIn();
	void UpdateAuto();

private:
// local motors, switches and joysticks
	CANSpeedController	*forkMotor, *liftMotorBack, *liftMotorFront, *leftIntakeMotor, *rightIntakeMotor;
	DigitalInput *forkLimitInner, *forkLimitOuter, *liftLimitLow, *liftLimitHigh;
	Encoder *liftEncoder;
	PIDController *controlLiftBack, *controlLiftFront;
	Joystick    *liftSysJoystick;

// other private variables
	float liftDir; //value from the joystick to determine the lift direction (i.e. forward or backwards or stopped)
	float curLiftPos; //current lift encoder position
	float targetLiftPos;
	bool liftPidOn;

	bool intakesOn;

	bool forksIn;
	bool forksOut;

	bool atTop, atTopHold;
	struct timespec startTimeAtTop;
	struct timespec curTime;
	struct timespec timeDiff;
	double timeDiffInSec;

	//prototypes
	float DistToSetpoint();
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
