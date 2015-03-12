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
	LiftSystem(CANSpeedController *pForkMotor, CANSpeedController *pLiftMotor, CANSpeedController *pLeftIntakeMotor, CANSpeedController *pRightIntakeMotor,
			DigitalInput *pForkLimitInner, DigitalInput *pForkLimitOuter, DigitalInput *pLiftLimitLow, DigitalInput *pLiftLimitHigh,
			Joystick *pLiftSysJoystick);
	virtual ~LiftSystem();
	void Update();

private:
// local motors, switches and joysticks
	CANSpeedController	*forkMotor, *liftMotor, *leftIntakeMotor, *rightIntakeMotor;
	DigitalInput *forkLimitInner, *forkLimitOuter, *liftLimitLow, *liftLimitHigh;
	Joystick    *liftSysJoystick;

// other private variables
	float liftSpeed;
	bool intakesOn;

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
