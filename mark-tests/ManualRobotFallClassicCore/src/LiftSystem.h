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
	LiftSystem(CANSpeedController *pLiftMotorBack, CANSpeedController *pLiftMotorFront, CANSpeedController *pLeftIntakeMotor, CANSpeedController *pRightIntakeMotor,
			DigitalInput *pLiftLimitLow, DigitalInput *pLiftLimitHigh,
			Encoder *pLiftEncoder, PIDController *pControlLiftBack, PIDController *pControlLiftFront,
			Joystick *pLiftSysJoystick);
	virtual ~LiftSystem();
	void Update();


private:
// local motors, switches and joysticks
	CANSpeedController	*liftMotorBack, *liftMotorFront, *leftIntakeMotor, *rightIntakeMotor;
	DigitalInput *liftLimitLow, *liftLimitHigh;
	Encoder *liftEncoder;
	PIDController *controlLiftBack, *controlLiftFront;
	Joystick    *liftSysJoystick;

// other private variables
	float liftDir; //value from the joystick to determine the lift direction (i.e. forward or backwards or stopped)
	float curLiftPos; //current lift encoder position
	float targetLiftPos;
	bool liftPidOn;

	bool atTop, atTopHold; //atTop indicates the lift has hit the upper limit switch; atTopHold indicates that automatic action was taken to move the lift away from the top and hold it in a safe state (i.e. to prevent repeated hits)

	//prototypes
	float DistToSetpoint();

	void SetLiftMotor(float val);
	bool CheckInakeMotorsCurrentSpike();
	bool IsButtonPressed(int button);
	bool GetLiftLimitSwitchLow();
	bool GetLiftLimitSwitchHigh();
	void IntakesIn();
	void IntakesOut();
	void IntakesOff();
};

#endif /* LIFTSYSTEM_H_ */
