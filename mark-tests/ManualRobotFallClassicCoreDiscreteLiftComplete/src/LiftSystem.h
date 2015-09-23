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
	bool IsLiftInitDone();
	void Update();

private:
// local motors, switches and joysticks
	CANSpeedController	*liftMotorBack, *liftMotorFront, *leftIntakeMotor, *rightIntakeMotor;
	DigitalInput *liftLimitLow, *liftLimitHigh;
	Encoder *liftEncoder;
	PIDController *controlLiftBack, *controlLiftFront;
	Joystick    *liftSysJoystick;

// other private variables
	bool liftInitDone; //indication of whether the lift has completed its initialization (i.e.initialized the absolute encoder)
	bool liftFailure; //indication of whether the lift has had a major failure (i.e. a failure sufficient that the lift should not be moved)
	float liftRefPos; //lift reference encoder position (aka zero position)
	enum liftStates {LOW, STEP, HIGH};
	liftStates liftStateGoal; //the lift's current state goal (i.e. where the lift is currently commanded to be)
	bool pickupInProgress; //indication if a pickup action is in progress

	//prototypes
	bool LiftOnTarget(PIDController *controller, PIDSource *source);
	bool GetLiftLimitSwitchLow();
	bool GetLiftLimitSwitchHigh();
	void SetLiftMotor(float val);
	bool CheckInakeMotorsCurrentSpike();
	bool IsButtonPressed(int button);
	void IntakesIn();
	void IntakesOut();
	void IntakesOff();
};

#endif /* LIFTSYSTEM_H_ */
