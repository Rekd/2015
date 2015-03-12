/*
 * LiftSystem.cpp
 *
 *  Created on: Feb 15: 2015
 *      Author: mll
 */

#include "LiftSystem.h"

LiftSystem::LiftSystem(CANSpeedController *pForkMotor, CANSpeedController *pLiftMotor, CANSpeedController *pLeftIntakeMotor, CANSpeedController *pRightIntakeMotor,
		DigitalInput *pForkLimitInner, DigitalInput *pForkLimitOuter, DigitalInput *pLiftLimitLow, DigitalInput *pLiftLimitHigh,
		Joystick *pLiftSysJoystick)
{
	//assign objects
	forkMotor = pForkMotor;
	liftMotor = pLiftMotor;
	leftIntakeMotor = pLeftIntakeMotor;
	rightIntakeMotor = pRightIntakeMotor;
	forkLimitInner = pForkLimitInner;
	forkLimitOuter = pForkLimitOuter;
	liftLimitLow = pLiftLimitLow;
	liftLimitHigh = pLiftLimitHigh;
	liftSysJoystick = pLiftSysJoystick;

	//initialize local variables
	liftSpeed = ZERO_FL; //start with the lift no moving
	TurnIntakesOff(); //this initializes intakesOn
}

LiftSystem::~LiftSystem()
{
}

//Limit Switches:
bool LiftSystem::GetForkLimitSwitchInner()
{
	//invert so that TRUE when limit switch is closed and FALSE when limit switch is open
	return !(forkLimitInner->Get());
}

bool LiftSystem::GetForkLimitSwitchOuter()
{
	//invert so that TRUE when limit switch is closed and FALSE when limit switch is open
	return !(forkLimitOuter->Get());
}

bool LiftSystem::GetLiftLimitSwitchLow()
{
	//invert so that TRUE when limit switch is closed and FALSE when limit switch is open
	return !(liftLimitLow->Get());
}

bool LiftSystem::GetLiftLimitSwitchHigh()
{
	//invert so that TRUE when limit switch is closed and FALSE when limit switch is open
	return !(liftLimitHigh->Get());
}

//Motors
void LiftSystem::SetForkMotor(float val)
{
	forkMotor->Set(FORK_MOTOR_REV_STATE*val);
}

void LiftSystem::SetLiftMotor(float val)
{
	liftMotor->Set(LIFT_MOTOR_REV_STATE*val);
}

bool  LiftSystem::CheckForkMotorCurrentSpike()
{
	if (forkMotor->GetOutputCurrent() > FORK_CURRENT_LIMIT)
		return(true);
	else
		return(false);
}

bool LiftSystem::CheckInakeMotorsCurrentSpike()
{
	if ((leftIntakeMotor->GetOutputCurrent() > INTAKE_CURRENT_LIMIT) || (rightIntakeMotor->GetOutputCurrent() > INTAKE_CURRENT_LIMIT))
		return(true);
	else
		return(false);
}

bool LiftSystem::IsButtonPressed(int button)
{
	return(liftSysJoystick->GetRawButton(button));
}

void LiftSystem::TurnIntakesOn()
{
	leftIntakeMotor->Set(LEFT_INTAKE_MOTOR_REV_STATE*INTAKE_MOTOR_SPEED);
	rightIntakeMotor->Set(RIGHT_INTAKE_MOTOR_REV_STATE*INTAKE_MOTOR_SPEED);
	intakesOn = true;
}

void LiftSystem::TurnIntakesOff()
{
	leftIntakeMotor->Set(MOTOR_STOP);
	rightIntakeMotor->Set(MOTOR_STOP);
	intakesOn = false;
}

//Main
void LiftSystem::Update()
{
	//intakes
	if(IsButtonPressed(INTAKE_BUTTON))
	{
		intakesOn = !intakesOn; //(off to on) or (on to off)
		if(intakesOn)
			TurnIntakesOn();
		else
			TurnIntakesOff();
	}
	if(CheckInakeMotorsCurrentSpike())
		TurnIntakesOff();

	//forks
	//
	//manual control
	if (liftSysJoystick->GetRawButton(BUT_JS_OUT) && !GetForkLimitSwitchOuter())  // move outwards
		SetForkMotor(FORK_MOTOR_SPEED_OUT);
	else if(liftSysJoystick->GetRawButton(BUT_JS_IN) && !GetForkLimitSwitchInner())  // move inwards
		SetForkMotor(-FORK_MOTOR_SPEED_IN);
	else
		SetForkMotor(MOTOR_STOP); //stop
	//check for current spike
	if (CheckForkMotorCurrentSpike())
		SetForkMotor(MOTOR_STOP);

	//lift
	//
	//either joystick direction moves the lift upward; can not actively move the lift down
	//not moving the lift upwards moves it down by gravity
	liftSpeed = abs(liftSysJoystick->GetY()) * LIFT_MOTOR_SPEED_UP;
	//Filter deadband
	if (liftSpeed < LIFT_DB_HIGH)
		liftSpeed = ZERO_FL;
	//manual control
	if ((liftSpeed > ZERO_FL) && !GetLiftLimitSwitchHigh())  // move up
		SetLiftMotor(liftSpeed);
	else
		SetLiftMotor(MOTOR_STOP); //stop
}
