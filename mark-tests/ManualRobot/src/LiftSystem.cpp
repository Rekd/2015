/*
 * LiftSystem.cpp
 *
 *  Created on: Feb 15: 2015
 *      Author: mll
 */

#include "LiftSystem.h"

//for debugging
char myString[64];
//sprintf(myString, "liftDir: %f\n", liftDir);
//SmartDashboard::PutString("DB/String 0", myString);

LiftSystem::LiftSystem(CANSpeedController *pForkMotor, CANSpeedController *pLiftMotorBack, CANSpeedController *pLiftMotorFront, CANSpeedController *pLeftIntakeMotor, CANSpeedController *pRightIntakeMotor,
		DigitalInput *pForkLimitInner, DigitalInput *pForkLimitOuter, DigitalInput *pLiftLimitLow, DigitalInput *pLiftLimitHigh,
		Joystick *pLiftSysJoystick)
{
	//assign objects
	forkMotor = pForkMotor;
	liftMotorBack = pLiftMotorBack;
	liftMotorFront = pLiftMotorFront;
	leftIntakeMotor = pLeftIntakeMotor;
	rightIntakeMotor = pRightIntakeMotor;
	forkLimitInner = pForkLimitInner;
	forkLimitOuter = pForkLimitOuter;
	liftLimitLow = pLiftLimitLow;
	liftLimitHigh = pLiftLimitHigh;
	liftSysJoystick = pLiftSysJoystick;

	//initialize local variables
	liftDir = ZERO_FL; //lift stopped on initialization
	TurnIntakesOff(); //this initializes intakesOn
	forksIn = false;
	forksOut = false;
	atTop = false;
	//times are initialized when needed

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
	liftMotorBack->Set(LIFT_MOTOR_REV_STATE*val);
	liftMotorFront->Set(LIFT_MOTOR_REV_STATE*val);
}

bool  LiftSystem::CheckForkMotorCurrentSpike()
{
	sprintf(myString, "forkCur: %f\n", forkMotor->GetOutputCurrent());
	SmartDashboard::PutString("DB/String 0", myString);

	if (forkMotor->GetOutputCurrent() > FORK_CURRENT_LIMIT)
		return(true);
	else
		return(false);
}

bool LiftSystem::CheckInakeMotorsCurrentSpike()
{
	sprintf(myString, "lInCur: %f\n", leftIntakeMotor->GetOutputCurrent());
	SmartDashboard::PutString("DB/String 1", myString);
	sprintf(myString, "rInCur: %f\n", leftIntakeMotor->GetOutputCurrent());
	SmartDashboard::PutString("DB/String 2", myString);

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
	if(IsButtonPressed(INTAKES_ON_BUTTON))
	{
		TurnIntakesOn();
	}
	if(IsButtonPressed(INTAKES_OFF_BUTTON))
	{
		TurnIntakesOff();
	}
	if(CheckInakeMotorsCurrentSpike())
		TurnIntakesOff();

	//forks
	//
	//manual control
	//
	//button input
	if(IsButtonPressed(BUT_FORKS_IN))
	{
		forksIn = true;
		forksOut = false; //do not allow conflicting fork commands
		SetForkMotor(-FORK_MOTOR_SPEED_IN);
	}
	if(IsButtonPressed(BUT_FORKS_OUT))
	{
		forksOut = true;
		forksIn = false; //do not allow conflicting fork commands
		SetForkMotor(FORK_MOTOR_SPEED_OUT);
	}
	if(IsButtonPressed(BUT_FORKS_STOP))
	{
		forksOut = false;
		forksIn = false; //do not allow conflicting fork commands
		//motor set to stop below
	}

	if(!forksOut && !forksIn)
	{
		SetForkMotor(MOTOR_STOP);
	}
	//
	//limit switches
	if(GetForkLimitSwitchOuter())
	{
		forksOut = false;
	}
	if(GetForkLimitSwitchInner())
	{
		forksIn = false;
	}
	//
	//check for current spike
	if (CheckForkMotorCurrentSpike())
	{
		SetForkMotor(MOTOR_STOP);
		forksIn= false;
		forksOut = false;
	}

	//lift
	//
	//prevent bouncing at the top
	if(GetLiftLimitSwitchHigh())
	{
		atTop = true;
		clock_gettime(CLOCK_REALTIME, &startTimeAtTop);
	}
	//test if sufficient duration from trip of upper limit switch has passed
	if(atTop)
	{
		clock_gettime(CLOCK_REALTIME, &curTime);
		timeDiffInSec = (curTime.tv_sec - startTimeAtTop.tv_sec) + SEC_IN_NANOSEC*(curTime.tv_nsec - startTimeAtTop.tv_nsec);

		if(timeDiffInSec > AT_TOP_LIFT_DUR)
			atTop = false;
	}

	liftDir = liftSysJoystick->GetY();
	//Filter deadband
	if ((liftDir > LIFT_DB_LOW) && (liftDir < LIFT_DB_HIGH))
		liftDir = ZERO_FL;
	//manual control
	if ((liftDir > ZERO_FL) && !GetLiftLimitSwitchHigh() && !atTop)  // move up
		SetLiftMotor(LIFT_MOTOR_SPEED_UP);
	else if((liftDir < ZERO_FL) && !GetLiftLimitSwitchLow())  // move down
		SetLiftMotor(-LIFT_MOTOR_SPEED_DOWN);
	else
		SetLiftMotor(MOTOR_STOP); //stop
}
