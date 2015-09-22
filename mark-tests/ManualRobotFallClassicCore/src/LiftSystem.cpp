/*
 * LiftSystem.cpp
 *
 *  Created on: Feb 15: 2015
 *      Author: mll
 */

#include "LiftSystem.h"

LiftSystem::LiftSystem(CANSpeedController *pLiftMotorBack, CANSpeedController *pLiftMotorFront, CANSpeedController *pLeftIntakeMotor, CANSpeedController *pRightIntakeMotor,
		DigitalInput *pLiftLimitLow, DigitalInput *pLiftLimitHigh,
		Encoder *pLiftEncoder, PIDController *pControlLiftBack, PIDController *pControlLiftFront,
		Joystick *pLiftSysJoystick)
{
	//assign objects
	liftMotorBack = pLiftMotorBack;
	liftMotorFront = pLiftMotorFront;
	leftIntakeMotor = pLeftIntakeMotor;
	rightIntakeMotor = pRightIntakeMotor;
	liftLimitLow = pLiftLimitLow;
	liftLimitHigh = pLiftLimitHigh;
	liftEncoder = pLiftEncoder;
	controlLiftBack = pControlLiftBack;
	controlLiftFront = pControlLiftFront;
	liftSysJoystick = pLiftSysJoystick;

	//initialize local variables
	liftDir = ZERO_FL; //lift stopped on initialization
	liftPidOn = false; //lift pid is off at initialization
	IntakesOff(); //intakes off at initialization
	atTop = atTopHold = false;
}

LiftSystem::~LiftSystem()
{
}

//Limit Switches
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
void LiftSystem::SetLiftMotor(float val)
{
	liftMotorBack->Set(LIFT_MOTOR_REV_STATE*val);
	liftMotorFront->Set(LIFT_MOTOR_REV_STATE*val);
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

void LiftSystem::IntakesIn()
{
	leftIntakeMotor->Set(LEFT_INTAKE_MOTOR_REV_STATE*INTAKE_IN_MOTOR_SPEED);
	rightIntakeMotor->Set(RIGHT_INTAKE_MOTOR_REV_STATE*INTAKE_IN_MOTOR_SPEED);
}

void LiftSystem::IntakesOut()
{
	leftIntakeMotor->Set(LEFT_INTAKE_MOTOR_REV_STATE*INTAKE_OUT_MOTOR_SPEED);
	rightIntakeMotor->Set(RIGHT_INTAKE_MOTOR_REV_STATE*INTAKE_OUT_MOTOR_SPEED);
}

void LiftSystem::IntakesOff()
{
	leftIntakeMotor->Set(MOTOR_STOP);
	rightIntakeMotor->Set(MOTOR_STOP);
}

//Main
void LiftSystem::Update()
{
	char myString[64]; //for debugging
	//intakes
	if(IsButtonPressed(INTAKES_IN_BUTTON))
		IntakesIn();
	if(IsButtonPressed(INTAKES_OUT_BUTTON))
		IntakesOut();
	if(IsButtonPressed(INTAKES_OFF_BUTTON) || CheckInakeMotorsCurrentSpike())
		IntakesOff();

	//lift
	//
	//prevent bouncing at the top
	if(GetLiftLimitSwitchHigh())
		atTop = true;
	//move the lift away from the top
	if(atTop)
	{
		sprintf(myString, "lift stopped at ul\n");
		SmartDashboard::PutString("DB/String 0", myString);
		curLiftPos = liftEncoder->GetDistance();
		targetLiftPos = curLiftPos - HALF_INCH_OFFSET;
		controlLiftBack->Enable();
		controlLiftBack->SetSetpoint(targetLiftPos);
		controlLiftFront->Enable();
		controlLiftFront->SetSetpoint(targetLiftPos);
		atTop = false;
		atTopHold = true;
	}

	liftDir = liftSysJoystick->GetY();

	if(atTopHold) //require a lift down command from the lift joystick before allowing the lift to move out of hold
	{
		if(liftDir < 0.0) //exit lift hold condition
		{
			controlLiftBack->Disable();
			controlLiftFront->Disable();
			atTopHold = false;
		}
	}
	else
	{
		//Filter deadband
		if ((liftDir > LIFT_DB_LOW) && (liftDir < LIFT_DB_HIGH)) //in the deadband so hold current position
		{
			if(!liftPidOn)
			{
				curLiftPos = liftEncoder->GetDistance();
				controlLiftBack->Enable();
				controlLiftBack->SetSetpoint(curLiftPos);
				controlLiftFront->Enable();
				controlLiftFront->SetSetpoint(curLiftPos);
				liftPidOn = true;
			}
		}
		else //not in the deadband so do not hold current position
		{
			if(liftPidOn)
			{
				controlLiftBack->Disable();
				controlLiftFront->Disable();
				liftPidOn = false;
			}

			//manual control
			if ((liftDir > ZERO_FL) && !GetLiftLimitSwitchHigh() && !atTop)  // move up
			{
				SetLiftMotor(LIFT_MOTOR_REV_STATE*LIFT_MOTOR_SPEED_UP);
				sprintf(myString, "lift moving up\n");
				SmartDashboard::PutString("DB/String 0", myString);
			}

			else if((liftDir < ZERO_FL) && !GetLiftLimitSwitchLow())  // move down
			{
				SetLiftMotor(LIFT_MOTOR_REV_STATE*LIFT_MOTOR_SPEED_DOWN); //fixed lift down speed
				sprintf(myString, "lift moving down\n");
				SmartDashboard::PutString("DB/String 0", myString);
			}
			else
			{
				SetLiftMotor(MOTOR_STOP); //stop
				sprintf(myString, "lift stopped at ll\n");
				SmartDashboard::PutString("DB/String 0", myString);
			}
		}
	}
}
