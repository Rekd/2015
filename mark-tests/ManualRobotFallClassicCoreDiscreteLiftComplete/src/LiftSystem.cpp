/*
 * LiftSystem.cpp
 *
 *  Created on: Feb 15: 2015
 *      Author: mll
 */

#include "LiftSystem.h"

//public

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
	liftInitDone = false; //lift does not start out initialized when LiftSystem is created
	liftFailure = false;
	//lift liftRefPos is set when lift initialization is complete
	//liftStateGoal is initialized at the end of LiftInit
	pickupInProgress = false;

	IntakesOff(); //intakes off when LiftSystem is created
}

LiftSystem::~LiftSystem()
{
}

bool LiftSystem::IsLiftInitDone()
{
	return liftInitDone;
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

void LiftSystem::setLiftStateLow()
{
	controlLiftBack->SetSetpoint(liftRefPos + LIFT_LOW_POS_OFFSET);
	controlLiftFront->SetSetpoint(liftRefPos + LIFT_LOW_POS_OFFSET);
	liftStateGoal = LOW;
	return;
}


void LiftSystem::setLiftStateStep()
{
	controlLiftBack->SetSetpoint(liftRefPos + LIFT_STEP_POS_OFFSET);
	controlLiftFront->SetSetpoint(liftRefPos + LIFT_STEP_POS_OFFSET);
	liftStateGoal = STEP;
}


void LiftSystem::setLiftStateHigh()
{
	controlLiftBack->SetSetpoint(liftRefPos + LIFT_HIGH_POS_OFFSET);
	controlLiftFront->SetSetpoint(liftRefPos + LIFT_HIGH_POS_OFFSET);
	liftStateGoal = HIGH;
}

void LiftSystem::setLiftStatePickup()
{
	setLiftStateLow();
	pickupInProgress = true;

}


void LiftSystem::Update()
{
	if(liftFailure)
	{
		//everything on the lift was already turned off when the liftFailure was detected
		return;
	}

	if(!liftInitDone) //initialize the position of the lift
	//lift pid is not on until lift initialization is complete
	{
		SetLiftMotor(LIFT_MOTOR_REV_STATE*LIFT_MOTOR_SPEED_DOWN); //fixed lift down speed

		//reached the bottom
		if(GetLiftLimitSwitchLow())
		{
			SetLiftMotor(MOTOR_STOP); //stop
			liftRefPos = (liftEncoder->GetDistance());
			liftInitDone = true;

			//enable lift pid and go to the low position
			setLiftStateLow();
			controlLiftBack->Enable();
			controlLiftFront->Enable();
		}
	}
	else //lift position has been initialized
	{
		//intakes
		if (IsButtonPressed(INTAKES_IN_BUTTON) && liftStateGoal==HIGH)
			IntakesIn();
		if(IsButtonPressed(INTAKES_OUT_BUTTON) && ((liftStateGoal==LOW) || (liftStateGoal==STEP)))
			IntakesOut();
		if(IsButtonPressed(INTAKES_OFF_BUTTON) || CheckIntakeMotorsCurrentSpike())
			IntakesOff();

		//lift
		if(GetLiftLimitSwitchHigh()) //major failure
		{
			liftFailure = true;

			//turn everything on the lift off
			controlLiftBack->Disable();
			controlLiftFront->Disable();
			SetLiftMotor(MOTOR_STOP);
			IntakesOff();

			return;
		}
		else //no major failure
		{
			if(!pickupInProgress) //pickup not in progress
			{
				if(IsButtonPressed(LIFT_LOW_POS_BUTTON) && !(liftStateGoal == STEP)) //do not allow lift motion down if the current liftStateGoal is the STEP
					setLiftStateLow();
				else if(IsButtonPressed(LIFT_STEP_POS_BUTTON))
					setLiftStateStep();
				else if(IsButtonPressed(LIFT_HIGH_POS_BUTTON))
					setLiftStateHigh();
				else if(IsButtonPressed(LIFT_PICKUP_BUTTON) && (liftStateGoal == HIGH) && !(liftStateGoal == STEP)) //do not allow lift motion down if the current liftStateGoal is the STEP
					setLiftStatePickup();
			}
			else //pickup in progress
			{
				if(liftStateGoal == LOW && (LiftOnTarget(controlLiftBack, liftEncoder) || LiftOnTarget(controlLiftFront, liftEncoder)))
					setLiftStateHigh();
				else if(liftStateGoal == HIGH && (LiftOnTarget(controlLiftBack, liftEncoder) || LiftOnTarget(controlLiftFront, liftEncoder)))
					pickupInProgress = false;
			} //end pickup check
		} //end initial lift failure identification check
	} //end lift initialization check
} //end Update()

//private

//function to determine if have reached a control setpoint (target)
//used by PID controllers for the lift
bool LiftSystem::LiftOnTarget(PIDController *controller, PIDSource *source)
{
	float error = controller->GetSetpoint() - source->PIDGet();
	return (fabs(error) < LIFT_PID_ERR_TOL);
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

bool LiftSystem::CheckIntakeMotorsCurrentSpike()
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


