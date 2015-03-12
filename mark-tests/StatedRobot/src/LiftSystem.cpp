/*
 * LiftSystem.cpp
 *
 *  Created on: Feb 15: 2015
 *      Author: mll
 */

#include "LiftSystem.h"

LiftSystem::LiftSystem(CANSpeedController *pForkMotor, CANSpeedController *pLiftMotor, CANSpeedController *pLeftIntakeMotor, CANSpeedController *pRightIntakeMotor,
		Counter *pGearToothCounter, Encoder *pLiftEnc, PIDController *pLiftControl,
		DigitalInput *pForkLimitInner, DigitalInput *pForkLimitOuter, DigitalInput *pLiftLimitLow, DigitalInput *pLiftLimitHigh,
		Joystick *pOperatorBox)
{
	//assign objects
	forkMotor = pForkMotor;
	liftMotor = pLiftMotor;
	leftIntakeMotor = pLeftIntakeMotor;
	rightIntakeMotor = pRightIntakeMotor;
	gearToothCounter = pGearToothCounter;
	liftEnc = pLiftEnc;
	liftControl = pLiftControl;
	forkLimitInner = pForkLimitInner;
	forkLimitOuter = pForkLimitOuter;
	liftLimitLow = pLiftLimitLow;
	liftLimitHigh = pLiftLimitHigh;
	operatorBox = pOperatorBox;

	//initialize local variables
	targetForkGearCount = 0;
	targetLiftEncoderPos = 0.0;
	//forkDirection will be initialized when first used
	//curForkSetSpeed will be initialized when first used
	absGearToothCount = OPEN_NARROW_COUNT;
	curGearToothCount = 0;
	lastGearToothCount = 0;

	//initialize the robot state machine
	robotState = opened_narrow_GP;
	openedNarrowSS = narrow_idle;
	TurnIntakesOff();
}

LiftSystem::~LiftSystem()
{
	operatorBox->SetOutputs(NO_LEDS);
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

void LiftSystem::CheckAndHandleHaltPickupGantry()
{
	if(GetForkLimitSwitchOuter() || GetLiftLimitSwitchLow() || GetLiftLimitSwitchHigh())
	{
		sprintf(myString, "L or F fault\n");
		SmartDashboard::PutString("DB/String 3", myString);

		SetForkMotor(MOTOR_STOP);
		liftControl->Disable(); //stops PID and sets the output to 0
		robotState = halt_pickup_gantry;
	}
}

//Motors
void LiftSystem::SetForkMotor(float val)
{
	curForkSetSpeed = FORK_MOTOR_REV_STATE*val;
	forkMotor->Set(curForkSetSpeed);
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
	if ((leftIntakeMotor->GetOutputCurrent() > FORK_CURRENT_LIMIT) || (rightIntakeMotor->GetOutputCurrent() > FORK_CURRENT_LIMIT))
		return(true);
	else
		return(false);
}

void LiftSystem::UpdateGearToothCount()
//this function turns the relative gear tooth counter into an absolute counter
{
	if ((FORK_MOTOR_REV_STATE*curForkSetSpeed) < 0)  // MLL - to deal with reversed state
		forkDirection = inwards;
	else //curForkSetSpeed > 0; this function should not be called when curForkSpeed = 0, should on be called when the forks are moving;
		forkDirection = outwards;

	curGearToothCount = gearToothCounter->Get();
	absGearToothCount += forkDirection*(curGearToothCount-lastGearToothCount);
	lastGearToothCount = curGearToothCount;
}

void LiftSystem::SetForkTarget(int target)
{
	targetForkGearCount = target;
}

int LiftSystem::GetForkTarget()  // Added by MLL
{
	return(targetForkGearCount);
}

void LiftSystem::SetLiftTarget(float target)
{
	targetLiftEncoderPos = target;
	liftControl->SetSetpoint(target);
}

void LiftSystem::DisableController()
{
	liftControl->Disable();
}

void LiftSystem::EnableController()
{
	liftControl->Enable();
}

void LiftSystem::ResetLiftEncoder()
{
	liftEnc->Reset();
}

bool LiftSystem::CheckForkHasReachedTarget()
{
	if (abs(absGearToothCount - targetForkGearCount) <= FORK_POS_TOL)  // MLL Reversed sign
		return true;
	else
		return false;
}


float LiftSystem::DistToSetpoint()
{
	if(!(liftControl->IsEnabled()))
			return UNINIT_VAL;
	else
		return ((float)(liftEnc->GetDistance()) - (float)(liftControl->GetSetpoint()));
}


bool LiftSystem::CheckLiftHasReachedTarget()
{
	if(!(liftControl->IsEnabled()))
		return false;
	else
		return (abs(DistToSetpoint()) < LIFT_POS_TOL*LIFT_ENCODER_DIST_PER_PULSE);
}

bool LiftSystem::IsButtonPressed(int button)
{
	return(operatorBox->GetRawButton(button));
}

void LiftSystem::TurnIntakesOn()
{
	leftIntakeMotor->Set(LEFT_INTAKE_MOTOR_REV_STATE*INTAKE_MOTOR_SPEED);
	rightIntakeMotor->Set(RIGHT_INTAKE_MOTOR_REV_STATE*INTAKE_MOTOR_SPEED);
	intakesOn = true;
	TurnLedOn(INTAKE_LED);
}

void LiftSystem::TurnIntakesOff()
{
	leftIntakeMotor->Set(MOTOR_STOP);
	rightIntakeMotor->Set(MOTOR_STOP);
	intakesOn = false;
	TurnLedOff(INTAKE_LED);
}

void LiftSystem::TurnLedOn(int led)
{
	operatorBox->SetOutput(led, true);
}

void LiftSystem::TurnLedOff(int led)
{
	operatorBox->SetOutput(led, false);
}

void LiftSystem::TurnForkLedsOff()
{
	TurnLedOff(OPEN_WIDE_LED);
	TurnLedOff(OPEN_NARROW_LED);
	TurnLedOff(CLOSE_LED);
	TurnLedOff(RELEASE_LED);
}

void LiftSystem::TurnLiftLedsOff()
{
	TurnLedOff(CARRY_ONE_LED);
	TurnLedOff(CARRY_TWO_LED);
	TurnLedOff(CARRY_THREE_LED);
	TurnLedOff(CARRY_STEP_LED);
}

void LiftSystem::MoveToResetState()
{
	SetForkMotor(FORK_MOTOR_IN_SPEED);
	SetLiftTarget(LARGE_NEG_POS);
	resetSS = movingToReset;
	robotState = reset;
}

void LiftSystem::MoveToKnownState()
{
	if ((robotState==reset) && (resetSS == atReset))
	{
		liftEnc->Reset();  // reset the encoder
		SetForkTarget(OPEN_NARROW_COUNT);
		SetForkMotor(FORK_MOTOR_IN_SPEED);
		SetLiftTarget(PICKUP_POS);
		resetSS = movingToPickupON;
	}
}

//Main
void LiftSystem::Update()
{

	sprintf(myString, "Fork Tgt: %d\n", GetForkTarget());
	SmartDashboard::PutString("DB/String 5", myString);
	sprintf(myString, "Curr F Pos: %d\n", absGearToothCount);
	SmartDashboard::PutString("DB/String 6", myString);
	sprintf(myString, "Gear Cnt: %d\n", curGearToothCount);
	SmartDashboard::PutString("DB/String 7", myString);

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


	//lift and fork
	switch (robotState)
	{
		case reset:
			sprintf(myString, "In Reset\n");
			SmartDashboard::PutString("DB/String 0", myString);
			switch (resetSS)
			{
				case movingToReset:
					if (CheckForkMotorCurrentSpike())
					{
						SetForkMotor(MOTOR_STOP);
						absGearToothCount = 0;
						targetForkGearCount = 0;
					}
					else if (GetForkLimitSwitchInner())
					{
						SetForkMotor(MOTOR_STOP);
						absGearToothCount = 0;
						targetForkGearCount = 0;
					}

					if (GetLiftLimitSwitchLow())
					{
						DisableController();
						ResetLiftEncoder();
						EnableController();
						SetLiftTarget(ZERO_FL);
					}

					if ((CheckForkHasReachedTarget()) && CheckLiftHasReachedTarget())
					{
						resetSS = atReset;
					}
					break;

				case atReset:
					MoveToKnownState();
					break;

				case movingToPickupON:
					if ((CheckForkHasReachedTarget()) && CheckLiftHasReachedTarget())
					{
						robotState = opened_narrow_GP;
						openedNarrowSS = narrow_idle;
					}
					break;
			}
			break;



		case opened_narrow_GP:
			sprintf(myString, "In opened narrow\n");
			SmartDashboard::PutString("DB/String 0", myString);
			TurnForkLedsOff();
			TurnLedOn(OPEN_NARROW_LED);
			TurnLiftLedsOff();

			switch (openedNarrowSS)
			{
				case narrow_idle:
					sprintf(myString, "In narrow_idle\n");
					SmartDashboard::PutString("DB/String 1", myString);
					if (IsButtonPressed(OPEN_WIDE_BUTTON))
					{
						sprintf(myString, "OW pressed\n");
						SmartDashboard::PutString("DB/String 2", myString);
						SetForkTarget(OPEN_WIDE_COUNT);
						SetForkMotor(FORK_MOTOR_OUT_SPEED);
						openedNarrowSS = opening_wide;
					}
					else if (IsButtonPressed(CLOSE_BUTTON))
					{
						sprintf(myString, "Close pressed\n");
						SmartDashboard::PutString("DB/String 2", myString);
						SetForkTarget(CLOSE_COUNT);
						SetForkMotor(FORK_MOTOR_IN_SPEED);
						openedNarrowSS = narrow_closing_fork;
					}
					break;
				case narrow_closing_fork:
					sprintf(myString, "In n_closing_f\n");
					SmartDashboard::PutString("DB/String 1", myString);
					UpdateGearToothCount(); //update whenever the fork is moving
					if (CheckForkMotorCurrentSpike())
					{
						SetForkMotor(MOTOR_STOP);
						SetLiftTarget(CARRY_ONE_POS);
						//do not need to set the lift target since PID sets this
						openedNarrowSS = narrow_changing_lift;
					}
					else if (GetForkLimitSwitchInner())
					{
						SetForkMotor(MOTOR_STOP);
						SetForkTarget(OPEN_NARROW_COUNT);
						SetForkMotor(FORK_MOTOR_OUT_SPEED);
						openedNarrowSS = narrow_error_recovery;
					}
					break;
				case opening_wide:
					sprintf(myString, "In opening_wide\n");
					SmartDashboard::PutString("DB/String 1", myString);
					UpdateGearToothCount(); //update whenever the fork is moving
					if (CheckForkHasReachedTarget())
					{
						sprintf(myString, "F @ targ: %d\n", absGearToothCount);
						SmartDashboard::PutString("DB/String 4", myString);
						SetForkMotor(MOTOR_STOP);  // stop fork motor
						robotState = opened_wide_GP;
						openedWideSS = wide_idle;
					}
					else
						CheckAndHandleHaltPickupGantry();
					break;
				case narrow_changing_lift:
					sprintf(myString, "In narrow_chng_l\n");
					SmartDashboard::PutString("DB/String 1", myString);
					if (CheckLiftHasReachedTarget())
					{
						sprintf(myString, "Lift @ Target\n");
						SmartDashboard::PutString("DB/String 3", myString);
						robotState = closed_C_Pos_One;
						closedSS = closed_idle;
					}
					else
					{
						sprintf(myString, "Lift NOT @ Target\n");
						SmartDashboard::PutString("DB/String 3", myString);
						CheckAndHandleHaltPickupGantry();
					}
					break;
				case narrow_error_recovery:
					sprintf(myString, "In narrow_err_rec\n");
					SmartDashboard::PutString("DB/String 1", myString);
					if (CheckForkHasReachedTarget())
					{
						sprintf(myString, "F @ targ: %d\n", absGearToothCount);
						SmartDashboard::PutString("DB/String 4", myString);
						SetForkMotor(MOTOR_STOP);
						openedNarrowSS = narrow_idle;
					}
					else
						CheckAndHandleHaltPickupGantry();
					break;
			}
			break;

		case opened_wide_GP:
			sprintf(myString, "In opened wide\n");
			SmartDashboard::PutString("DB/String 0", myString);

			TurnForkLedsOff();
			TurnLedOn(OPEN_WIDE_LED);
			TurnLiftLedsOff();

			switch (openedWideSS)
			{
				case wide_idle:
					if (IsButtonPressed(OPEN_NARROW_BUTTON))
					{
						sprintf(myString, "ON pressed\n");
						SmartDashboard::PutString("DB/String 2", myString);
						SetForkTarget(OPEN_NARROW_COUNT);
						SetForkMotor(FORK_MOTOR_IN_SPEED);
						openedWideSS = opening_narrow;
					}
					else if (IsButtonPressed(CLOSE_BUTTON))
					{
						sprintf(myString, "Close pressed\n");
						SmartDashboard::PutString("DB/String 2", myString);
						SetForkTarget(CLOSE_COUNT);
						SetForkMotor(FORK_MOTOR_IN_SPEED);
						openedWideSS = wide_closing_fork;
					}
					break;
				case wide_closing_fork:
					UpdateGearToothCount(); //update whenever the fork is moving
					if (CheckForkMotorCurrentSpike())
					{
						SetForkMotor(MOTOR_STOP);
						SetLiftTarget(CARRY_ONE_POS);
						//do not need to set the lift target since PID sets this
						openedWideSS = wide_changing_lift;
					}
					else if (GetForkLimitSwitchInner())
					{
						SetForkMotor(MOTOR_STOP);
						SetForkTarget(OPEN_WIDE_COUNT);
						SetForkMotor(FORK_MOTOR_OUT_SPEED);
						openedWideSS = wide_error_recovery;
					}
					break;
				case opening_narrow:
					UpdateGearToothCount(); //update whenever the fork is moving
					if(CheckForkHasReachedTarget())
					{
						sprintf(myString, "F @ targ: %d\n", absGearToothCount);
						SmartDashboard::PutString("DB/String 4", myString);
						SetForkMotor(MOTOR_STOP);
						robotState = opened_narrow_GP;
						openedNarrowSS = narrow_idle;
					}
					else if (GetForkLimitSwitchInner())
					{
						SetForkMotor(MOTOR_STOP);
						SetForkTarget(OPEN_WIDE_COUNT);
						//SetForkMotor(FORK_MOTOR_OUT_SPEED);
						openedWideSS = wide_error_recovery;
					}
					break;
				case wide_changing_lift:
					if (CheckLiftHasReachedTarget())
					{
						sprintf(myString, "Lift @ Target\n");
						SmartDashboard::PutString("DB/String 3", myString);
						robotState = closed_C_Pos_One;
						closedSS = closed_idle;
					}
					else
					{
						sprintf(myString, "Lift NOT @ Target\n");
						SmartDashboard::PutString("DB/String 3", myString);
						CheckAndHandleHaltPickupGantry();
					}
					break;
				case wide_error_recovery:
					if (CheckForkHasReachedTarget())
					{
						sprintf(myString, "F @ targ: %d\n", absGearToothCount);
						SmartDashboard::PutString("DB/String 4", myString);
						SetForkMotor(MOTOR_STOP);
						openedWideSS = wide_idle;
					}
					else
						CheckAndHandleHaltPickupGantry();
					break;
			}
			break;

		case closed_C_Pos_One:
			sprintf(myString, "In Carry 1\n");
			SmartDashboard::PutString("DB/String 0", myString);

			TurnForkLedsOff();
			TurnLedOn(CLOSE_LED);
			TurnLiftLedsOff();
			TurnLedOn(CARRY_ONE_LED);

			switch (closedSS)
			{
				case closed_idle:
					sprintf(myString, "In Carry 1 idl\n");
					SmartDashboard::PutString("DB/String 1", myString);
					if (IsButtonPressed(CARRY_TWO_BUTTON))
					{
						sprintf(myString, "C2 pressed\n");
						SmartDashboard::PutString("DB/String 2", myString);
						SetLiftTarget(CARRY_TWO_POS);
						//do not need to set the lift target since PID sets this
						closedSS = closed_changing_level;
					}
#if 0  // removed Carry 3
					else if (IsButtonPressed(CARRY_THREE_BUTTON))
					{
						SetLiftTarget(CARRY_THREE_POS);
						//do not need to set the lift target since PID sets this
						closedSS = closed_changing_level;
					}
#endif
					else if (IsButtonPressed(CARRY_STEP_BUTTON))
					{
						sprintf(myString, "CStep pressed\n");
						SmartDashboard::PutString("DB/String 2", myString);
						SetLiftTarget(CARRY_STEP_POS);
						//do not need to set the lift target since PID sets this
						closedSS = closed_changing_level;
					}
					else if (IsButtonPressed(RELEASE_BUTTON))
					{
						sprintf(myString, "REL pressed\n");
						SmartDashboard::PutString("DB/String 2", myString);
						SetLiftTarget(CARRY_ONE_POS - LIFT_OFFSET);
						//do not need to set the lift target since PID sets this
						closedSS = releasing_lowering;
					}
					break;
				case closed_changing_level:
					if(CheckLiftHasReachedTarget())
					{
						sprintf(myString, "Lift @ Target\n");
						SmartDashboard::PutString("DB/String 3", myString);
						if(abs(targetLiftEncoderPos-CARRY_TWO_POS) <= FLOAT_COMP_TOL)
							robotState = closed_C_Pos_Two;
#if 0 // removed carry 3
						else if(abs(targetLiftEncoderPos-CARRY_THREE_POS) <= FLOAT_COMP_TOL)
							robotState = closed_C_Pos_Three;
#endif
						else if(abs(targetLiftEncoderPos-CARRY_STEP_POS) <= FLOAT_COMP_TOL)
								robotState = closed_C_Pos_Step;
						closedSS = closed_idle;
					}
					else
					{
						sprintf(myString, "Lift NOT @ Target\n");
						SmartDashboard::PutString("DB/String 3", myString);
						CheckAndHandleHaltPickupGantry();
					}
					break;
				case releasing_lowering:
					if (CheckLiftHasReachedTarget())
					{
						sprintf(myString, "Lift @ Target\n");
						SmartDashboard::PutString("DB/String 3", myString);
						SetForkTarget(absGearToothCount + FORK_OFFSET);
						SetForkMotor(FORK_MOTOR_OUT_SPEED);
						closedSS = releasing_open_forks;
					}
					else
					{
						sprintf(myString, "Lift NOT @ Target\n");
						SmartDashboard::PutString("DB/String 3", myString);
						CheckAndHandleHaltPickupGantry();
					}
					break;
				case releasing_open_forks:
					UpdateGearToothCount(); //update whenever the fork is moving
					if (CheckForkHasReachedTarget())
					{
						sprintf(myString, "F @ targ: %d\n", absGearToothCount);
						SmartDashboard::PutString("DB/String 4", myString);
						SetForkMotor(MOTOR_STOP);
						robotState = released;
						releasedSS = released_idle;
					}
					else
						CheckAndHandleHaltPickupGantry();
					break;
			}
			break;

		case closed_C_Pos_Two:
			sprintf(myString, "In Carry 2\n");
			SmartDashboard::PutString("DB/String 0", myString);
			TurnForkLedsOff();
			TurnLedOn(CLOSE_LED);
			TurnLiftLedsOff();
			TurnLedOn(CARRY_TWO_LED);

			switch (closedSS)
			{
				case closed_idle:
					if (IsButtonPressed(CARRY_ONE_BUTTON))
					{
						sprintf(myString, "C1 pressed\n");
						SmartDashboard::PutString("DB/String 2", myString);
						SetLiftTarget(CARRY_ONE_POS);
						//do not need to set the lift target since PID sets this
						closedSS = closed_changing_level;
					}
#if 0 // remove carry 3
					else if (IsButtonPressed(CARRY_THREE_BUTTON))
					{
						SetLiftTarget(CARRY_THREE_POS);
						//do not need to set the lift target since PID sets this
						closedSS = closed_changing_level;
					}
#endif
					else if (IsButtonPressed(CARRY_STEP_BUTTON))
					{
						sprintf(myString, "CS pressed\n");
						SmartDashboard::PutString("DB/String 2", myString);
						SetLiftTarget(CARRY_STEP_POS);
						//do not need to set the lift target since PID sets this
						closedSS = closed_changing_level;
					}
					else if (IsButtonPressed(RELEASE_BUTTON))
					{
						sprintf(myString, "REL pressed\n");
						SmartDashboard::PutString("DB/String 2", myString);
						SetLiftTarget(CARRY_TWO_POS - LIFT_OFFSET);
						//do not need to set the lift target since PID sets this
						closedSS = releasing_lowering;
					}
					break;
				case closed_changing_level:
					if(CheckLiftHasReachedTarget())
					{
						sprintf(myString, "Lift @ Target\n");
						SmartDashboard::PutString("DB/String 3", myString);
						if(abs(targetLiftEncoderPos-CARRY_ONE_POS) <= FLOAT_COMP_TOL)
							robotState = closed_C_Pos_One;
#if 0 // removed carry 3
						else if(abs(targetLiftEncoderPos-CARRY_THREE_POS) <= FLOAT_COMP_TOL)
							robotState = closed_C_Pos_Three;
#endif
						else if(abs(targetLiftEncoderPos-CARRY_STEP_POS) <= FLOAT_COMP_TOL)
							robotState = closed_C_Pos_Step;
						closedSS = closed_idle;
					}
					else
					{
						sprintf(myString, "Lift NOT @ Target\n");
						SmartDashboard::PutString("DB/String 3", myString);
						CheckAndHandleHaltPickupGantry();
					}
					break;
				case releasing_lowering:
					if (CheckLiftHasReachedTarget())
					{
						sprintf(myString, "Lift @ Target\n");
						SmartDashboard::PutString("DB/String 3", myString);
						SetForkTarget(absGearToothCount + FORK_OFFSET);
						SetForkMotor(FORK_MOTOR_OUT_SPEED);
						closedSS = releasing_open_forks;
					}
					else
					{
						sprintf(myString, "Lift NOT @ Target\n");
						SmartDashboard::PutString("DB/String 3", myString);
						CheckAndHandleHaltPickupGantry();
					}
					break;
				case releasing_open_forks:
					UpdateGearToothCount(); //update whenever the fork is moving
					if (CheckForkHasReachedTarget())
					{
						sprintf(myString, "F @ targ: %d\n", absGearToothCount);
						SmartDashboard::PutString("DB/String 4", myString);
						SetForkMotor(MOTOR_STOP);
						robotState = released;
						releasedSS = released_idle;
					}
					else
						CheckAndHandleHaltPickupGantry();
					break;
			}
			break;
#if 0  // removed carry 3
		case closed_C_Pos_Three:
			TurnForkLedsOff();
			TurnLedOn(CLOSE_LED);
			TurnLiftLedsOff();
			TurnLedOn(CARRY_THREE_LED);

			switch (closedSS)
			{
			case closed_idle:
				if (IsButtonPressed(CARRY_ONE_BUTTON))
				{
					SetLiftTarget(CARRY_ONE_POS);
					//do not need to set the lift target since PID sets this
					closedSS = closed_changing_level;
				}
				else if (IsButtonPressed(CARRY_TWO_BUTTON))
				{
					SetLiftTarget(CARRY_TWO_POS);
					//do not need to set the lift target since PID sets this
					closedSS = closed_changing_level;
				}
				else if (IsButtonPressed(CARRY_STEP_BUTTON))
				{
					SetLiftTarget(CARRY_STEP_POS);
					//do not need to set the lift target since PID sets this
					closedSS = closed_changing_level;
				}
				else if (IsButtonPressed(RELEASE_BUTTON))
				{
					SetLiftTarget(CARRY_THREE_POS - LIFT_OFFSET);
					//do not need to set the lift target since PID sets this
					closedSS = releasing_lowering;
				}
				break;
			case closed_changing_level:
				if(CheckLiftHasReachedTarget())
				{
					if(abs(targetLiftEncoderPos-CARRY_ONE_POS) <= FLOAT_COMP_TOL)
						robotState = closed_C_Pos_One;
					else if(abs(targetLiftEncoderPos-CARRY_TWO_POS) <= FLOAT_COMP_TOL)
						robotState = closed_C_Pos_Two;
					else if(abs(targetLiftEncoderPos-CARRY_STEP_POS) <= FLOAT_COMP_TOL)
						robotState = closed_C_Pos_Step;
					closedSS = closed_idle;
				}
				else
					CheckAndHandleHaltPickupGantry();
				break;
			case releasing_lowering:
				if (CheckLiftHasReachedTarget())
				{
					SetForkTarget(absGearToothCount + FORK_OFFSET);
					SetForkMotor(FORK_MOTOR_OUT_SPEED);
					closedSS = releasing_open_forks;
				}
				else
					CheckAndHandleHaltPickupGantry();
				break;
			case releasing_open_forks:
				UpdateGearToothCount(); //update whenever the fork is moving
				if (CheckForkHasReachedTarget())
				{
					SetForkMotor(MOTOR_STOP);
					robotState = released;
					releasedSS = released_idle;
				}
				else
					CheckAndHandleHaltPickupGantry();
				break;
			}
			break;

		case closed_C_Pos_Three:
			TurnForkLedsOff();
			TurnLedOn(CLOSE_LED);
			TurnLiftLedsOff();
			TurnLedOn(CARRY_STEP_LED);

			switch (closedSS)
			{
			case closed_idle:
				if (IsButtonPressed(CARRY_ONE_BUTTON))
				{
					SetLiftTarget(CARRY_ONE_POS);
					//do not need to set the lift target since PID sets this
					closedSS = closed_changing_level;
				}
				else if (IsButtonPressed(CARRY_TWO_BUTTON))
				{
					SetLiftTarget(CARRY_TWO_POS);
					//do not need to set the lift target since PID sets this
					closedSS = closed_changing_level;
				}
				else if (IsButtonPressed(CARRY_THREE_BUTTON))
				{
					SetLiftTarget(CARRY_THREE_POS);
					//do not need to set the lift target since PID sets this
					closedSS = closed_changing_level;
				}
				else if (IsButtonPressed(RELEASE_BUTTON))
				{
					SetLiftTarget(CARRY_STEP_POS - LIFT_OFFSET);
					//do not need to set the lift target since PID sets this
					closedSS = releasing_lowering;
				}
				break;
			case closed_changing_level:
				if(CheckLiftHasReachedTarget())
				{
					if(abs(targetLiftEncoderPos-CARRY_ONE_POS) <= FLOAT_COMP_TOL)
						robotState = closed_C_Pos_One;
					else if(abs(targetLiftEncoderPos-CARRY_TWO_POS) <= FLOAT_COMP_TOL)
						robotState = closed_C_Pos_Two;
#if 0  // removed carry 3
					else if(abs(targetLiftEncoderPos-CARRY_THREE_POS) <= FLOAT_COMP_TOL)
						robotState = closed_C_Pos_Three;
#endif
					closedSS = closed_idle;
				}
				else
					CheckAndHandleHaltPickupGantry();
				break;
			case releasing_lowering:
				if (CheckLiftHasReachedTarget())
				{
					SetForkTarget(absGearToothCount + FORK_OFFSET);
					SetForkMotor(FORK_MOTOR_OUT_SPEED);
					closedSS = releasing_open_forks;
				}
				else
					CheckAndHandleHaltPickupGantry();
				break;
			case releasing_open_forks:
				UpdateGearToothCount(); //update whenever the fork is moving
				if (CheckForkHasReachedTarget())
				{
					SetForkMotor(MOTOR_STOP);
					robotState = released;
					releasedSS = released_idle;
				}
				else
					CheckAndHandleHaltPickupGantry();
				break;
			}
			break;
#endif
		case released:
			sprintf(myString, "In Released\n");
			SmartDashboard::PutString("DB/String 0", myString);
			TurnForkLedsOff();
			TurnLedOn(RELEASE_LED);
			//keep the current lift LEDs on

			switch (releasedSS)
			{
			case released_idle:
				sprintf(myString, "In rel_idle\n");
				SmartDashboard::PutString("DB/String 1", myString);
				if(IsButtonPressed(OPEN_NARROW_BUTTON))
				{
					sprintf(myString, "ON pressed\n");
					SmartDashboard::PutString("DB/String 2", myString);
					SetForkTarget(OPEN_NARROW_COUNT);
					//do not move the forks until the lift is in the ground pickup position
					SetLiftTarget(PICKUP_POS);
					//do not need to set the lift target since PID sets this
					releasedSS = released_lowering;
				}
				else if(IsButtonPressed(OPEN_WIDE_BUTTON))
				{
					sprintf(myString, "OW pressed\n");
					SmartDashboard::PutString("DB/String 2", myString);
					SetForkTarget(OPEN_WIDE_COUNT);
					//do not move the forks until the lift is in the ground pickup position
					SetLiftTarget(PICKUP_POS);
					//do not need to set the lift target since PID sets this
					releasedSS = released_lowering;
				}
				break;
			case released_lowering:
				sprintf(myString, "In rel_lower\n");
				SmartDashboard::PutString("DB/String 1", myString);
				if(CheckLiftHasReachedTarget())
				{
					sprintf(myString, "Lift @ Target\n");
					SmartDashboard::PutString("DB/String 3", myString);
					if(absGearToothCount < targetForkGearCount)
						SetForkMotor(FORK_MOTOR_OUT_SPEED);
					else if(absGearToothCount < targetForkGearCount)
						SetForkMotor(FORK_MOTOR_IN_SPEED);
					//do nothing if already at the target since the moving_to_open substate will detect that are already at the target
					releasedSS = moving_to_open;
				}
				else
				{
					sprintf(myString, "Lift NOT @ Target\n");
					SmartDashboard::PutString("DB/String 3", myString);
					CheckAndHandleHaltPickupGantry();
				}
				break;
			case moving_to_open:
				sprintf(myString, "In mov_to_open\n");
				SmartDashboard::PutString("DB/String 1", myString);
				if(CheckForkHasReachedTarget())
				{
					sprintf(myString, "F @ targ: %d\n", absGearToothCount);
					SmartDashboard::PutString("DB/String 4", myString);
					SetForkMotor(MOTOR_STOP);
					if(targetForkGearCount == OPEN_NARROW_COUNT)
					{
						robotState = opened_narrow_GP;
						openedNarrowSS = narrow_idle;
					}
					else //targetForkGearCount == OPEN_WIDE_COUNT
					{
						robotState = opened_wide_GP;
						openedWideSS = wide_idle;
					}
				}
				else if(GetForkLimitSwitchInner())
				{
					SetForkMotor(MOTOR_STOP);
					SetForkTarget(OPEN_NARROW_COUNT);
					SetForkMotor(FORK_MOTOR_OUT_SPEED);
					robotState = opened_narrow_GP;
					openedNarrowSS = narrow_error_recovery;
				}
				else
					CheckAndHandleHaltPickupGantry();  // MLL to deal with outer limit
				break;
			}
			break;

			case halt_pickup_gantry:
				sprintf(myString, "In Halt\n");
				SmartDashboard::PutString("DB/String 0", myString);
				operatorBox->SetOutputs(ALL_LEDS);
			break;
	}
}
