/*
 * LiftSystem.cpp
 *
 *  Created on: Feb 15: 2015
 *      Author: mll
 */

#include "LiftSystem.h"
#include <math.h>

LiftSystem::LiftSystem(CANSpeedController *pForkMotor, CANSpeedController *pLiftMotor, CANSpeedController *pLeftIntake, CANSpeedController *pRightIntake,
		Counter *pGearToothCounter, Encoder *pLiftEnc, PIDController *pLiftControl,
		DigitalInput *pForkLimitInner, DigitalInput *pForkLimitOuter, DigitalInput *pLiftLimitLow, DigitalInput *pLiftLimitHigh,
		Joystick *pOperatorBox)
{
	//assign objects
	forkMotor = pForkMotor;
	liftMotor = pLiftMotor;
	leftIntake = pLeftIntake;
	rightIntake = pRightIntake;
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
	targetLiftEncoderCount = 0;
	//forkDirection will be initialized when first used
	//curForkSetSpeed will be initialized when first used
	absGearToothCount = 0;
	curGearToothCount = 0;
	lastGearToothCount = 0;

	//initialize the robot state machine
	robotState = opened_narrow_GP;
	openedNarrowSS = narrow_idle;
}

LiftSystem::~LiftSystem()
{
	delete forkMotor;
	delete liftMotor;
	delete leftIntake;
	delete rightIntake;
	delete gearToothCounter;
	delete liftEnc;
	delete forkLimitInner;
	delete forkLimitOuter;
	delete liftLimitLow;
	delete liftLimitHigh;
	delete operatorBox;
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
		SetForkMotor(MOTOR_STOP);
		liftControl->Disable();
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

void LiftSystem::UpdateGearToothCount()
//this function turns the relative gear tooth counter into an absolute counter
{
	if (curForkSetSpeed < 0)
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

void LiftSystem::SetLiftTarget(int target)
{
	targetLiftEncoderCount = target;
	liftControl->SetSetpoint(target);
}

bool LiftSystem::CheckForkHasReachedTarget()
{
	if (abs(absGearToothCount - targetForkGearCount) >= FORK_POS_TOL)
		return true;
	else
		return false;
}

bool LiftSystem::CheckLiftHasReachedTarget()
{
	if (abs(liftControl->GetError()) >= LIFT_POS_TOL)
		return true;
	else
		return false;
}

bool LiftSystem::IsButtonPressed(int button)
{
	return(operatorBox->GetRawButton(button));
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

//Main
void LiftSystem::Update()
{
	switch (robotState)
	{
		case opened_narrow_GP:
			TurnForkLedsOff();
			TurnLedOn(OPEN_NARROW_LED);
			TurnLiftLedsOff();

			switch (openedNarrowSS)
			{
				case narrow_idle:
					if (IsButtonPressed(OPEN_WIDE_BUTTON))
					{
						SetForkTarget(OPEN_WIDE_COUNT);
						SetForkMotor(FORK_MOTOR_OUT_SPEED);
						openedNarrowSS = opening_wide;
					}
					else if (IsButtonPressed(CLOSE_BUTTON))
					{
						SetForkTarget(CLOSE_COUNT);
						SetForkMotor(FORK_MOTOR_IN_SPEED);
						openedNarrowSS = narrow_closing_fork;
					}
					break;
				case narrow_closing_fork:
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
					UpdateGearToothCount(); //update whenever the fork is moving
					if (CheckForkHasReachedTarget())
					{
						SetForkMotor(MOTOR_STOP);  // stop fork motor
						robotState = opened_wide_GP;
						openedWideSS = wide_idle;
					}
					else
						CheckAndHandleHaltPickupGantry();
					break;
				case narrow_changing_lift:

					if (CheckLiftHasReachedTarget())
					{
						robotState = closed_C_Pos_One;
						closedSS = closed_idle;
					}
					else
						CheckAndHandleHaltPickupGantry();
					break;
				case narrow_error_recovery:
					if (CheckForkHasReachedTarget())
					{
						SetForkMotor(MOTOR_STOPPED);
						openedNarrowSS = narrow_idle;
					}
					else
						CheckAndHandleHaltPickupGantry();
					break;
			}
			break;

		case opened_wide_GP:
			TurnForkLedsOff();
			TurnLedOn(OPEN_WIDE_LED);
			TurnLiftLedsOff();

			switch (openedWideSS)
			{
				case wide_idle:
					if (IsButtonPressed(OPEN_NARROW_BUTTON))
					{
						SetForkTarget(OPEN_NARROW_COUNT);
						SetForkMotor(FORK_MOTOR_IN_SPEED);
						openedWideSS = opening_narrow;
					}
					else if (IsButtonPressed(CLOSE_BUTTON))
					{
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
						SetForkMotor(MOTOR_STOP);
						robotState = opened_narrow_GP;
						openedNarrowSS = narrow_idle;
					}
					else if (GetForkLimitSwitchInner())
					{
						SetForkMotor(MOTOR_STOP);
						SetForkTarget(OPEN_WIDE_COUNT);
						SetForkMotor(FORK_MOTOR_OUT_SPEED);
						openedWideSS = wide_error_recovery;
					}
					break;
				case wide_changing_lift:
					if (CheckLiftHasReachedTarget())
					{
						robotState = closed_C_Pos_One;
						closedSS = closed_idle;
					}
					else
						CheckAndHandleHaltPickupGantry();
					break;
				case wide_error_recovery:
					if (CheckForkHasReachedTarget())
					{
						SetForkMotor(MOTOR_STOPPED);
						openedWideSS = wide_idle;
					}
					else
						CheckAndHandleHaltPickupGantry();
					break;
			}
			break;

		case closed_C_Pos_One:
			TurnForkLedsOff();
			TurnLedOn(CLOSE_LED);
			TurnLiftLedsOff();
			TurnLedOn(CARRY_ONE_LED);

			switch (closedSS)
			{
				case closed_idle:
					if (IsButtonPressed(CARRY_TWO_BUTTON))
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
					else if (IsButtonPressed(CARRY_STEP_BUTTON))
					{
						SetLiftTarget(CARRY_STEP_POS);
						//do not need to set the lift target since PID sets this
						closedSS = closed_changing_level;
					}
					else if (IsButtonPressed(RELEASE_BUTTON))
					{
						SetLiftTarget(CARRY_ONE_POS - LIFT_OFFSET);
						//do not need to set the lift target since PID sets this
						closedSS = releasing_lowering;
					}
					break;
				case closed_changing_level:
					if(CheckLiftHasReachedTarget())
					{
						switch(targetLiftEncoderCount)
						{
							case CARRY_TWO_POS:
								robotState = closed_C_Pos_Two;
								break;
							case CARRY_THREE_POS:
								robotState = closed_C_Pos_Three;
								break;
							case CARRY_STEP_POS:
								robotState = closed_C_Pos_Step;
								break;
						}
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
						SetForkMotor(STOP_MOTOR);
						robotState = released;
						releasedSS = released_idle;
					}
					else
						CheckAndHandleHaltPickupGantry();
					break;
			}
			break;

		case closed_C_Pos_Two:
			TurnForkLedsOff();
			TurnLedOn(CLOSE_LED);
			TurnLiftLedsOff();
			TurnLedOn(CARRY_TWO_LED);

			switch (closedSS)
			{
				case closed_idle:
					if (IsButtonPressed(CARRY_ONE_BUTTON))
					{
						SetLiftTarget(CARRY_ONE_POS);
						//do not need to set the lift target since PID sets this
						closedSS = closed_changing_level;
					}
					else if (IsButtonPressed(CARRY_THREE_BUTTON))
					{
						SetLiftTarget(CARRY_THREE_POS);
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
						SetLiftTarget(CARRY_TWO_POS - LIFT_OFFSET);
						//do not need to set the lift target since PID sets this
						closedSS = releasing_lowering;
					}
					break;
				case closed_changing_level:
					if(CheckLiftHasReachedTarget())
					{
						switch(targetLiftEncoderCount)
						{
							case CARRY_ONE_POS:
								robotState = closed_C_Pos_One;
								break;
							case CARRY_THREE_POS:
								robotState = closed_C_Pos_Three;
								break;
							case CARRY_STEP_POS:
								robotState = closed_C_Pos_Step;
								break;
						}
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
						SetForkMotor(STOP_MOTOR);
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
					switch(targetLiftEncoderCount)
					{
					case CARRY_ONE_POS:
						robotState = closed_C_Pos_One;
						break;
					case CARRY_TWO_POS:
						robotState = closed_C_Pos_Two;
						break;
					case CARRY_STEP_POS:
						robotState = closed_C_Pos_Step;
						break;
					}
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
					SetForkMotor(STOP_MOTOR);
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
					switch(targetLiftEncoderCount)
					{
					case CARRY_ONE_POS:
						robotState = closed_C_Pos_One;
						break;
					case CARRY_TWO_POS:
						robotState = closed_C_Pos_Two;
						break;
					case CARRY_THREE_POS:
						robotState = closed_C_Pos_Three;
						break;
					}
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
					SetForkMotor(STOP_MOTOR);
					robotState = released;
					releasedSS = released_idle;
				}
				else
					CheckAndHandleHaltPickupGantry();
				break;
			}
			break;

		case released:
			TurnForkLedsOff();
			TurnLedOn(RELEASE_LED);
			//keep the current lift LEDs on

			switch (releasedSS)
			{
			case released_idle:
				if(IsButtonPressed(OPEN_NARROW_BUTTON))
				{
					SetForkTarget(OPEN_NARROW_COUNT);
					//do not move the forks until the lift is in the ground pickup position
					SetLiftTarget(PICKUP_POS);
					//do not need to set the lift target since PID sets this
					releasedSS = released_lowering;
				}
				else if(IsButtonPressed(OPEN_WIDE_BUTTON))
				{
					SetForkTarget(OPEN_WIDE_COUNT);
					//do not move the forks until the lift is in the ground pickup position
					SetLiftTarget(PICKUP_POS);
					//do not need to set the lift target since PID sets this
					releasedSS = released_lowering;
				}
				break;
			case released_lowering:
				if(CheckLiftHasReachedTarget())
				{
					if(absGearToothCount < targetForkGearCount)
						SetForkMotor(FORK_MOTOR_OUT_SPEED);
					else if(absGearToothCount < targetForkGearCount)
						SetForkMotor(FORK_MOTOR_IN_SPEED);
					//do nothing if already at the target since the moving_to_open substate will detect that are already at the target
					releasedSS = moving_to_open;
				}
				else
					CheckAndHandleHaltPickupGantry();
				break;
			case moving_to_open:
				if(CheckForkHasReachedTarget())
				{
					SetForkMotor(MOTOR_STOP);
					if(targetForkGearCount == OPEN_NARROW_COUNT)
					{
						robotState = opened_narrow_GP;
						openedNarrowSS = narrow_idle;
					}
					else //targetForkGearCount == OPEN_WIDE_COUNT
					{
						robotState = opened_wide_GP;
						openedWideSS = narrow_idle;
					}
				}
				else
					CheckAndHandleHaltPickupGantry();
				break;
			}
			break;

			case halt_pickup_gantry:
			operatorBox->SetOutputs(1023); //1023 = 2^10-1 to turn all 10 output leds
			break;
	}
}
