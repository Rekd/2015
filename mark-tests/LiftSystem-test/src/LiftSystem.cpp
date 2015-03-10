/*
 * LiftSystem.cpp
 *
 *  Created on: Feb 15: 2015
 *      Author: mll
 */

#include "LiftSystem.h"


LiftSystem::LiftSystem(CANSpeedController *pforkMotor, CANSpeedController *pliftMotor, Counter *pgearToothCounter, Encoder *liftEnc,
		DigitalInput *forkLimitMin, DigitalInput *forkLimitMax, DigitalInput *liftLimitMin,
		DigitalInput *liftLimitMax, Joystick *pjoystick)
{
// set the local motor
	forkMotor = pforkMotor;
	liftMotor = pliftMotor;
	liftEncoder = liftEnc;
	gearToothCounter = pgearToothCounter;
	forkLimitSwitchMin = forkLimitMin;
	forkLimitSwitchMax = forkLimitMax;
	liftLimitSwitchMin = liftLimitMin;
	liftLimitSwitchMax = liftLimitMax;
	operatorBox = pjoystick;

	targetForkGearCount = 0;
	targetLiftEncoderCount = 0;

// initialize the robot state machines
	robotState = opened_narrow_GP;  //change to opened_wide at a later time
	openedNarrowSubState = narrow_idle;

}

LiftSystem::~LiftSystem()
{
	delete forkMotor;
	delete liftMotor;
	delete gearToothCounter;
	delete liftEncoder;
	delete forkLimitSwitchMin;
	delete forkLimitSwitchMax;
	delete liftLimitSwitchMin;
	delete liftLimitSwitchMax;
	delete operatorBox;
}


//Limit Switches:
bool LiftSystem::GetForkLimitSwitchMin()
{
	//invert so that TRUE when limit switch is closed and FALSE when limit switch is open
	return !(forkLimitSwitchMin->Get());
}

bool LiftSystem::GetForkLimitSwitchMax()
{
	//invert so that TRUE when limit switch is closed and FALSE when limit switch is open
	return !(forkLimitSwitchMax->Get());
}

bool LiftSystem::GetLiftLimitSwitchMin()
{
	return !(liftLimitSwitchMin->Get());
}

bool LiftSystem::GetLiftLimitSwitchMax()
{
	return !(liftLimitSwitchMax->Get());
}

//Motors
void LiftSystem::SetForkMotor(float val)
{
	if (val > 0)
	{
		//out = true, in = false
		direction = true;
	}else if (val < 0)
	{
		direction = false;
	}
	UpdateGearCount();

	forkMotor->Set(val);
}

void LiftSystem::SetLiftMotor(float val)
{
	liftMotor->Set(val);
}

bool  LiftSystem::CheckForkMotorCurrentSpike()
{
	if (forkMotor->GetOutputCurrent() > FORK_CURRENT_LIMIT)
		return(true);
	else
		return(false);
}

void LiftSystem::SetForkTarget(int target)
{
	targetForkGearCount = target;
	// need to add code to adjust the motorDirection
}

void LiftSystem::SetLiftTarget(float target)
{
	targetLiftEncoderCount = target;
}

void LiftSystem::UpdateGearCount ()
{
	rawGearToothCount = gearToothCounter->Get();
	difference = std::abs(lastGearToothCount-rawGearToothCount);
	lastGearToothCount = rawGearToothCount;

	if (direction)
	{
		gearToothCount += difference;
	}else
	{
		gearToothCount -= difference;
	}
}

bool LiftSystem::CheckForkHasReachedTarget()
{
	if (gearToothCounter->Get() >= targetForkGearCount)  // reached target
		return true;
	else
		return false;
}

bool LiftSystem::CheckLiftHasReachedTarget()
{
	if (liftEncoder->Get() >= targetLiftEncoderCount)  // reached target
		return true;
	else
		return false;
}

void LiftSystem::ResetGearCounter()
{
	gearToothCounter->Reset();
}

bool LiftSystem::IsOpenWideButtonPressed()
{
	return(operatorBox->GetRawButton(1));
}

bool LiftSystem::IsOpenNarrowButtonPressed()
{
	return(operatorBox->GetRawButton(2));
}

bool LiftSystem::IsCloseButtonPressed()
{
	return(operatorBox->GetRawButton(3));  //CHECK NUMBER
}

bool LiftSystem::IsCarryButtonOnePressed()
{
	return(operatorBox->GetRawButton(4));  //CHECK NUMBER
}

bool LiftSystem::IsCarryButtonTwoPressed()
{
	return(operatorBox->GetRawButton(5));  //CHECK NUMBER
}

bool LiftSystem::IsCarryButtonThreePressed()
{
	return(operatorBox->GetRawButton(6)); //CHECK NUMBER
}

bool LiftSystem::IsCarryButtonStepPressed()
{
	return(operatorBox->GetRawButton(7)); //CHECK NUMBER
}

bool LiftSystem::IsReleaseButtonPressed()
{
	return(operatorBox->GetRawButton(8)); //CHECK NUMBER
}

bool LiftSystem::IsReleaseWideButtonPressed()
{
	return(operatorBox->GetRawButton(9)); //CHECK NUMBER
}

bool LiftSystem::IsReleaseNarrowButtonPressed()
{
	return(operatorBox->GetRawButton(10)); //CHECK NUMBER
}

//Main
void LiftSystem::Update()
{
	char myString [64];


	sprintf(myString, "in state %d\n", robotState);
	SmartDashboard::PutString("DB/String 2", myString);

	switch (robotState)  // ignoring intakes and fatal error states for now
	{
		case opened_narrow_GP:  // in open-narrow state
			sprintf(myString, "in SS %d\n", openedNarrowSubState);
			SmartDashboard::PutString("DB/String 3", myString);

			switch (openedNarrowSubState)
			{
				case narrow_idle:
					if (IsOpenWideButtonPressed())    // check for open wide button press...if pressed
					{
						sprintf(myString, "rec OW\n");
						SmartDashboard::PutString("DB/String 3", myString);
						ResetGearCounter();
						SetForkTarget(WIDE_NARROW_DIFF);     // calculate new fork position
						SetForkMotor(FORK_MOTOR_OUT_SPEED);  // start fork moving
						openedNarrowSubState = opening_wide;   // change openNarrowSS to opening_wide
					}
					else if (IsCloseButtonPressed())  // check for close button press...if pressed
					{
						sprintf(myString, "closing\n");
						SmartDashboard::PutString("DB/String 3", myString);
						ResetGearCounter();
						SetForkTarget(CLOSING_COUNT);  // calculate new fork position - way in
						SetForkMotor(FORK_MOTOR_IN_SPEED);// start fork closing
						openedNarrowSubState = narrow_closing_fork; // change openNarrowSS to narrow_closing
					}
					break;
				case narrow_closing_fork:
					if (CheckForkMotorCurrentSpike())   // has fork motor current spiked?  If so
					{
						sprintf(myString, "curr spike\n");
						SmartDashboard::PutString("DB/String 3", myString);
						SetForkMotor(0.0);                          // stop fork motor
						// calculate new lift position - closed_C_Pos_One
					    // start lift moving
						openedNarrowSubState = narrow_changing_lift;   // change openNarrowSS to narrow_raising_lift;
					}
					if (GetForkLimitSwitchMin())   // has fork reached inner limit switch (missed pickup)...if so
					{
						sprintf(myString, "inner lim\n");
						SmartDashboard::PutString("DB/String 3", myString);
						SetForkMotor(0.0);         // stop fork motion
						ResetGearCounter();
						SetForkTarget(OPEN_NARROW_COUNT);            // calculate fork position for open narrow position
						SetForkMotor(FORK_MOTOR_OUT_SPEED);          // start fork motor opening
						openedNarrowSubState = narrow_error_recovery;  // change openNarrow_SS to narrow_error_recovery
					}
					break;
				case narrow_changing_lift:

					if (CheckLiftHasReachedTarget())   // has lift reached its new position?  If so
					{
						SetLiftMotor(0.0);  // stop lift motor
						ClosedSubState = closed_idle; // change closedSS to closed_idle
						// turn off open narrow LED
						// turn on closed_C_Pos_One LED
						robotState = closed_C_Pos_One;  // change robotState to closed_C_Pos_One
					}
					break;
				case narrow_error_recovery:
					sprintf(myString, "in error rec\n");
					SmartDashboard::PutString("DB/String 4", myString);
					if (CheckForkHasReachedTarget())  // has fork reached open_narrow position?  if so
					{
						SetForkMotor(0.0);  // stop fork motion
						openedNarrowSubState = narrow_idle;  // set openNarrowSS to narrow_idle
					}
					break;
				case opening_wide:
					if (CheckForkHasReachedTarget()) // has fork reached its new position?  If so
					{
						SetForkMotor(0.0);  // stop fork motor
						// turn off open narrow LED
						// turn on opened_wide_GP LED
						openedWideSubState = wide_idle;  // set OpenedWideSubState to wide_idle
						robotState = opened_wide_GP;  // change robotState to opened_wide_GP
						sprintf(myString, "in OW\n");
						SmartDashboard::PutString("DB/String 4", myString);
					}
					if (!GetForkLimitSwitchMax())   // has fork reached outer limit switch (overextended)...if so
					{
						SetForkMotor(0.0);         // stop fork motion
						//enter an error state
						sprintf(myString, "outer limit\n");
						SmartDashboard::PutString("DB/String 4", myString);
					}
					break;
			}
			break;

		case opened_wide_GP:  // in open-wide state
			sprintf(myString, "in SS %d\n", openedWideSubState);
			SmartDashboard::PutString("DB/String 3", myString);

			switch (openedWideSubState)
			{
				case wide_idle:
					if (IsOpenNarrowButtonPressed())   // check for open narrow button press...if pressed,
					{
						sprintf(myString, "rec ON\n");
						SmartDashboard::PutString("DB/String 3", myString);
						ResetGearCounter();
						SetForkTarget(NARROW_WIDE_DIFF);  // calculate new fork position
						SetForkMotor(FORK_MOTOR_IN_SPEED); // start fork moving
						openedWideSubState = opening_narrow; // change openWideSS to opening_narrow
					}
					else if (IsCloseButtonPressed()) // check for close button press...if pressed
					{
						sprintf(myString, "closing\n");
						SmartDashboard::PutString("DB/String 3", myString);
						ResetGearCounter();
						SetForkTarget(CLOSING_COUNT); // calculate new fork position
						SetForkMotor(FORK_MOTOR_IN_SPEED); // start fork closing
						openedNarrowSubState = wide_closing_fork; // change openWideSS to wide_closing
					}
					break;
				case wide_closing_fork:
					if (CheckForkMotorCurrentSpike()) // has fork motor current spiked?  If so,
					{
						sprintf(myString, "curr spike\n");
						SmartDashboard::PutString("DB/String 3", myString);
						SetForkMotor(0.0);		// stop fork motor
						SetLiftTarget(POS_ONE);// calculate new lift position - closed_C_Pos_One
						SetLiftMotor(LIFT_MOTOR_UP_SPEED);// start lift moving
						openedNarrowSubState = wide_changing_lift;// change openWideSS to wide_raising_lift;
					}

					if (GetForkLimitSwitchMin())   // has fork reached inner limit switch (missed pickup)...if so
					{
						sprintf(myString, "inner lim\n");
						SmartDashboard::PutString("DB/String 3", myString);
						SetForkMotor(0.0);         // stop fork motion
						ResetGearCounter();
						SetForkTarget(OPEN_WIDE_COUNT);            // calculate fork position for open wide position
						SetForkMotor(FORK_MOTOR_OUT_SPEED);          // start fork motor opening
						openedNarrowSubState = wide_error_recovery;  // change openWide_SS to wide_error_recovery
					}
					break;
				case wide_changing_lift:
					if (CheckLiftHasReachedTarget())  // has lift reached its new position?  If so,
					{
						SetLiftMotor(0.0); // stop lift motor
						ClosedSubState = closed_idle; // change closedSS to closed_idle
						RobotState = closed_C_Pos_One; // change robotState to closed_C_Pos_One
					}
					break;
				case wide_error_recovery: //Do later
					// has fork reached open_narrow position?  if so
						// stop fork motion
						// set openWideSS to wide_idle
					break;
				case opening_narrow:
					if(CheckForkHasReachedTarget()) // has fork reached its new position?  If so,
					{
						SetForkMotor(0.0);// stop fork motor
						robotState = opened_narrow_GP;// change robotState to opened_narrow_GP
					}
					break;
			}
			break;

		case closed_C_Pos_One:
			switch (closedSS)
			{
				case closed_idle:
					// check for alternate closed button press...if pressed
					// calculate new lift position based on which button is pressed
					if (IsCarryButtonTwoPressed())
					{
						SetLiftTarget(POS_TWO);
						SetLiftMotor(LIFT_MOTOR_UP_SPEED);
					}else if (IsCarryButtonThreePressed())
					{
						SetLiftTarget(POS_THREE);
						SetLiftMotor(LIFT_MOTOR_UP_SPEED);
					}else if (IsCarryButtonStepPressed())
					{
						SetLiftTarget(POS_STEP);
						SetLiftMotor(LIFT_MOTOR_UP_SPEED);
					}
					// start lift motor moving to new position
					closedSS = closed_changing_level;	// set closedSS to closed_changing_level

					if (IsReleaseButtonPressed()) // check for release button press...if pressed
					{
						SetLiftTarget(liftEncoder->Get()-LIFT_OFFSET);// calculate new lift position (slightly lower)
						SetLiftMotor(LIFT_MOTOR_DOWN_SPEED);// start lift motor
						closedSS = releasing_lowering;    // set closedSS to releasing_lowering
					}
					break;

				case closed_changing_level:
					if(CheckLiftHasReachedTarget()) // has lift reached new position?  if so
					{
						SetLiftMotor(0.0); // stop lift motor
						closedSS = closed_idle; // change closedSS to closed_idle
						// turn off closed_C_Pos_One LED
						// Turn on closed_C_Pos_Two,  closed_C_Pos_Three, or closed_C_Pos_Step LED as appropriate
						switch(targetLiftEncoderCount) // change robotState to closed_C_Pos_Two,  closed_C_Pos_Three, or closed_C_Pos_Step, as appropriate
						{
							case POS_TWO:
								robotState = closed_C_Pos_Two;
								break;

							case POS_THREE:
								robotState = closed_C_Pos_Three;
								break;

							case POS_STEP:
								robotState = closed_C_Pos_Step;
								break;
						}
					}
					break;
				case releasing_lowering:
					if (CheckLiftHasReachedTarget()) // has lift reached new position?  if so
					{
						SetLiftMotor(0.0);// stop lift motor
						SetForkTarget(gearToothCounter->Get() + FORK_OFFSET);// calculate new fork position - slightly wider
						SetForkMotor(FORK_MOTOR_OUT_SPEED);// start fork motor
						closedSS = releasing_open_forks; // set closedSS to releasing_open_forks
					}
					break;
				case releasing_open_forks:
					if (CheckForkHasReachedTarget()) // has fork motors reached new position?  if so,
					{
						SetForkMotor(0.0);// stop fork motor
						releasedSS = released_idle;// set releasedSS to released_idle
						robotState = released;// set robotState to released
					}
					break;
			}
			break;
		case closed_C_Pos_Two:
			switch (closedSS)
			{
				case closed_idle:
					// Check for alternate closed button press...if pressed,
					// 1) calculate new lift position based on which button is pressed,
					if (IsCarryButtonTwoPressed())
					{
						SetLiftTarget(POS_ONE);
						SetLiftMotor(LIFT_MOTOR_DOWN_SPEED); // 2) start lift motor moving to new position,
					}else if (IsCarryButtonThreePressed())
					{
						SetLiftTarget(POS_THREE);
						SetLiftMotor(LIFT_MOTOR_UP_SPEED); // 2) start lift motor moving to new position,
					}else if (IsCarryButtonStepPressed())
					{
						SetLiftTarget(POS_STEP);
						SetLiftMotor(LIFT_MOTOR_DOWN_SPEED); // 2) start lift motor moving to new position,
					}
					closedSS = closed_changing_level;	// 3) set closedSS to closed_changing_level.
					if (IsReleaseButtonPressed()) // Check for release button press...if pressed,
					{
						SetLiftTarget(liftEncoder->Get() - LIFT_OFFSET); // 1) calculate new lift position (slightly lower),
						SetLiftMotor(FORK_MOTOR_OUT_SPEED); // 2) start lift motor,
						closedSS = releasing_lowering; // 3) set closedSS to releasing_lowering.
					}
					break;

				case closed_changing_level:
					if(CheckLiftHasReachedTarget()) // Has lift reached new position?  If so,
					{
						SetLiftMotor(0.0);// 1) Stop lift motor,
						// turn off closed_C_Pos_Two LED
						// Turn on closed_C_Pos_One, closed_C_Pos_Three, or closed_C_Pos_Step LED as appropriate
						closedSS = closed_idle;// 2) change closedSS to closed_idle,
						switch(targetLiftEncoderCount) // 3) change robotState to closed_C_Pos_One,  closed_C_Pos_Three, or closed_C_Pos_Step, as appropriate.
						{
							case POS_ONE:
								robotState = closed_C_Pos_One;
								break;

							case POS_THREE:
								robotState = closed_C_Pos_Three;
								break;

							case POS_STEP:
								robotState = closed_C_Pos_Step;
								break;
						}
					}
					break;

				case releasing_lowering:
					if (CheckLiftHasReachedTarget()) // has lift reached new position?  if so
					{
						SetLiftMotor(0.0);// stop lift motor
						SetForkTarget(gearToothCounter->Get() + FORK_OFFSET);// calculate new fork position - slightly wider
						SetForkMotor(FORK_MOTOR_OUT_SPEED);// start fork motor
						closedSS = releasing_open_forks; // set closedSS to releasing_open_forks
					}
					break;
				case releasing_open_forks:
					if (CheckForkHasReachedTarget()) // has fork motors reached new position?  if so,
					{
						SetForkMotor(0.0);// stop fork motor
						releasedSS = released_idle;// set releasedSS to released_idle
						robotState = released;// set robotState to released
					}
					break;
				}
				break;
		case closed_C_Pos_Three:
			switch (closedSS)
			{
				case closed_idle:
					// Check for alternate closed button press...if pressed,
						// 1) calculate new (lower) lift position based on which button is pressed,
					if (IsCarryButtonOnePressed())
					{
						SetLiftTarget(POS_ONE);
						SetLiftMotor(LIFT_MOTOR_DOWN_SPEED); // 2) start lift motor moving to new position,
					}else if (IsCarryButtonTwoPressed())
					{
						SetLiftTarget(POS_TWO);
						SetLiftMotor(LIFT_MOTOR_DOWN_SPEED); // 2) start lift motor moving to new position,
					}else if (IsCarryButtonStepPressed())
					{
						SetLiftTarget(POS_STEP);
						SetLiftMotor(LIFT_MOTOR_DOWN_SPEED); // 2) start lift motor moving to new position,
					}
					closedSS = closed_changing_level;	// 3) set closedSS to closed_changing_level.
					if (IsReleaseButtonPressed()) // Check for release button press...if pressed,
					{
						SetLiftTarget(liftEncoder->Get() - LIFT_OFFSET); // 1) calculate new lift position (slightly lower),
						SetLiftMotor(LIFT_MOTOR_DOWN_SPEED); // 2) start lift motor,
						closedSS = releasing_lowering; // 3) set closedSS to releasing_lowering.
					}
					break;

				case closed_changing_level:
					if(CheckLiftHasReachedTarget()) // Has lift reached new position?  If so,
					{
						SetLiftMotor(0.0);// 1) Stop lift motor,
						// turn off closed_C_Pos_Two LED
						// Turn on closed_C_Pos_One, closed_C_Pos_Three, or closed_C_Pos_Step LED as appropriate
						closedSS = closed_idle;// 2) change closedSS to closed_idle,
						switch(targetLiftEncoderCount) // 3) change robotState to closed_C_Pos_One,  closed_C_Pos_Two, or closed_C_Pos_Step, as appropriate.
						{
							case POS_ONE:
								robotState = closed_C_Pos_One;
								break;

							case POS_TWO:
								robotState = closed_C_Pos_Two;
								break;

							case POS_STEP:
								robotState = closed_C_Pos_Step;
								break;
						}
					break;

				case releasing_lowering:
					if (CheckLiftHasReachedTarget()) // has lift reached new position?  if so
					{
						SetLiftMotor(0.0);// stop lift motor
						SetForkTarget(gearToothCounter->Get() + FORK_OFFSET);// calculate new fork position - slightly wider
						SetForkMotor(FORK_MOTOR_OUT_SPEED);// start fork motor
						closedSS = releasing_open_forks; // set closedSS to releasing_open_forks
					}
					break;
				case releasing_open_forks:
					if (CheckForkHasReachedTarget()) // has fork motors reached new position?  if so,
					{
						SetForkMotor(0.0);// stop fork motor
						releasedSS = released_idle;// set releasedSS to released_idle
						robotState = released;// set robotState to released
					}
					break;
				}
		break;
		case closed_C_Pos_Step:
			switch (closedSS)
			{
			case closed_idle:
				// Check for alternate closed button press...if pressed,
					// 1) calculate new lift position based on which button is pressed,
				if (IsCarryButtonOnePressed())
				{
					SetLiftTarget(POS_ONE);
					SetLiftMotor(LIFT_MOTOR_DOWN_SPEED);
				}else if (IsCarryButtonTwoPressed())
				{
					SetLiftTarget(POS_TWO);
					SetLiftMotor(LIFT_MOTOR_UP_SPEED);
				}else  if (IsCarryButtonThreePressed())
				{
					SetLiftTarget(POS_THREE);
					SetLiftMotor(LIFT_MOTOR_UP_SPEED);
				}
				closedSS = closed_changing_level;	// 3) set closedSS to closed_changing_level.
				if (IsReleaseButtonPressed()) // Check for release button press...if pressed,
				{
					SetLiftTarget(liftEncoder->Get() - LIFT_OFFSET); // 1) calculate new lift position (slightly lower),
					SetLiftMotor(LIFT_MOTOR_DOWN_SPEED); // 2) start lift motor,
					closedSS = releasing_lowering; // 3) set closedSS to releasing_lowering.
				}
				break;

			case closed_changing_level:
				if(CheckLiftHasReachedTarget())// Has lift reached new position?  If so,
				{
					SetLiftMotor(0.0);// 1) Stop lift motor,
					// turn off closed_C_Pos_Step LED
					// Turn on closed_C_Pos_Two,  closed_C_Pos_Three, or closed_C_Pos_One LED as appropriate
					closedSS = closed_idle;// 2) change closedSS to closed_idle,
					switch(targetLiftEncoderCount) // 3) change robotState to closed_C_Pos_One,  closed_C_Pos_Two, or closed_C_Pos_Three, as appropriate.
					{
						case POS_ONE:
							robotState = closed_C_Pos_One;
							break;

						case POS_TWO:
							robotState = closed_C_Pos_Two;
							break;

						case POS_THREE:
							robotState = closed_C_Pos_Three;
							break;
					}
				}
				break;

			case releasing_lowering:
				if (CheckLiftHasReachedTarget()) // has lift reached new position?  if so
				{
					SetLiftMotor(0.0);// stop lift motor
					SetForkTarget(gearToothCounter->Get() + FORK_OFFSET);// calculate new fork position - slightly wider
					SetForkMotor(FORK_MOTOR_OUT_SPEED);// start fork motor
					closedSS = releasing_open_forks; // set closedSS to releasing_open_forks
				}
				break;
			case releasing_open_forks:
				if (CheckForkHasReachedTarget()) // has fork motors reached new position?  if so,
				{
					SetForkMotor(0.0);// stop fork motor
					releasedSS = released_idle;// set releasedSS to released_idle
					robotState = released;// set robotState to released
				}
				break;
			}
			break;
		case released:
			switch (releasedSS)
			{
			case released_idle: // How to get to known position after getting offset by release?
				if(IsReleaseWideButtonPressed()) // check for two "open" buttons.  if pressed
				{
					// calculate new fork position based on which button was pressed
				}else if(IsReleaseNarrowButtonPressed())
				{
					// calculate new fork position based on which button was pressed
				}

				if (IsReleaseWideButtonPressed()||IsReleaseNarrowButtonPressed())
				{
					// calculate new lift position based on which button was pressed
					// start lift and fork motors
					// set done flag to 0
				}

				break;
			case moving_to_open:
				// has fork motor met or exceeded its position?  if so, stop the motor & increment done flag
				// has lift motor met or exceeded its position?  if so, stop the motor & increment done flag
				// is done flag == 2?  then move is complete.
					// set openNarrowSS to narrow_idle or openWideSS to wide_idle as appropriate
					// Turn off released LED
					// Turn on opened_narrow_GP or opened_wide_GP LED as appropriate
					// set robotState to opened_narrow_GP or opened_wide_GP as appropriate
				break;
			}
			break;
		case lift_error:

			break;

	}
}
