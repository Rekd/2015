//Adding a line to get Git to sync

#include "WPILib.h"
#include "Robot.h"

#define BUT_PICKUP 1
#define BUT_CARRY_1 2
#define BUT_CARRY_2 3

class Robot: public IterativeRobot
{
private:
#if BUILD_VERSION == COMPETITION
		CANTalon *liftMotor;
#else
		CANJaguar *liftMotor;
#endif
	Encoder *liftEncoder;
	PIDController *controlLift;
	DigitalInput *liftLimitSwitchMin;
	DigitalInput *liftLimitSwitchMax;
	Joystick *joystick;
	enum {pickup, carry1, carry2} liftState;
	float maxLiftEncDist; //encoder distance at the top

	bool initialZeroing; //initial zeroing but not yet zeroed

	bool GetLiftLimitSwitchMin()
	{
		//invert so that TRUE when limit switch is closed and FALSE when limit switch is open
		return !(liftLimitSwitchMin->Get());
	}

	bool GetLiftLimitSwitchMax()
	{
		//invert so that TRUE when limit switch is closed and FALSE when limit switch is open
		return !(liftLimitSwitchMax->Get());
	}

	void SetLiftMotor(float val)
	{
		liftMotor->Set(LIFT_MOTOR_DIR*val);
	}

	float DistToSetpoint()
	{
		if(!(controlLift->IsEnabled()))
				return UNINIT_VAL;
		else
			return ((float)(liftEncoder->GetDistance()) - (float)(controlLift->GetSetpoint()));
	}

	bool AtSetpoint()
	{
		if(!(controlLift->IsEnabled()))
			return false;
		else
			return (abs(DistToSetpoint()) < LIFT_POS_TOL*LIFT_ENCODER_DIST_PER_PULSE);
	}

	void AutonomousInit()
	{
		//not used
	}

	void AutonomousPeriodic()
	{
		//not used
	}

	void RobotInit()
	{
		//not used
	}

	void TeleopInit()
	{
		initialZeroing = true;
//		SetLiftMotor(-MOTOR_SPEED_DOWN); //move towards the bottom
		maxLiftEncDist = UNINIT_VAL;
	}


	void TeleopPeriodic()
	{
		char myString [STAT_STR_LEN];

		if (initialZeroing)  // moving towards the bottom
		{
			sprintf(myString, "initZero ip\n"); //in progress
			SmartDashboard::PutString("DB/String 0", myString);
			if (GetLiftLimitSwitchMin())
			{
//				SetLiftMotor(MOTOR_SPEED_STOP);
				liftEncoder->Reset();
				initialZeroing = false;
				liftState = pickup;
				sprintf(myString, "initZero comp\n"); //complete
				SmartDashboard::PutString("DB/String 0", myString);
			}
		}
		else  //manual control
		{
			if (liftState == pickup)
			{
				if (joystick->GetRawButton(BUT_CARRY_1)) // change to LS_1
				{
					controlLift->SetSetpoint(CARRY_ONE_POS);
					liftState = carry1;

				}
				else if (joystick->GetRawButton(BUT_CARRY_2))  // change to LS_2
				{
					controlLift->SetSetpoint(CARRY_TWO_POS);
					liftState = carry2;
				}
			}
			else if (liftState == carry1)
			{
				if (joystick->GetRawButton(BUT_PICKUP)) // change to pickup
				{
					controlLift->SetSetpoint(PICKUP_POS);
					liftState = pickup;

				}
				else if (joystick->GetRawButton(BUT_CARRY_2))  // change to Carry2
				{
					controlLift->SetSetpoint(CARRY_TWO_POS);
					liftState = carry2;
				}
			}
			else if (liftState == carry2)
			{
				if (joystick->GetRawButton(BUT_PICKUP)) // change to pickup
				{
					controlLift->SetSetpoint(PICKUP_POS);
					liftState = pickup;

				}
				else if (joystick->GetRawButton(BUT_CARRY_1))  // change to Carry1
				{
					controlLift->SetSetpoint(CARRY_ONE_POS);
					liftState = carry1;
				}
			}

			//counter control
			if ((joystick->GetRawButton(BUT_JS_RES_EN)) || (GetLiftLimitSwitchMin()))  // reset the encoder
				liftEncoder->Reset();
		}

		//status
		sprintf(myString, "Pickup bt %d\n", joystick->GetRawButton(BUT_PICKUP));
		SmartDashboard::PutString("DB/String 1", myString);
		sprintf(myString, "Carry1 %d\n", joystick->GetRawButton(BUT_CARRY_1));
		SmartDashboard::PutString("DB/String 2", myString);
		sprintf(myString, "Carry2 %d\n", joystick->GetRawButton(BUT_CARRY_2));
		SmartDashboard::PutString("DB/String 3", myString);
		sprintf(myString, "curr: %f\n", liftMotor->GetOutputCurrent());
		SmartDashboard::PutString("DB/String 4", myString);
		sprintf(myString, "new SP: %f\n", controlLift->GetSetpoint());
		SmartDashboard::PutString("DB/String 5", myString);
		sprintf(myString, "Dist: %f\n", liftEncoder->GetDistance());
		SmartDashboard::PutString("DB/String 6", myString);
		sprintf(myString, "Dist to SP: %f\n", DistToSetpoint());
		SmartDashboard::PutString("DB/String 7", myString);
		sprintf(myString, "At SP: %d\n", AtSetpoint());
		SmartDashboard::PutString("DB/String 8", myString);
	}

	void TestPeriodic()
	{
		//not used
	}

public:
	Robot()
	{
#if BUILD_VERSION == COMPETITION
		liftMotor = new CANTalon(CHAN_LIFT_MOTOR);
#else
		liftMotor = new CANJaguar(CHAN_LIFT_MOTOR);
#endif
		liftEncoder = new Encoder(CHAN_LIFT_ENCODER_A, CHAN_LIFT_ENCODER_B, false, Encoder::EncodingType::k4X);
		liftEncoder->SetDistancePerPulse(LIFT_ENCODER_DIST_PER_PULSE);
		controlLift = new PIDController(LIFT_PROPORTIONAL_TERM, LIFT_INTEGRAL_TERM, LIFT_DIFFERENTIAL_TERM, liftEncoder, liftMotor);
		controlLift->SetContinuous(true); //treat input to controller as continuous; true by default
		controlLift->SetOutputRange(LIFT_PID_OUT_MIN, LIFT_PID_OUT_MAX);
		controlLift->Enable(); //do not enable until in holding position mode

		liftLimitSwitchMin = new DigitalInput(CHAN_LIFT_LOW_LS);
		liftLimitSwitchMax = new DigitalInput(CHAN_LIFT_HIGH_LS);
		joystick = new Joystick(CHAN_JS);
	}

	~Robot()
	{
		delete liftMotor;
		delete liftEncoder;
		delete liftLimitSwitchMin;
		delete liftLimitSwitchMax;
		delete joystick;
	}
};

START_ROBOT_CLASS(Robot);
