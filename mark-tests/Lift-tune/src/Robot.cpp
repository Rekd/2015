//Adding a line to get Git to sync

#include "WPILib.h"
#include "Robot.h"

class Robot: public IterativeRobot
{
private:
#if BUILD_VERSION == COMPETITION
		CANTalon *liftMotorBack;
		CANTalon *liftMotorFront;
#else
		CANJaguar *liftMotorBack;
		CANJaguar *liftMotorFront;
#endif
	Encoder *liftEncoder;
	DigitalInput *liftLimitSwitchMin;
	DigitalInput *liftLimitSwitchMax;
	Joystick *joystick;

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
		liftMotorBack->Set(LIFT_MOTOR_DIR*val);
		liftMotorFront->Set(LIFT_MOTOR_DIR*val);
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
		SetLiftMotor(-MOTOR_SPEED_DOWN); //move towards the bottom
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
				SetLiftMotor(MOTOR_SPEED_STOP);
				liftEncoder->Reset();
				initialZeroing = false;
				sprintf(myString, "initZero comp\n"); //complete
				SmartDashboard::PutString("DB/String 0", myString);
			}
		}
		else  //manual control
		{
			//motor control
			if (joystick->GetRawButton(BUT_JS_UP) && !GetLiftLimitSwitchMax())  // move to the top
				SetLiftMotor(MOTOR_SPEED_UP);
			else if(joystick->GetRawButton(BUT_JS_DOWN) && !GetLiftLimitSwitchMin())  // move to the bottom
				SetLiftMotor(-MOTOR_SPEED_DOWN);
			else
				SetLiftMotor(MOTOR_SPEED_STOP); //stop

			//counter control
			if ((joystick->GetRawButton(BUT_JS_RES_EN)) || (GetLiftLimitSwitchMin()))  // reset the encoder
				liftEncoder->Reset();
		}

		//status
		sprintf(myString, "jsUp %d\n", joystick->GetRawButton(BUT_JS_UP));
		SmartDashboard::PutString("DB/String 1", myString);
		sprintf(myString, "jsDown %d\n", joystick->GetRawButton(BUT_JS_DOWN));
		SmartDashboard::PutString("DB/String 2", myString);
		sprintf(myString, "jsResEnc %d\n", joystick->GetRawButton(BUT_JS_RES_EN));
		SmartDashboard::PutString("DB/String 3", myString);
		sprintf(myString, "backCurr: %f\n", liftMotorBack->GetOutputCurrent());
		SmartDashboard::PutString("DB/String 4", myString);
		sprintf(myString, "frCurr: %f\n", liftMotorFront->GetOutputCurrent());
		SmartDashboard::PutString("DB/String 5", myString);
		sprintf(myString, "enc dist: %f\n", liftEncoder->GetDistance()); //encoder distance
		SmartDashboard::PutString("DB/String 6", myString);
	}

	void TestPeriodic()
	{
		//not used
	}

public:
	Robot()
	{
#if BUILD_VERSION == COMPETITION
		liftMotorBack = new CANTalon(CHAN_LIFT_MOTOR_BACK);
		liftMotorFront = new CANTalon(CHAN_LIFT_MOTOR_FRONT);
#else
		liftMotorBack = new CANJaguar(CHAN_LIFT_MOTOR_BACK);
		liftMotorFront = new CANJaguar(CHAN_LIFT_MOTOR_FRONT);
#endif
		liftEncoder = new Encoder(CHAN_LIFT_ENCODER_A, CHAN_LIFT_ENCODER_B, false, Encoder::EncodingType::k4X);
		liftEncoder->SetDistancePerPulse(LIFT_ENCODER_DIST_PER_PULSE);
		liftLimitSwitchMin = new DigitalInput(CHAN_LIFT_LOW_LS);
		liftLimitSwitchMax = new DigitalInput(CHAN_LIFT_HIGH_LS);
		joystick = new Joystick(CHAN_JS);
	}

	~Robot()
	{
		delete liftMotorBack;
		delete liftMotorFront;
		delete liftEncoder;
		delete liftLimitSwitchMin;
		delete liftLimitSwitchMax;
		delete joystick;
	}
};

START_ROBOT_CLASS(Robot);
