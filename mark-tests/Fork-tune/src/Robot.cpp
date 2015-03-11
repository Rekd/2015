#include "WPILib.h"
#include "Robot.h"

class Robot: public IterativeRobot
{
private:
#if BUILD_VERSION == COMPETITION
	CANTalon *forkMotor;
#else
	CANJaguar *forkMotor;
#endif
	AnalogTrigger *toothTrigger;
	Counter *gearToothCounter;
	DigitalInput *forkLimitSwitchMin;
	DigitalInput *forkLimitSwitchMax;
	Joystick *joystick;

	bool initialZeroing; //initial zeroing but not yet zeroed

	bool GetForkLimitSwitchMin()
	{
		//invert so that TRUE when limit switch is closed and FALSE when limit switch is open
		return !(forkLimitSwitchMin->Get());
	}

	bool GetForkLimitSwitchMax()
	{
		//invert so that TRUE when limit switch is closed and FALSE when limit switch is open
		return !(forkLimitSwitchMax->Get());
	}

	void SetForkMotor(float val)
	{
		forkMotor->Set(FORK_MOTOR_DIR*val);
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
		SetForkMotor(-MOTOR_SPEED_GO); //move to the inner limit switch
		initialZeroing = true;
	}


	void TeleopPeriodic()
	{
		char myString [STAT_STR_LEN];

		if (initialZeroing)  // moving to the inner limit
		{
			sprintf(myString, "initZero ip\n"); //in progress
			SmartDashboard::PutString("DB/String 0", myString);
			if (GetForkLimitSwitchMin())
			{
				SetForkMotor(MOTOR_SPEED_STOP);
				gearToothCounter->Reset();
				initialZeroing = false;
				sprintf(myString, "initZero comp\n"); //complete
				SmartDashboard::PutString("DB/String 0", myString);
			}
		}
		else  //manual control
		{
			//motor control
			if (joystick->GetRawButton(BUT_JS_OUT) && !GetForkLimitSwitchMax())  // move outwards
				SetForkMotor(MOTOR_SPEED_GO);
			else if(joystick->GetRawButton(BUT_JS_IN) && !GetForkLimitSwitchMin())  // move inwards
				SetForkMotor(-MOTOR_SPEED_GO);
			else
				SetForkMotor(MOTOR_SPEED_STOP); //stop

			//counter control
			if (joystick->GetRawButton(BUT_JS_RES_GTC))  // reset the gear tooth counter
				gearToothCounter->Reset();
		}

		//status
		sprintf(myString, "jsOut %d\n", joystick->GetRawButton(BUT_JS_OUT));
		SmartDashboard::PutString("DB/String 1", myString);
		sprintf(myString, "jsIn %d\n", joystick->GetRawButton(BUT_JS_IN));
		SmartDashboard::PutString("DB/String 2", myString);
		sprintf(myString, "jsResGtc %d\n", joystick->GetRawButton(BUT_JS_RES_GTC));
		SmartDashboard::PutString("DB/String 3", myString);
		sprintf(myString, "curr: %f\n", forkMotor->GetOutputCurrent());
		SmartDashboard::PutString("DB/String 4", myString);
		sprintf(myString, "gtc #: %d\n", gearToothCounter->Get()); //gtc count
		SmartDashboard::PutString("DB/String 5", myString);
	}

	void TestPeriodic()
	{
		//not used
	}

public:
	Robot()
	{
#if BUILD_VERSION == COMPETITION
		forkMotor = new CANTalon(CHAN_FORK_MOTOR);
#else
		forkMotor = new CANJaguar(CHAN_FORK_MOTOR);
#endif
		toothTrigger = new AnalogTrigger(CHAN_GTC);
		toothTrigger->SetLimitsRaw(ANALOG_TRIG_MIN, ANALOG_TRIG_MAX);
		gearToothCounter = new Counter(toothTrigger);
		forkLimitSwitchMin = new DigitalInput(CHAN_FORK_MIN_LS);
		forkLimitSwitchMax = new DigitalInput(CHAN_FORK_MAX_LS);
		joystick = new Joystick(CHAN_JS);
	}

	~Robot()
	{
		delete forkMotor;
		delete toothTrigger;
		delete gearToothCounter;
		delete forkLimitSwitchMin;
		delete forkLimitSwitchMax;
		delete joystick;
	}
};

START_ROBOT_CLASS(Robot);
