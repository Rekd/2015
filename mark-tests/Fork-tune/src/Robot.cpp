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

	Counter *gearToothCounter;
	AnalogTrigger *toothTrigger;
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
		forkMotor->Set(val);
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
		joystick = new Joystick(0);
		toothTrigger = new AnalogTrigger(3);
		toothTrigger->SetLimitsRaw(450, 2400);
		gearToothCounter = new Counter(toothTrigger);
#if BUILD_VERSION == COMPETITION
		forkMotor = new CANTalon(13);
#else
		forkMotor = new CANJaguar(13);
#endif
		forkLimitSwitchMin = new DigitalInput(0);
		forkLimitSwitchMax = new DigitalInput(1);
		SetForkMotor(0.25f); //move in the direction of the inner limit
		initialZeroing = true;
	}


	void TeleopPeriodic()
	{
		char myString [64];

		if (initialZeroing)  // moving to the inner limit
		{
			sprintf(myString, "initZero ip\n"); //in progress
			SmartDashboard::PutString("DB/String 0", myString);
			if (GetForkLimitSwitchMin())
			{
				SetForkMotor(0.0f);
				gearToothCounter->Reset();
				initialZeroing = false;
				sprintf(myString, "initZero comp\n"); //complete
				SmartDashboard::PutString("DB/String 0", myString);
			}
		}
		else  //manual control
		{
			//motor control
			if (joystick->GetRawButton(1) && !GetForkLimitSwitchMax())  // moving to the outer limit
				SetForkMotor(-0.25f);
			else if(joystick->GetRawButton(2) && !GetForkLimitSwitchMin())  // moving to the inner limit
				SetForkMotor(0.25f);
			else
				SetForkMotor(0.0f); //stop

			//counter control
			if (joystick->GetRawButton(3))  // reset the gear tooth counter
				gearToothCounter->Reset();

		}

		//status
		sprintf(myString, "joystickB1 %d\n", joystick->GetRawButton(1));
		SmartDashboard::PutString("DB/String 1", myString);
		sprintf(myString, "joystickB2 %d\n", joystick->GetRawButton(2));
		SmartDashboard::PutString("DB/String 2", myString);
		sprintf(myString, "joystickB3 %d\n", joystick->GetRawButton(3));
		SmartDashboard::PutString("DB/String 3", myString);
		sprintf(myString, "curr: %f\n", forkMotor->GetOutputCurrent());
		SmartDashboard::PutString("DB/String 4", myString);
		sprintf(myString, "gear count: %d\n", gearToothCounter->Get());
		SmartDashboard::PutString("DB/String 5", myString);
	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot);
