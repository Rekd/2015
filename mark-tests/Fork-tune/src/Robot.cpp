#include "WPILib.h"
//#include "CANTalon.h"

class Robot: public IterativeRobot
{
private:
	typedef enum{closing,
		opening} ForkState;

	CANTalon *forkMotor;
	Counter *gearToothCounter;
	AnalogTrigger *toothTrigger;
	DigitalInput *forkLimitSwitchMin;
	DigitalInput *forkLimitSwitchMax;
	Joystick *joystick;

	ForkState forkState;

	bool runningZero, zeroed;


	bool GetForkLimitSwitchMin()
	{
		return forkLimitSwitchMin->Get();
	}

	bool GetForkLimitSwitchMax()
	{
		return forkLimitSwitchMax->Get();
	}

	void SetForkMotor(float val)
	{
		forkMotor->Set(val);
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void RobotInit()
	{

	}

	void TeleopInit()
	{
		forkState = closing;
		joystick = new Joystick(0);
		toothTrigger = new AnalogTrigger(3);
		toothTrigger->SetLimitsRaw(450, 2400);
		gearToothCounter = new Counter(toothTrigger);
		forkMotor = new CANTalon(13);
		forkLimitSwitchMin = new DigitalInput(0);
		forkLimitSwitchMax = new DigitalInput(1);
		SetForkMotor(0.25f);
		runningZero = true;
		zeroed = false;
	}


	void TeleopPeriodic()
	{
		char myString [64];

		if (runningZero)  // moving to the inner limit
		{
			sprintf(myString, "moving to 0\n");
			SmartDashboard::PutString("DB/String 1", myString);
			if (!GetForkLimitSwitchMin())
			{
				SetForkMotor(0.0f);
				gearToothCounter->Reset();
				zeroed=true;
				runningZero = false;
			}
		}
		else
		{
			sprintf(myString, "joystick %d\n", joystick->GetRawButton(1));
			SmartDashboard::PutString("DB/String 1", myString);
			if (joystick->GetRawButton(1))  // button pressed...run motor
				SetForkMotor(-0.25f);
			else  // button not pressed
				SetForkMotor(0.0f);
		}
		sprintf(myString, "curr: %f\n", forkMotor->GetOutputCurrent());
		SmartDashboard::PutString("DB/String 2", myString);
		sprintf(myString, "gear count: %d\n", gearToothCounter->Get());
		SmartDashboard::PutString("DB/String 3", myString);
		sprintf(myString, "State: %d\n", forkState);
		SmartDashboard::PutString("DB/String 4", myString);
	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot);
