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

	ForkState forkState;

	bool running;
	float  motorSpeed;


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

		toothTrigger = new AnalogTrigger(3);
		toothTrigger->SetLimitsRaw(450, 2400);
		gearToothCounter = new Counter(toothTrigger);
		forkMotor = new CANTalon(13);
		forkLimitSwitchMin = new DigitalInput(0);
		forkLimitSwitchMax = new DigitalInput(1);
		motorSpeed = 0.1;
		running = true;

	}

	void TeleopPeriodic()
	{
		char myString [64];

		if (running)
		{
			switch (forkState)
			{
				case closing:
					SetForkMotor(motorSpeed);
					if (motorSpeed < 0.6)
						motorSpeed = motorSpeed+0.1;
					if (!GetForkLimitSwitchMin())
					{
						SetForkMotor(0.0f);
						motorSpeed = -0.05;
						sprintf(myString, "max count: %d\n", gearToothCounter->Get());
						SmartDashboard::PutString("DB/String 5", myString);
						gearToothCounter->Reset();
						forkState = opening;
					}
					break;

				case opening:
					SetForkMotor(motorSpeed);
					if (motorSpeed>-0.6)
						motorSpeed=motorSpeed-0.1;
					if (!GetForkLimitSwitchMax())
					{
						SetForkMotor(0.0f);
						motorSpeed=0.1;
						sprintf(myString, "max count: %d\n", gearToothCounter->Get());
						SmartDashboard::PutString("DB/String 5", myString);
						gearToothCounter->Reset();
						forkState = closing;
					}
					break;
			}
		}
		if (forkMotor->GetOutputCurrent() > 7.0f)
		{
			SetForkMotor(0.0f);
			running = false;
		}

		sprintf(myString, "curr: %f\n", forkMotor->GetOutputCurrent());
		SmartDashboard::PutString("DB/String 1", myString);
		sprintf(myString, "gear count: %d\n", gearToothCounter->Get());
		SmartDashboard::PutString("DB/String 2", myString);
		sprintf(myString, "State: %d\n", forkState);
		SmartDashboard::PutString("DB/String 3", myString);
		sprintf(myString, "running: %d\n", running);
		SmartDashboard::PutString("DB/String 4", myString);
	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot);
