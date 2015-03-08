#include "WPILib.h"
#include "Robot.h"

class Robot: public IterativeRobot
{
private:
	typedef enum{closing,
		opening} ForkState;

#if BUILD_VERSION == COMPETITION
	CANTalon *forkMotor;
#else
	CANJaguar *forkMotor;
#endif

	Counter *gearToothCounter;
	AnalogTrigger *toothTrigger;
	DigitalInput *forkLimitSwitchMin;
	DigitalInput *forkLimitSwitchMax;

	ForkState forkState;

	bool running;
	float  motorSpeed;
	float  maxCurr = 0.0;


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
#if BUILD_VERSION == COMPETITION
		forkMotor = new CANTalon(13);
#else
		forkMotor = new CANJaguar(13);
#endif
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
					if (GetForkLimitSwitchMin())
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
					if (GetForkLimitSwitchMax())
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
		float curr = forkMotor->GetOutputCurrent();
		if (curr > maxCurr)
			maxCurr = curr;
		sprintf(myString, "max curr = %f\n", maxCurr);
		SmartDashboard::PutString("DB/String 6", myString);

		if (curr > 24.0f)
		{
			sprintf(myString, "kill motor\n");
			SmartDashboard::PutString("DB/String 6", myString);
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
		sprintf(myString, "motorSpeed: %f\n", motorSpeed);
		SmartDashboard::PutString("DB/String 5", myString);
	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot);
