#include "WPILib.h"
//#include "CANTalon.h"

class Robot: public IterativeRobot
{
private:
	typedef enum{closing, wait_open,
		opening, wait_close} ForkState;

	CANTalon *forkMotor;
	Counter *gearToothCounter;
	AnalogTrigger *toothTrigger;
	DigitalInput *forkLimitSwitchMin;
	DigitalInput *forkLimitSwitchMax;

	ForkState forkState;

	bool running;
	float  motorSpeed;

// global fork motor count
	bool direction;
	int rawGearToothCount;
	int globalGearToothCount;
	int lastGearToothCount;
	int difference;


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
		if (val > 0.0)
		{
			//out = true, in = false
			UpdateGearCount();
			direction = true;
		}else if (val < 0.0)
		{
			UpdateGearCount();
			direction = false;
		}

		forkMotor->Set(val);
	}

	void UpdateGearCount ()
	{
		rawGearToothCount = gearToothCounter->Get();
//		difference = std::abs(lastGearToothCount-rawGearToothCount);
//		lastGearToothCount = rawGearToothCount;

		if (direction)
		{
			globalGearToothCount += rawGearToothCount;
		}else
		{
			globalGearToothCount -= rawGearToothCount;
		}
		gearToothCounter->Reset();
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
		motorSpeed = -0.3;
		SetForkMotor(motorSpeed);
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
//					SetForkMotor(motorSpeed);
					if (!GetForkLimitSwitchMin())
					{
						SetForkMotor(0.0f);
						motorSpeed = 0.3;
						sprintf(myString, "max count: %d\n", gearToothCounter->Get());
						SmartDashboard::PutString("DB/String 5", myString);
//						gearToothCounter->Reset();
						forkState = wait_open;
					}
					break;

				case wait_open:
					wait(0.5);
					SetForkMotor(motorSpeed);
					forkState = opening;
					break;

				case opening:
//					SetForkMotor(motorSpeed);
					if (!GetForkLimitSwitchMax())
					{
						SetForkMotor(0.0f);
						motorSpeed=-0.3;
						sprintf(myString, "max count: %d\n", gearToothCounter->Get());
						SmartDashboard::PutString("DB/String 5", myString);
//						gearToothCounter->Reset();
						forkState = wait_close;
					}
					break;
				case wait_close:
					wait(0.5);
					SetForkMotor(MotorSpeed);
					forkState = closing;
					break;
			}
		}
		if (forkMotor->GetOutputCurrent() > 25.0f)
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
		sprintf(myString, "dir: %d\n", direction);
		SmartDashboard::PutString("DB/String 5", myString);
		sprintf(myString, "calcGear: %d\n", gearToothCount);
		SmartDashboard::PutString("DB/String 5", myString);
		sprintf(myString, "G gear count: %d\n", globalGearToothCount);
		SmartDashboard::PutString("DB/String 9", myString);
	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot);
