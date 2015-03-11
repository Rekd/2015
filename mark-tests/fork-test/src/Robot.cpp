#include "WPILib.h"

//constants
#define MOTOR_REV							-1
#define MOTOR_NOT_REV						1
#define MOTOR_STOP							0.0
#define PRACTICE 0
#define COMPETITION 1

//configure
#define BUILD_VERSION 						PRACTICE
#define FORK_MOTOR_REV_STATE				MOTOR_REV
#define CH_TOOTH_TRIGGER					3 //analog
#define CH_FORK_MOTOR						13 //can
#define CH_FORK_LS_MIN						0 //dio
#define CH_FORK_LS_MAX						1 //dio
#define CH_JS								3 //usb
#define MOTOR_SPEED							0.3 //should be non-signed

//speed convention is:
//+ is outwards
//- is inwards

class Robot: public IterativeRobot
{
private:
	typedef enum{closing,
		opening} ForkState;

	typedef enum {
		inwards = -1,
		outwards = 1
	} ForkDirection;

#if BUILD_VERSION == COMPETITION
	CANTalon *forkMotor;
#else
	CANJaguar *forkMotor;
#endif
	Counter *gearToothCounter;
	AnalogTrigger *toothTrigger;
	DigitalInput *forkLimitSwitchMin;
	DigitalInput *forkLimitSwitchMax;

	Joystick *dsBox;  //this is a separate test running simultaneous with the fork test

	int running; //0 = do not run the test program, 1 = run the test program

	ForkState forkState;

	// fork position tracking
	int  targetForkGearCount; //target is an integer gear count
	ForkDirection forkDirection;
	float curForkSetSpeed; //the current fork speed
	int absGearToothCount; //absolute gear tooth count (not relative)
	int curGearToothCount; //current gear tooth count
	int lastGearToothCount; //last gear tooth count


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
		curForkSetSpeed = FORK_MOTOR_REV_STATE*val;
		forkMotor->Set(curForkSetSpeed);
		/*
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
		*/
	}

	void UpdateGearCount ()
	//this function turns the relative gear tooth counter into an absolute counter
	{
		if (curForkSetSpeed < 0)
			forkDirection = inwards;
		else //curForkSetSpeed > 0; this function should not be called when curForkSpeed = 0, should on be called when the forks are moving;
			forkDirection = outwards;

		curGearToothCount = gearToothCounter->Get();
		absGearToothCount += forkDirection*(curGearToothCount-lastGearToothCount);
		lastGearToothCount = curGearToothCount;

		/*
		rawGearToothCount = gearToothCounter->Get();
//		difference = std::abs(lastGearToothCount-rawGearToothCount);
//		lastGearToothCount = rawGearToothCount;

		if (direction)
		{
			globalGearToothCount += rawGearToothCount;
		}
		else
		{
			globalGearToothCount -= rawGearToothCount;
		}
		gearToothCounter->Reset();
		*/
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit() {
		forkState = closing;
		SetForkMotor(-MOTOR_SPEED); //start in closing state
		absGearToothCount = 0;
		curGearToothCount = 0;
		lastGearToothCount = 0;
		running = true;
		//targetForkGearCount not used in this test code
		//forkDirection initialized when first used
		//curForkSetSpeed initialized when first used
	}

	void TeleopPeriodic()
	{
		char myString [64];

		if (running)
		{
			dsBox->SetOutputs(1023);

			switch (forkState)
			{
				case closing:
					UpdateGearCount (); //update whenever the forks are moving
					if (GetForkLimitSwitchMin())
					{
						SetForkMotor(MOTOR_STOP);
						sprintf(myString, "max count: %d\n", gearToothCounter->Get());
						SmartDashboard::PutString("DB/String 5", myString);
//						gearToothCounter->Reset();
						SetForkMotor(MOTOR_SPEED);
						forkState = opening;
					}
					break;

				case opening:
					UpdateGearCount (); //update whenever the forks are moving
					if (GetForkLimitSwitchMax())
					{
						SetForkMotor(MOTOR_STOP);
						sprintf(myString, "max count: %d\n", gearToothCounter->Get());
						SmartDashboard::PutString("DB/String 5", myString);
//						gearToothCounter->Reset();
						SetForkMotor(-MOTOR_SPEED);
						forkState = closing;
					}
					break;
			}
		}

		//current monitor check for safety
		if (forkMotor->GetOutputCurrent() > 25.0f)
		{
			SetForkMotor(MOTOR_STOP);
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
		sprintf(myString, "forkDir: %d\n", forkDirection);
		SmartDashboard::PutString("DB/String 5", myString);
//		sprintf(myString, "calcGear: %d\n", gearToothCount);
//		SmartDashboard::PutString("DB/String 5", myString);
		sprintf(myString, "abs gear count: %d\n", absGearToothCount);
		SmartDashboard::PutString("DB/String 9", myString);
	}

	void TestPeriodic()
	{

	}

public:
	Robot()
	{
		toothTrigger = new AnalogTrigger(CH_TOOTH_TRIGGER);
		toothTrigger->SetLimitsRaw(450, 2400);
		gearToothCounter = new Counter(toothTrigger);
#if BUILD_VERSION == COMPETITION
		forkMotor = new CANTalon(CH_FORK_MOTOR);
#else
		forkMotor = new CANJaguar(CH_FORK_MOTOR);
#endif
		forkLimitSwitchMin = new DigitalInput(CH_FORK_LS_MIN);
		forkLimitSwitchMax = new DigitalInput(CH_FORK_LS_MAX);

		dsBox = new Joystick(CH_JS);
	}

	~Robot() {
		delete toothTrigger;
		delete gearToothCounter;
		delete forkMotor;
		delete forkLimitSwitchMin;
		delete forkLimitSwitchMax;
		dsBox->SetOutputs(0);

		delete dsBox;
	}
};

START_ROBOT_CLASS(Robot);
