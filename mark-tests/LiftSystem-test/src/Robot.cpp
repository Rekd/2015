#include "WPILib.h"
#include "Constants.h"
#include "LiftSystem.h"
#include "robot.h"

class Robot: public IterativeRobot
{
private:

	LiftSystem  *robotLiftSystem;
#if BUILD_VERSION == COMPETITION
	CANTalon * forkMotor;
	CANTalon * liftMotor;
#else
	CANJaguar * forkMotor;
	CANJaguar * liftMotor;
#endif
	Encoder     *liftEnc;
	Counter     *gearToothCounter;
	AnalogTrigger *toothTrigger;
	DigitalInput  *forkLimitMin, *forkLimitMax, *liftLimitMin, *liftLimitMax;

	Joystick *joystick;
	bool runningZero, zeroed, runningOpenNarrow;


	void RobotInit()
	{
		toothTrigger = new AnalogTrigger(ACHAN_GEAR_COUNT);
		toothTrigger->SetLimitsRaw(450, 2400);
		gearToothCounter = new Counter(toothTrigger);

#if BUILD_VERSION == COMPETITION
		forkMotor = new CANTalon(FORK_MOTOR_ID);
		liftMotor = new CANTalon(LIFT_MOTOR_ID);
#else
		forkMotor = new CANJaguar(FORK_MOTOR_ID);
		liftMotor = new CANJaguar(LIFT_MOTOR_ID);
#endif

// add later		liftEnc = new Encoder(CHAN_ENCODER_LIFT);
		forkLimitMin = new DigitalInput(CHAN_FORK_LIMIT_MIN);
		forkLimitMax = new DigitalInput(CHAN_FORK_LIMIT_MAX);
		liftLimitMin = new DigitalInput(CHAN_LIFT_LIMIT_MIN);
		liftLimitMax = new DigitalInput(CHAN_LIFT_LIMIT_MAX);

		joystick = new Joystick(0);

		robotLiftSystem = new LiftSystem(forkMotor, liftMotor, gearToothCounter, liftEnc,
				 forkLimitMin, forkLimitMax, liftLimitMin, liftLimitMax, joystick);





	}

	void AutonomousInit()
	{
		runningZero = true;
		zeroed = false;
		runningOpenNarrow = false;
		forkMotor->Set(0.25f);
	}

	void AutonomousPeriodic()
	{
		char myString [64];


		if (runningZero)  // moving to the inner limit
		{
			sprintf(myString, "moving to 0\n");
			SmartDashboard::PutString("DB/String 1", myString);
			if (!forkLimitMin->Get())
			{
				gearToothCounter->Reset();
				forkMotor->Set(-0.25f);
				zeroed=true;
				runningZero = false;
				runningOpenNarrow = true;
			}
		}
		else if (runningOpenNarrow)
		{
			if (gearToothCounter->Get() >= OPEN_NARROW_COUNT)
			{
				forkMotor->Set(-0.0f);
				runningOpenNarrow = true;
			}
			else if (!forkLimitMax->Get())
			{
				forkMotor->Set(-0.0f);
				runningOpenNarrow = true;
			}
			sprintf(myString, "moving to O-N\n");
			SmartDashboard::PutString("DB/String 1", myString);
		}
	}

	void TeleopInit()
	{
		char myString [64];


		sprintf(myString, "");
		SmartDashboard::PutString("DB/String 0", myString);
		SmartDashboard::PutString("DB/String 1", myString);
		SmartDashboard::PutString("DB/String 2", myString);
		SmartDashboard::PutString("DB/String 3", myString);
		SmartDashboard::PutString("DB/String 4", myString);
		SmartDashboard::PutString("DB/String 5", myString);
		SmartDashboard::PutString("DB/String 6", myString);
		SmartDashboard::PutString("DB/String 7", myString);
		SmartDashboard::PutString("DB/String 8", myString);
		SmartDashboard::PutString("DB/String 9", myString);
	}

	void TeleopPeriodic()
	{
		robotLiftSystem->Update();
	}



	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot);
