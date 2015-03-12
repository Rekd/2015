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
	CANJaguar * leftIntakeMotor;
	CANJaguar * rightIntakeMotor;
#endif
	PIDController *controlLift;
	Encoder     *liftEncoder;
	Counter     *gearToothCounter;
	AnalogTrigger *toothTrigger;
	DigitalInput  *forkLimitMin, *forkLimitMax, *liftLimitMin, *liftLimitMax;

	Joystick *operatorBox;
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
		leftIntakeMotor = new CANJaguar(FORK_MOTOR_ID);
		rightIntakeMotor = new CANJaguar(LIFT_MOTOR_ID);
#endif

		liftEncoder = new Encoder(CHAN_LIFT_ENCODER_A, CHAN_LIFT_ENCODER_B, false, Encoder::EncodingType::k4X);
		liftEncoder->SetDistancePerPulse(LIFT_ENCODER_DIST_PER_PULSE);

		controlLift = new PIDController(LIFT_PROPORTIONAL_TERM, LIFT_INTEGRAL_TERM, LIFT_DIFFERENTIAL_TERM, liftEncoder, liftMotor);
		controlLift->SetContinuous(true); //treat input to controller as continuous; true by default
		controlLift->SetOutputRange(LIFT_PID_OUT_MIN, LIFT_PID_OUT_MAX);
		controlLift->Enable();

		forkLimitMin = new DigitalInput(CHAN_FORK_LIMIT_MIN);
		forkLimitMax = new DigitalInput(CHAN_FORK_LIMIT_MAX);
		liftLimitMin = new DigitalInput(CHAN_LIFT_LIMIT_MIN);
		liftLimitMax = new DigitalInput(CHAN_LIFT_LIMIT_MAX);

		operatorBox = new Joystick(3);

		robotLiftSystem = new LiftSystem(forkMotor, liftMotor, leftIntakeMotor, rightIntakeMotor,
				gearToothCounter, liftEncoder, controlLift,
				 forkLimitMin, forkLimitMax, liftLimitMin, liftLimitMax, operatorBox);
	}

	void AutonomousInit()
	{
		runningZero = true;
		zeroed = false;
		runningOpenNarrow = false;
		forkMotor->Set(0.5f);
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
				forkMotor->Set(-0.5f);
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

// add later?		robotLiftSystem->MoveToResetState();
		controlLift->SetSetpoint(PICKUP_POS);
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
