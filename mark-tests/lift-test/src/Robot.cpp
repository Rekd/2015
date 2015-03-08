#include "WPILib.h"
#include "Robot.h"

class Robot: public IterativeRobot
{
private:
	typedef enum{lowering,
		raising} LiftState;

#if BUILD_VERSION == COMPETITION
	CANTalon *liftMotor;
#else
	CANJaguar *liftMotor;
#endif

	Encoder *liftEncoder;
	double liftEncoderDistPerPulse = 1.0/LIFT_ENCODER_RESOLUTION;

	DigitalInput *liftLimitSwitchMin; //at the bottom of the lift
	DigitalInput *liftLimitSwitchMax; //at the top of the lift

	LiftState liftState;

	bool running;
	float  motorSpeed;
	bool liftEncZeroed = false; //zero is at the bottom of the lift
	double maxLiftEncDist = -1; //encoder distance at the top


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
		liftMotor->Set(val);
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
		liftState = lowering;

		liftEncoder = new Encoder(CHAN_LIFT_ENCODER_LEFT_A, CHAN_LIFT_ENCODER_LEFT_B, false, Encoder::EncodingType::k4X);
		liftEncoder->SetDistancePerPulse(liftEncoderDistPerPulse);


#if BUILD_VERSION == COMPETITION
		liftMotor = new CANTalon(13);
#else
		liftMotor = new CANJaguar(13);
#endif
		liftLimitSwitchMin = new DigitalInput(CHAN_LIFT_LOW_LS);
		liftLimitSwitchMax = new DigitalInput(CHAN_LIFT_HIGH_LS);
		motorSpeed = -0.4;  //start by lowering the lift
		running = true;
	}

	void TeleopPeriodic()
	{
		char myString [64];

		if (running)
		{
			switch (liftState)
			{
				case raising:
					SetLiftMotor(motorSpeed);

					if (GetLiftLimitSwitchMax())
					{
						SetLiftMotor(0.0f);
						if(maxLiftEncDist == -1)
						{
							maxLiftEncDist = liftEncoder->GetDistance();
						}
						motorSpeed = -0.4;
						liftState = lowering;
					}
					break;

				case lowering:
					SetLiftMotor(motorSpeed);

					if (GetLiftLimitSwitchMin())
					{

						SetLiftMotor(0.0f);
						if(!liftEncZeroed)
						{
							liftEncoder->Reset();
							liftEncZeroed = true;
						}
						motorSpeed=0.4;
						liftState = raising;
					}
					break;
			}
		}

		//status
		sprintf(myString, "running: %d\n", running);
		SmartDashboard::PutString("DB/String 0", myString);
		sprintf(myString, "State: %d\n", liftState);
		SmartDashboard::PutString("DB/String 1", myString);
		sprintf(myString, "motorSpeed: %f\n", motorSpeed);
		SmartDashboard::PutString("DB/String 2", myString);
		sprintf(myString, "lift encoder zeroed: %d\n", liftEncZeroed);
		SmartDashboard::PutString("DB/String 3", myString);
		sprintf(myString, "maxLiftEncDist: %f\n", maxLiftEncDist);
		SmartDashboard::PutString("DB/String 4", myString);
		sprintf(myString, "encoder distance: %f\n", liftEncoder->GetDistance());
		SmartDashboard::PutString("DB/String 5", myString);

	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot);
