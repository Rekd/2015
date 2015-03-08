#include "WPILib.h"
#include "Robot.h"
#include <math.h>

class Robot: public IterativeRobot
{
private:
	typedef enum{lowering,
		raising,
		holding} LiftState;

#if BUILD_VERSION == COMPETITION
	CANTalon *liftMotor;
#else
	CANJaguar *liftMotor;
#endif
	Encoder *liftEncoder;
	PIDController *controlLift;
	DigitalInput *liftLimitSwitchMin; //at the bottom of the lift
	DigitalInput *liftLimitSwitchMax; //at the top of the lift
	Joystick *joystick; //used to enter and exit the holding position state

	double liftEncoderDistPerPulse = 1.0/LIFT_ENCODER_RESOLUTION;

	LiftState liftState;
	bool running = false;
	float  motorSpeed = 0.0f;
	bool enterHoldCommand = false; //command from the joystick to enter into holding position state
	bool exitHoldCommand = true; //command from the joystick to exit the holding position state
	bool liftEncZeroed = false; //zero is at the bottom of the lift
	bool liftEncFullRanged = false; //full range is at the top of the lift
	double maxLiftEncDist = -1; //encoder distance at the top
	float pidPosSetPoint = -1; //set point for the PID controller


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

	float DistToSetpoint()
	{
		if(!(controlLift->IsEnabled()))
				return -1;
		else
			return (liftEncoder->GetDistance() - pidPosSetPoint);
	}

	bool AtSetpoint()
	{
		int setTol = 25; //integer number of the distance per pulse

		if(!(controlLift->IsEnabled()))
			return false;
		else
			return (abs(DistToSetpoint()) < setTol*liftEncoderDistPerPulse);
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
		liftEncoder->SetMinRate(0.25);

		liftEncoder->SetPIDSourceParameter(liftEncoder->kDistance);
		controlLift = new PIDController(0.5, 0.05, 0, 0, liftEncoder, liftMotor);
		controlLift->SetContinuous(true); //treat input to controller as continuous; true by default
		controlLift->SetOutputRange(-1.0, 1.0);
		controlLift->Disable(); //do not enable until in holding position mode


#if BUILD_VERSION == COMPETITION
		liftMotor = new CANTalon(13);
#else
		liftMotor = new CANJaguar(13);
#endif
		liftLimitSwitchMin = new DigitalInput(CHAN_LIFT_LOW_LS);
		liftLimitSwitchMax = new DigitalInput(CHAN_LIFT_HIGH_LS);
		joystick = new Joystick(0);
		motorSpeed = -0.4;  //start by lowering the lift
		running = true;
	}

	void TeleopPeriodic()
	{
		char myString [64];

		if (running)
		{
			enterHoldCommand = joystick->GetRawButton(1);
			exitHoldCommand = joystick->GetRawButton(2);

			switch (liftState)
			{
				case raising:
					SetLiftMotor(motorSpeed);

					if (GetLiftLimitSwitchMax())
					{
						SetLiftMotor(0.0f);
						if(!liftEncFullRanged)
						{
							maxLiftEncDist = liftEncoder->GetDistance();
							liftEncFullRanged = true;
						}
						motorSpeed = -0.4;
						liftState = lowering;
					}

					if (enterHoldCommand && liftEncZeroed && liftEncFullRanged)
					{
						liftState = holding;
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

					if (enterHoldCommand && liftEncZeroed && liftEncFullRanged)
					{
						liftState = holding;
					}
					break;

				case holding:
					if(!(controlLift->IsEnabled()))
					{
						pidPosSetPoint = maxLiftEncDist/2; //go to the midpoint of the range
						controlLift->SetSetpoint(pidPosSetPoint);
						controlLift->Enable();
					}

					if(exitHoldCommand)
					{
						controlLift->Disable();
						motorSpeed = -0.4;
						liftState = lowering;
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
		sprintf(myString, "max enc set: %d\n", liftEncFullRanged);
		SmartDashboard::PutString("DB/String 4", myString);
		sprintf(myString, "maxLiftEncDist: %f\n", maxLiftEncDist);
		SmartDashboard::PutString("DB/String 5", myString);
		sprintf(myString, "encoder distance: %f\n", liftEncoder->GetDistance());
		SmartDashboard::PutString("DB/String 6", myString);
		sprintf(myString, "pid: %d\n", controlLift->IsEnabled());
		SmartDashboard::PutString("DB/String 7", myString);
		sprintf(myString, "dist to sp : %f\n", DistToSetpoint());
		SmartDashboard::PutString("DB/String 8", myString);
		sprintf(myString, "at sp : %d\n", AtSetpoint());
		SmartDashboard::PutString("DB/String 9", myString);
	}

	void TestPeriodic()
	{
		//not used
	}
};

START_ROBOT_CLASS(Robot);
