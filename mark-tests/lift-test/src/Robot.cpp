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
	float  motorSpeed = MOTOR_SPEED_STOP;
	bool enterHoldCommand = false; //command from the joystick to enter into holding position state
	bool exitHoldCommand = true; //command from the joystick to exit the holding position state
	bool liftEncZeroed = false; //zero is at the bottom of the lift
	bool liftEncFullRanged = false; //full range is at the top of the lift
	double maxLiftEncDist = UNINIT_VAL; //encoder distance at the top
	float pidPosSetPoint = UNINIT_VAL; //set point for the PID controller


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
				return UNINIT_VAL;
		else
			return (liftEncoder->GetDistance() - pidPosSetPoint);
	}

	bool AtSetpoint()
	{
		if(!(controlLift->IsEnabled()))
			return false;
		else
			return (abs(DistToSetpoint()) < PID_POS_TOL*liftEncoderDistPerPulse);
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
		liftEncoder->SetMinRate(MIN_ENCODER_RATE);

		liftEncoder->SetPIDSourceParameter(liftEncoder->kDistance);
		controlLift = new PIDController(PID_P, PID_I, PID_D, PID_F, liftEncoder, liftMotor);
		controlLift->SetContinuous(true); //treat input to controller as continuous; true by default
		controlLift->SetOutputRange(PID_OUT_MIN, PID_OUT_MAX);
		controlLift->Disable(); //do not enable until in holding position mode


#if BUILD_VERSION == COMPETITION
		liftMotor = new CANTalon(CHAN_LIFT_MOTOR);
#else
		liftMotor = new CANJaguar(CHAN_LIFT_MOTOR);
#endif
		liftLimitSwitchMin = new DigitalInput(CHAN_LIFT_LOW_LS);
		liftLimitSwitchMax = new DigitalInput(CHAN_LIFT_HIGH_LS);
		joystick = new Joystick(CHAN_JS);
		motorSpeed = -MOTOR_SPEED_GO;  //start by lowering the lift
		running = true;
	}

	void TeleopPeriodic()
	{
		char myString [STAT_STR_LEN];

		if (running)
		{
			enterHoldCommand = joystick->GetRawButton(BUT_JS_ENT_POS_HOLD);
			exitHoldCommand = joystick->GetRawButton(BUT_JS_EXIT_POS_HOLD);

			switch (liftState)
			{
				case raising:
					SetLiftMotor(motorSpeed);

					if (GetLiftLimitSwitchMax())
					{
						SetLiftMotor(MOTOR_SPEED_STOP);
						if(!liftEncFullRanged)
						{
							maxLiftEncDist = liftEncoder->GetDistance();
							liftEncFullRanged = true;
						}
						motorSpeed = -MOTOR_SPEED_GO;
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

						SetLiftMotor(MOTOR_SPEED_STOP);
						if(!liftEncZeroed)
						{
							liftEncoder->Reset();
							liftEncZeroed = true;
						}
						motorSpeed=MOTOR_SPEED_GO;
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
						pidPosSetPoint = SP_RANGE_FRACTION*maxLiftEncDist; //go to the midpoint of the range
						controlLift->SetSetpoint(pidPosSetPoint);
						controlLift->Enable();
					}

					if(exitHoldCommand)
					{
						controlLift->Disable();
						motorSpeed = -MOTOR_SPEED_GO;
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
