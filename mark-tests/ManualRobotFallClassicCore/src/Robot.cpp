#include "math.h"
#include "WPILib.h"
#include "Constants.h"
#include "LiftSystem.h"
#include "DriveSystem.cpp"

char myString[64]; //for debugging

//function to determine if have reached a control setpoint (target)
//used by PID controllers
bool myOnTarget(PIDController *controller, PIDSource *source)
{
	float error = controller->GetSetpoint() - source->PIDGet();
	return (fabs(error) < POS_ERR_TOL);
}

//function used to determine the drive velocity
//input x is the raw value from the joystick (from -1 to 1)
float driveVelocityProfileX(float x)
{
	int sign;

	if (x == ZERO_FL)
		return ZERO_FL;

	if (x > 0.0)
		sign = 1.0;
	else
		sign = -1.0;

	return(sign * DRIVE_VELOCITY_SCALE * (log10(fabs(x) + 0.1) + 1.0));
}

//function used to determine the drive velocity
//input y is the raw value from the joystick (from -1 to 1)
float driveVelocityProfileY(float y)
{
	//use the same profile as x
	return driveVelocityProfileX(y);
}

class Robot: public IterativeRobot
{
private:
	//memory management
	bool enteredTelopInit; //drive system is created in teleop init so only delete the drive system in the destructor if entered teleop init

	//drive system
#if BUILD_VER == COMPETITION
	Talon * leftDrive;
	Talon * rightDrive;
#elif BUILD_VER == PRACTICE
	Victor * leftDrive;
	Victor * rightDrive;
#endif
	Encoder *leftEncoder;
	Encoder *rightEncoder;
	DriveSystem *driveSystem;

	// Linear Drive PID controllers for nudging
	PIDController *controlPosNudgeRight;
	PIDController *controlPosNudgeLeft;

	//driver controls
	Joystick *driveJoystick;
	Joystick *steeringWheel;
	float driveX, driveY; //values from driver controls: x from steeringWheel, y from driveJoystick
	bool drivePID; //tracks if drive PID is currently on or not

	//drive nudge flags
	bool nudgeLeft, nudgeRight;

	//lift system
#if BUILD_VER == COMPETITION
	CANTalon *liftMotorBack;
	CANTalon *liftMotorFront;
	CANTalon *leftIntakeMotor;
	CANTalon *rightIntakeMotor;
#elif BUILD_VER == PRACTICE
	CANJaguar *liftMotorBack;
	CANJaguar *liftMotorFront;
	CANJaguar *leftIntakeMotor;
	CANJaguar *rightIntakeMotor;
#endif
	DigitalInput *liftLimitLow; //at the bottom of the lift
	DigitalInput *liftLimitHigh; //at the top of the lift
	Encoder *liftEncoder;
	PIDController *controlLiftBack;
	PIDController *controlLiftFront;
	Joystick *liftSysJoystick;
	LiftSystem *liftSystem;

	void RobotInit()
	{
		CameraServer::GetInstance()->SetQuality(50);
		//the camera name (e.g. "cam0") can be found through the roborio web interface
		CameraServer::GetInstance()->StartAutomaticCapture("cam1");
	}

	void AutonomousInit()
	{
		//not used
	}

	void AutonomousPeriodic()
	{
		//not used
	}

	void TeleopInit()
	{
		enteredTelopInit = true;
		driveSystem = new DriveSystem(leftEncoder, rightEncoder, leftDrive, rightDrive);

		//disable nudge position control to start
		controlPosNudgeLeft->Disable();
		controlPosNudgeRight->Disable();
		nudgeRight = false;
		nudgeLeft = false;

		//Initialize drive system PID
		driveSystem->SetPIDDrive(GLOBAL_DRIVE_PID_CONFIG);
		drivePID = GLOBAL_DRIVE_PID_CONFIG;
		//Set Wheel Diameter
		driveSystem->SetWheelDiameter(WHEEL_DIAMETER);
		// start recording the distance traveled
		driveSystem->StartRecordingDistance();
	}

	void TeleopPeriodic()
	{
		//check for PID button on/off
		//the global drive PID setting overrides the joystick, i.e. if the global setting is drive PID off, the joystick has no impact on drive PID
		if(driveJoystick->GetRawButton(DRIVE_PID_OFF_BUTTON) && GLOBAL_DRIVE_PID_CONFIG)
		{
			driveSystem->SetPIDDrive(false);
			drivePID = false;
		}
		else if(driveJoystick->GetRawButton(DRIVE_PID_ON_BUTTON) && GLOBAL_DRIVE_PID_CONFIG)
		{
			driveSystem->SetPIDDrive(true);
			drivePID = true;
		}

		//check for nudge button presses
		//nudge can only be active in one direction at any time
		if((driveJoystick->GetRawButton(DRIVE_NUDGE_LEFT_BUTTON)) && (!nudgeLeft && !nudgeRight))
		{
			nudgeLeft = true;
			nudgeRight = false;
		}
		if((driveJoystick->GetRawButton(DRIVE_NUDGE_RIGHT_BUTTON)) && (!nudgeLeft && !nudgeRight))
		{
			nudgeLeft = false;
			nudgeRight = true;
		}

		if(nudgeLeft)
		{
			//turn on the nudge
			if(!(controlPosNudgeLeft->IsEnabled()))
			{
				driveSystem->SetPIDDrive(false); //disabling velocity-controlled drive system
		        leftEncoder->SetPIDSourceParameter(leftEncoder->kDistance);
		        rightEncoder->SetPIDSourceParameter(rightEncoder->kDistance);
				controlPosNudgeLeft->Enable();
				controlPosNudgeLeft->SetSetpoint(-NUDGE_MOVE_DIST);
				controlPosNudgeRight->Enable();
				controlPosNudgeRight->SetSetpoint(-NUDGE_MOVE_DIST);
			}
			//determine if the nudge is complete
			if ((myOnTarget(controlPosNudgeLeft, leftEncoder)) || (myOnTarget(controlPosNudgeRight, rightEncoder)))
			{
				controlPosNudgeLeft->Disable();
				controlPosNudgeRight->Disable();
				nudgeLeft = false;
				nudgeRight = false;
				if(drivePID) //if drivePID was on before the nudge, turn it back on
					driveSystem->SetPIDDrive(true); //enabling main velocity-controlled drive system
			}
		}
		else if(nudgeRight)
		{
			//turn on the nudge
			if(!(controlPosNudgeLeft->IsEnabled()))
			{
				driveSystem->SetPIDDrive(false); //disabling main velocity-controlled drive system
		        leftEncoder->SetPIDSourceParameter(leftEncoder->kDistance);
		        rightEncoder->SetPIDSourceParameter(rightEncoder->kDistance);
				controlPosNudgeLeft->Enable();
				controlPosNudgeLeft->SetSetpoint(NUDGE_MOVE_DIST);
				controlPosNudgeRight->Enable();
				controlPosNudgeRight->SetSetpoint(NUDGE_MOVE_DIST);
			}
			//determine if the nudge is complete
			if ((myOnTarget(controlPosNudgeLeft, leftEncoder)) || (myOnTarget(controlPosNudgeRight, rightEncoder)))
			{
				controlPosNudgeLeft->Disable();
				controlPosNudgeRight->Disable();
				nudgeRight = false;
				nudgeLeft = false;
				if(drivePID) //if drivePID was on before the nudge, turn it back on
					driveSystem->SetPIDDrive(true); //enabling main velocity controlled drive system
			}
		}
		else
		{
			//get driver controls values
			driveX = -steeringWheel->GetX();
			driveY = driveJoystick->GetY();

			//Filter drive controls deadband
			if (driveX > STEERING_DB_LOW && driveX < STEERING_DB_HIGH)
				driveX = ZERO_FL;
			if (driveY > DRIVE_JS_DB_LOW && driveY < DRIVE_JS_DB_HIGH)
				driveY = ZERO_FL;

			//map to velocity profile
			driveX = driveVelocityProfileX(driveX);
			driveY = driveVelocityProfileY(driveY);

			//Give drive instructions
			driveSystem->SetDriveInstruction(driveY * MAX_RPS, driveX * MAX_RPS);
			driveSystem->Update();
		}

		//Give lift instructions
		liftSystem->Update();
	}

	void TestPeriodic()
	{
#if 0 // for limit switch testing
		if (liftSystem->GetLiftLimitSwitchLow())
		{
			sprintf(myString, "L limit closed\n");
			SmartDashboard::PutString("DB/String 0", myString);
		}
		else
		{
			sprintf(myString, "L limit open\n");
			SmartDashboard::PutString("DB/String 0", myString);
		}
		if (liftSystem->GetLiftLimitSwitchHigh())
		{
			sprintf(myString, "H limit closed\n");
			SmartDashboard::PutString("DB/String 1", myString);
		}
		else
		{
			sprintf(myString, "H limit open\n");
			SmartDashboard::PutString("DB/String 1", myString);
		}
#endif
	}

public:
	Robot()
	{
		//memory management
		enteredTelopInit = false;

		//drive system
#if BUILD_VER == COMPETITION
		leftDrive = new Talon(CHAN_LEFT_DRIVE);
		rightDrive = new Talon(CHAN_RIGHT_DRIVE);
#elif BUILD_VER == PRACTICE
		leftDrive = new Victor(CHAN_LEFT_DRIVE);
		rightDrive = new Victor(CHAN_RIGHT_DRIVE);
#endif
		leftEncoder = new Encoder(CHAN_ENCODER_LEFT_A, CHAN_ENCODER_LEFT_B, false, Encoder::EncodingType::k4X);
		leftEncoder->Reset();
		leftEncoder->SetDistancePerPulse(DRIVE_ENCODER_DIST_PER_PULSE);
        leftEncoder->SetPIDSourceParameter(leftEncoder->kDistance);
		rightEncoder = new Encoder(CHAN_ENCODER_RIGHT_A, CHAN_ENCODER_RIGHT_B, false, Encoder::EncodingType::k4X);
		rightEncoder->Reset();
		rightEncoder->SetDistancePerPulse(DRIVE_ENCODER_DIST_PER_PULSE);
        rightEncoder->SetPIDSourceParameter(rightEncoder->kDistance);

		// Linear Drive PID controllers for nudging
        controlPosNudgeLeft = new PIDController(POS_NUDGE_PROPORTIONAL_TERM, POS_NUDGE_INTEGRAL_TERM, POS_NUDGE_DIFFERENTIAL_TERM, leftEncoder, leftDrive);
        controlPosNudgeLeft->SetContinuous(true);
        controlPosNudgeLeft->SetOutputRange(NUDGE_MAX_REVERSE_SPEED, NUDGE_MAX_FORWARD_SPEED);
        controlPosNudgeRight = new PIDController(POS_NUDGE_PROPORTIONAL_TERM, POS_NUDGE_INTEGRAL_TERM, POS_NUDGE_DIFFERENTIAL_TERM, rightEncoder, rightDrive);
        controlPosNudgeRight->SetContinuous(true);
        controlPosNudgeRight->SetOutputRange(NUDGE_MAX_REVERSE_SPEED, NUDGE_MAX_FORWARD_SPEED);

		//driver controls
        driveJoystick = new Joystick(CHAN_DRIVE_JS);
		steeringWheel = new Joystick(CHAN_STEERING_WHEEL);

		//lift system
#if BUILD_VER == COMPETITION
		liftMotorBack = new CANTalon(CHAN_LIFT_MOTOR_BACK);
		liftMotorFront = new CANTalon(CHAN_LIFT_MOTOR_FRONT);
		leftIntakeMotor = new CANTalon(CHAN_L_INTAKE_MOTOR);
		rightIntakeMotor = new CANTalon(CHAN_R_INTAKE_MOTOR);
#else //practice
		liftMotorBack = new CANJaguar(CHAN_LIFT_MOTOR_BACK);
		liftMotorFront = new CANJaguar(CHAN_LIFT_MOTOR_FRONT);
		leftIntakeMotor = new CANJaguar(CHAN_L_INTAKE_MOTOR);
		rightIntakeMotor = new CANJaguar(CHAN_R_INTAKE_MOTOR);
#endif
		liftLimitLow = new DigitalInput(CHAN_LIFT_LOW_LS);
		liftLimitHigh = new DigitalInput(CHAN_LIFT_HIGH_LS);
		liftEncoder = new Encoder(CHAN_LIFT_ENCODER_A, CHAN_LIFT_ENCODER_B, false, Encoder::EncodingType::k4X);
		liftEncoder->SetDistancePerPulse(LIFT_ENCODER_DIST_PER_PULSE);
#if BUILD_VER == COMPETITION
		liftEncoder->SetReverseDirection(true);
#endif
		liftEncoder->SetPIDSourceParameter(liftEncoder->kDistance);
		liftEncoder->Reset();
		controlLiftBack = new PIDController(LIFT_PROPORTIONAL_TERM, LIFT_INTEGRAL_TERM, LIFT_DIFFERENTIAL_TERM, liftEncoder, liftMotorBack);
		controlLiftBack->SetContinuous(true);
		controlLiftBack->SetOutputRange(LIFT_PID_OUT_MIN, LIFT_PID_OUT_MAX);
		controlLiftBack->Disable();
		controlLiftFront = new PIDController(LIFT_PROPORTIONAL_TERM, LIFT_INTEGRAL_TERM, LIFT_DIFFERENTIAL_TERM, liftEncoder, liftMotorFront);
		controlLiftFront->SetContinuous(true);
		controlLiftFront->SetOutputRange(LIFT_PID_OUT_MIN, LIFT_PID_OUT_MAX);
		controlLiftFront->Disable();

		liftSysJoystick = new Joystick(CHAN_LIFT_SYS_JS);

		liftSystem = new LiftSystem(liftMotorBack, liftMotorFront, leftIntakeMotor, rightIntakeMotor,
				liftLimitLow, liftLimitHigh,
				liftEncoder, controlLiftBack, controlLiftFront,
				liftSysJoystick);
	}

	~Robot()
	{
		if(enteredTelopInit)
			delete driveSystem;

		//drive system
		delete leftDrive;
		delete rightDrive;
		delete leftEncoder;
		delete rightEncoder;

		// Linear Drive PID controllers for nudging
		delete controlPosNudgeLeft;
		delete controlPosNudgeRight;

		//driver controls
		delete driveJoystick;
		delete steeringWheel;

		//lift system
		delete liftMotorBack;
		delete liftMotorFront;
		delete leftIntakeMotor;
		delete rightIntakeMotor;
		delete liftLimitLow;
		delete liftLimitHigh;
		delete liftEncoder;
		delete controlLiftBack;
		delete controlLiftFront;
		delete liftSysJoystick;
		delete liftSystem;
	}
};

START_ROBOT_CLASS(Robot);
