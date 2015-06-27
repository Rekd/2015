\
#include "WPILib.h"
#include "Constants.h"
#include "LiftSystem.h"
#include "DriveSystem.cpp"
#include "math.h"
#include "IMU.h"
#include "IMUAdvanced.h"
#include "AHRS.h"

char myString[64]; //for debugging

//local function
bool myOnTarget(PIDController *controller, PIDSource *source)
{
	float error = controller->GetSetpoint() - source->PIDGet();
	float tolerance = error * POS_ERR_TOL; //any percentage value
	return (fabs(tolerance) < POS_TOL_COMP); // a tuned value
}

#if 0 // piece-wise
float velocityProfileX(float x)
{
	if(x > -0.5 && x < 0.5)
		return 0.5*x;
	else
		return (1.5*(x-0.5) + 0.25);
}

#endif

// log-based profile
float velocityProfileX(float x)
{
	int sign;

	if (x == ZERO_FL)
		return(ZERO_FL);

	if (x > 0.0)
		sign = 1.0;
	else
		sign = -1.0;
	return(sign * VELOCITY_SCALE * (log10(fabs(x) + 0.1) + 1.0));
}


float velocityProfileY(float y)
{
	//for now use the same profile as x
	return velocityProfileX(y);
}

class Robot: public IterativeRobot
{
	NetworkTable *table;
	AHRS *imu;
	SerialPort *serial_port;
	bool first_iteration;

private:
	//memory management
	bool enteredTelopInit; //drive system is created in teleop init so only delete the drive system in the destructor if entered teleop init

	//drive system
#if BUILD_VER == COMPETITION
	Talon * leftDrive;
	Talon * rightDrive;
#else
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

	//drive nudge flags
	bool nudgeLeft, nudgeRight;

	//lift system
#if BUILD_VER == COMPETITION
	CANTalon *forkMotor;
	CANTalon *liftMotorBack;
	CANTalon *liftMotorFront;
	CANTalon *leftIntakeMotor;
	CANTalon *rightIntakeMotor;
#else
	CANJaguar *forkMotor;
	CANJaguar *liftMotorBack;
	CANJaguar *liftMotorFront;
	CANJaguar *leftIntakeMotor;
	CANJaguar *rightIntakeMotor;
#endif
	DigitalInput *forkLimitInner;
	DigitalInput *forkLimitOuter;
	DigitalInput *liftLimitLow; //at the bottom of the lift
	DigitalInput *liftLimitHigh; //at the top of the lift
	Encoder *liftEncoder;
	PIDController *controlLiftBack;
	PIDController *controlLiftFront;
	Joystick *liftSysJoystick;
	LiftSystem *liftSystem;

	// Linear Drive PID controllers for Autonomous
	PIDController *controlPosRight;
	PIDController *controlPosLeft;
	enum {start, fork_in, drive, release, hold} autonomousDriveState;

	void RobotInit()
	{
		//IMU init
		table = NetworkTable::GetTable("datatable");
		serial_port = new SerialPort(57600,SerialPort::kMXP);
        uint8_t update_rate_hz = 50;
        imu = new AHRS(serial_port,update_rate_hz);
        if ( imu ) {
        	LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", imu);
        }
        first_iteration = true;

        //Camera
		CameraServer::GetInstance()->SetQuality(50);
		//the camera name (ex "cam0") can be found through the roborio web interface
		CameraServer::GetInstance()->StartAutomaticCapture("cam1");


		driveSystem = new DriveSystem(leftEncoder, rightEncoder, leftDrive, rightDrive);

		//Initialize PID
		driveSystem->SetPIDDrive(PID_CONFIG);
		//Set Wheel Diameter
		driveSystem->SetWheelDiameter(WHEEL_DIAMETER);
		// start recording the distance traveled
		driveSystem->StartRecordingDistance();
	}

	void AutonomousInit()
	{

		liftSystem->StartForksInAuto();  // start the forks moving in
		autonomousDriveState = fork_in;
	}

	void AutonomousPeriodic()
	{
		//Pre-IMU 1-tote autonomous
//		char myString[64];
//
//		liftSystem->UpdateAuto();
//		if (autonomousDriveState == fork_in)
//		{
//			if (!(liftSystem->CheckForksIn()))  // forks have stopped - move
//			{
//				controlPosLeft->Enable();
//				controlPosLeft->SetSetpoint(AUTONMOUS_MOVE_DIST);
//				controlPosRight->Enable();
//				controlPosRight->SetSetpoint(-AUTONMOUS_MOVE_DIST);
//				autonomousDriveState = drive;
//			}
//		}
//		if (autonomousDriveState == drive)
//		{
//    		sprintf(myString, "Driving\n");
//    		SmartDashboard::PutString("DB/String 0", myString);
//    		sprintf(myString, "L Setpoint: %f\n", controlPosLeft->GetSetpoint());
//    		SmartDashboard::PutString("DB/String 6", myString);
//       		sprintf(myString, "L PID: %f\n", leftEncoder->PIDGet());
//    		SmartDashboard::PutString("DB/String 7", myString);
//    		sprintf(myString, "R Setpoint: %f\n", controlPosRight->GetSetpoint());
//    		SmartDashboard::PutString("DB/String 8", myString);
//       		sprintf(myString, "R PID: %f\n", rightEncoder->PIDGet());
//    		SmartDashboard::PutString("DB/String 9", myString);
//
//			if ((myOnTarget(controlPosLeft, leftEncoder)) && (myOnTarget(controlPosRight, rightEncoder)))
//			{
//				controlPosLeft->Disable();  // disable the drive PID controllers
//				controlPosRight->Disable();
//				liftSystem->StartForksOutAuto();  // start the forks moving in
//				autonomousDriveState = release;
//			}
//		}
		int reset_yaw_count = 0;
		int reset_displacement_count = 0;

		bool hasReachedTarget = false;
		float target = 0.0;
		float autoSpeed = 0.0;
		float turnSpeed = 0.4;
		float margin = 4.0;

		while (IsAutonomous())
		{
			if ( first_iteration ) {
				bool is_calibrating = imu->IsCalibrating();
				if ( !is_calibrating ) {
					Wait( 0.3 );
					imu->ZeroYaw();
					first_iteration = false;
				}

				target = imu->GetAngle() + 90;

			}
			bool yaw_axis_up;
			uint8_t yaw_axis = imu->GetBoardYawAxis(yaw_axis_up);

			bool reset_yaw_button_pressed = driveJoystick->GetRawButton(IMU_YAW_RESET);
			if ( reset_yaw_button_pressed ) {
				imu->ZeroYaw();
				reset_yaw_count++;
			}
			bool reset_displacement_button_pressed = DriverStation::GetInstance()->GetStickButton(0,2);
			if ( reset_displacement_button_pressed ) {
				imu->ResetDisplacement();
				reset_displacement_count++;
			}

			float offFromTarget = fabs(imu->GetAngle() - target);
			if(offFromTarget < margin) {
				hasReachedTarget = true;
				driveSystem->SetDriveInstruction(0.0,0.0);
				driveSystem->Update();
			}

			if(!hasReachedTarget) {
				driveSystem->SetDriveInstruction(autoSpeed * MAX_RPS, turnSpeed * MAX_RPS);
				driveSystem->Update();
			}

			SmartDashboard::PutNumber("target", target);
			SmartDashboard::PutNumber("offFromTarget",offFromTarget);

			SmartDashboard::PutNumber("Reset_Yaw_Count", reset_yaw_count);
			SmartDashboard::PutNumber("Reset_Displacement_Count", reset_displacement_count);
			SmartDashboard::PutBoolean("Yaw_Axis_Up",       yaw_axis_up);
			SmartDashboard::PutNumber( "Yaw_Axis", 			yaw_axis);

			SmartDashboard::PutBoolean( "IMU_Connected", imu->IsConnected());
			SmartDashboard::PutNumber("IMU_Yaw", imu->GetYaw());
			SmartDashboard::PutNumber("IMU_Pitch", imu->GetPitch());
			SmartDashboard::PutNumber("IMU_Roll", imu->GetRoll());
			SmartDashboard::PutNumber("IMU_CompassHeading", imu->GetCompassHeading());
			SmartDashboard::PutNumber("IMU_Update_Count", imu->GetUpdateCount());
			SmartDashboard::PutNumber("IMU_Byte_Count", imu->GetByteCount());

			/* These functions are compatible w/the WPI Gyro Class */
			SmartDashboard::PutNumber(   "IMU_TotalYaw", 		imu->GetAngle());
			SmartDashboard::PutNumber(   "IMU_YawRateDPS",		imu->GetRate());

			SmartDashboard::PutNumber("IMU_Accel_X", imu->GetWorldLinearAccelX());
			SmartDashboard::PutNumber("IMU_Accel_Y", imu->GetWorldLinearAccelY());
			SmartDashboard::PutBoolean("IMU_IsMoving", imu->IsMoving());
			SmartDashboard::PutNumber("IMU_Temp_C", imu->GetTempC());
			SmartDashboard::PutBoolean("IMU_IsCalibrating", imu->IsCalibrating());

			SmartDashboard::PutNumber("Velocity_X",       	imu->GetVelocityX() );
			SmartDashboard::PutNumber("Velocity_Y",       	imu->GetVelocityY() );
			SmartDashboard::PutNumber("Displacement_X",     imu->GetDisplacementX() );
			SmartDashboard::PutNumber("Displacement_Y",     imu->GetDisplacementY() );

			SmartDashboard::PutBoolean("Reached_Target",       hasReachedTarget);

			//Wait(0.1);				// wait for a while
		}
	}

	void TeleopInit()
	{
		enteredTelopInit = true;

		//to disable nudge position control to start
		controlPosNudgeLeft->Disable();
		controlPosNudgeRight->Disable();
		nudgeRight = false;
		nudgeLeft = false;
	}

	void TeleopPeriodic()
	{
		//check for nudge button presses
		if((steeringWheel->GetRawButton(DRIVE_NUDGE_LEFT_BUTTON)) && (!nudgeLeft && !nudgeRight))
		{
			nudgeLeft = true;
			nudgeRight = false;
		}
		if((steeringWheel->GetRawButton(DRIVE_NUDGE_RIGHT_BUTTON)) && (!nudgeLeft && !nudgeRight))
		{
			nudgeLeft = false;
			nudgeRight = true;
		}

		if(nudgeLeft)
		{
			sprintf(myString, "NL: %d|%d\n", myOnTarget(controlPosNudgeLeft, leftEncoder),
					myOnTarget(controlPosNudgeRight, rightEncoder));
			SmartDashboard::PutString("DB/String 7", myString);

			if(!(controlPosNudgeLeft->IsEnabled()))
			{
				driveSystem->SetPIDDrive(false); //disabling main velocity controlled drive system
				controlPosNudgeLeft->Enable();
				controlPosNudgeLeft->SetSetpoint(-NUDGE_MOVE_DIST);
				controlPosNudgeRight->Enable();
				controlPosNudgeRight->SetSetpoint(-NUDGE_MOVE_DIST);
			}

			if ((myOnTarget(controlPosNudgeLeft, leftEncoder)) || (myOnTarget(controlPosNudgeRight, rightEncoder)))
			{
				controlPosNudgeLeft->Disable();
				controlPosNudgeRight->Disable();
				nudgeLeft = false;
				driveSystem->SetPIDDrive(PID_CONFIG); //enabling main velocity controlled drive system
			}
		}
		else if(nudgeRight)
		{
			sprintf(myString, "NR: %d|%d\n", myOnTarget(controlPosNudgeLeft, leftEncoder),
					myOnTarget(controlPosNudgeRight, rightEncoder));
			SmartDashboard::PutString("DB/String 7", myString);
			if(!(controlPosNudgeLeft->IsEnabled()))
			{
				driveSystem->SetPIDDrive(false); //disabling main velocity controlled drive system
				controlPosNudgeLeft->Enable();
				controlPosNudgeLeft->SetSetpoint(NUDGE_MOVE_DIST);
				controlPosNudgeRight->Enable();
				controlPosNudgeRight->SetSetpoint(NUDGE_MOVE_DIST);
			}

			if ((myOnTarget(controlPosNudgeLeft, leftEncoder)) || (myOnTarget(controlPosNudgeRight, rightEncoder)))
			{
				controlPosNudgeLeft->Disable();
				controlPosNudgeRight->Disable();
				nudgeRight = false;
				driveSystem->SetPIDDrive(PID_CONFIG); //enabling main velocity controlled drive system
			}
		}
		else
		{
			//get driver controls values
			driveX = -steeringWheel->GetX();
			driveY = driveJoystick->GetY();
			sprintf(myString, "J: %5.3f|%5.3f\n", driveX, driveY);
			SmartDashboard::PutString("DB/String 0", myString);

			//Filter deadband
			if (driveX > STEERING_DB_LOW && driveX < STEERING_DB_HIGH)
				driveX = ZERO_FL;
			if (driveY > DRIVE_DB_LOW && driveY < DRIVE_DB_HIGH)
				driveY = ZERO_FL;

			//map to velocity profile
			driveX = velocityProfileX(driveX);
			driveY = velocityProfileY(driveY);


			sprintf(myString, "D: %5.3f|%5.3f\n", driveX, driveY);
			SmartDashboard::PutString("DB/String 1", myString);

			//Give drive instructions
			driveSystem->SetDriveInstruction(driveY * MAX_RPS, driveX * MAX_RPS);
			driveSystem->Update();

			sprintf(myString, "SP: %5.3f|%5.3f\n", driveSystem->GetLeftPIDSetpoint(), driveSystem->GetRightPIDSetpoint());
			SmartDashboard::PutString("DB/String 2", myString);
			sprintf(myString, "Out: %5.3f|%5.3f\n", driveSystem->GetLeftPIDOutput(), driveSystem->GetRightPIDOutput());
			SmartDashboard::PutString("DB/String 3", myString);
			sprintf(myString, "F curr: %f\n", forkMotor->GetOutputCurrent());
			SmartDashboard::PutString("DB/String 4", myString);
			sprintf(myString, "L curr: %5.3f|%5.3f\n", liftMotorBack->GetOutputCurrent(), liftMotorBack->GetOutputCurrent());
			SmartDashboard::PutString("DB/String 5", myString);

			sprintf(myString, "end: %5.3f|%5.3f\n", leftEncoder->GetRate(), rightEncoder->GetRate());
			SmartDashboard::PutString("DB/String 6", myString);

	//		sprintf(myString, "curPos: %f\n", curLiftPos);
	//		SmartDashboard::PutString("DB/String 8", myString);
	//		sprintf(myString, "tarPos: %f\n", targetLiftPos);
	//		SmartDashboard::PutString("DB/String 9", myString);
		}

		//Give lift instructions
		liftSystem->Update();
	}

	void TestPeriodic()
	{
		//not used
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
#else
		leftDrive = new Victor(CHAN_LEFT_DRIVE);
		rightDrive = new Victor(CHAN_RIGHT_DRIVE);
#endif
		leftEncoder = new Encoder(CHAN_ENCODER_LEFT_A, CHAN_ENCODER_LEFT_B, false, Encoder::EncodingType::k4X);
		leftEncoder->SetDistancePerPulse(ENCODER_DIST_PER_PULSE);
        leftEncoder->SetPIDSourceParameter(leftEncoder->kDistance);
		rightEncoder = new Encoder(CHAN_ENCODER_RIGHT_A, CHAN_ENCODER_RIGHT_B, false, Encoder::EncodingType::k4X);
		rightEncoder->SetDistancePerPulse(ENCODER_DIST_PER_PULSE);
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
		forkMotor = new CANTalon(CHAN_FORK_MOTOR);
		liftMotorBack = new CANTalon(CHAN_LIFT_MOTOR_BACK);
		liftMotorFront = new CANTalon(CHAN_LIFT_MOTOR_FRONT);
		leftIntakeMotor = new CANTalon(CHAN_L_INTAKE_MOTOR);
		rightIntakeMotor = new CANTalon(CHAN_R_INTAKE_MOTOR);
#else
		forkMotor = new CANJaguar(CHAN_FORK_MOTOR);
		liftMotorBack = new CANJaguar(CHAN_LIFT_MOTOR_BACK);
		liftMotorFront = new CANJaguar(CHAN_LIFT_MOTOR_FRONT);
		leftIntakeMotor = new CANJaguar(CHAN_L_INTAKE_MOTOR);
		rightIntakeMotor = new CANJaguar(CHAN_R_INTAKE_MOTOR);
#endif
		forkLimitInner = new DigitalInput(CHAN_FORK_MIN_LS);
		forkLimitOuter = new DigitalInput(CHAN_FORK_MAX_LS);
		liftLimitLow = new DigitalInput(CHAN_LIFT_LOW_LS);
		liftLimitHigh = new DigitalInput(CHAN_LIFT_HIGH_LS);
		liftEncoder = new Encoder(CHAN_LIFT_ENCODER_A, CHAN_LIFT_ENCODER_B, false, Encoder::EncodingType::k4X);
		liftEncoder->SetDistancePerPulse(LIFT_ENCODER_DIST_PER_PULSE);
#if BUILD_VER == COMPETITION
		liftEncoder->SetReverseDirection(true);
#endif
		liftEncoder->SetPIDSourceParameter(liftEncoder->kDistance);
		liftEncoder->Reset(); //zero at the starting position
		controlLiftBack = new PIDController(LIFT_PROPORTIONAL_TERM, LIFT_INTEGRAL_TERM, LIFT_DIFFERENTIAL_TERM, liftEncoder, liftMotorBack);
		controlLiftBack->SetContinuous(true); //treat input to controller as continuous; true by default
		controlLiftBack->SetOutputRange(LIFT_PID_OUT_MIN, LIFT_PID_OUT_MAX);
		controlLiftBack->Disable(); //hold position is off at initialization
		controlLiftFront = new PIDController(LIFT_PROPORTIONAL_TERM, LIFT_INTEGRAL_TERM, LIFT_DIFFERENTIAL_TERM, liftEncoder, liftMotorFront);
		controlLiftFront->SetContinuous(true); //treat input to controller as continuous; true by default
		controlLiftFront->SetOutputRange(LIFT_PID_OUT_MIN, LIFT_PID_OUT_MAX);
		controlLiftFront->Disable(); //hold position is off at initialization

		liftSysJoystick = new Joystick(CHAN_LIFT_SYS_JS);
		liftSystem = new LiftSystem(forkMotor, liftMotorBack, liftMotorFront, leftIntakeMotor, rightIntakeMotor,
				forkLimitInner, forkLimitOuter, liftLimitLow, liftLimitHigh,
				liftEncoder, controlLiftBack, controlLiftFront,
				liftSysJoystick);

		// Linear Drive PID controllers for Autonomous
        controlPosLeft = new PIDController(POS_PROPORTIONAL_TERM, POS_INTEGRAL_TERM, POS_DIFFERENTIAL_TERM, leftEncoder, leftDrive);
        controlPosLeft->SetContinuous(true);
        controlPosLeft->SetOutputRange(AUTONOMOUS_MAX_REVERSE_SPEED, AUTONOMOUS_MAX_FORWARD_SPEED);
        controlPosRight = new PIDController(POS_PROPORTIONAL_TERM, POS_INTEGRAL_TERM, POS_DIFFERENTIAL_TERM, rightEncoder, rightDrive);
        controlPosRight->SetContinuous(true);
        controlPosRight->SetOutputRange(AUTONOMOUS_MAX_REVERSE_SPEED, AUTONOMOUS_MAX_FORWARD_SPEED);
        autonomousDriveState = start;
	}

	~Robot()
	{
		//drive system
		delete leftDrive;
		delete rightDrive;
		delete leftEncoder;
		delete rightEncoder;
		if(enteredTelopInit)
			delete driveSystem;
		// Linear Drive PID controllers for nudging
		delete controlPosNudgeLeft;
		delete controlPosNudgeRight;

		//driver controls
		delete driveJoystick;
		delete steeringWheel;

		//lift system
		delete forkMotor;
		delete liftMotorBack;
		delete liftMotorFront;
		delete leftIntakeMotor;
		delete rightIntakeMotor;
		delete forkLimitInner;
		delete forkLimitOuter;
		delete liftLimitLow;
		delete liftLimitHigh;
		delete liftEncoder;
		delete controlLiftBack;
		delete controlLiftFront;
		delete liftSysJoystick;
		delete liftSystem;

		// Linear Drive PID controllers for autonomous
		delete controlPosLeft;
		delete controlPosRight;
	}
};

START_ROBOT_CLASS(Robot);
