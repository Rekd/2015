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

#define MAX_SPEED 0.35
#define MAX_REV_SPEED -0.35


//local function
bool myOnTarget(PIDController *controller, PIDSource *source)
{
	float error = controller->GetSetpoint() - source->PIDGet();
	SmartDashboard::PutNumber("setpoint",controller->GetSetpoint());
	SmartDashboard::PutNumber("PIDget",source->PIDGet());
	SmartDashboard::PutNumber("error",error);
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
//	AHRS *imu;
	IMU *imu;
	SerialPort *serial_port;
	bool first_iteration;
	float turn_dir;

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
//	DriveSystem *driveSystem;

	// Linear Drive PID controllers for nudging
	PIDController *controlPosNudgeRight;
	PIDController *controlPosNudgeLeft;

	//driver controls
	Joystick *driveJoystick;
	Joystick *steeringWheel;
	float driveX, driveY; //values from driver controls: x from steeringWheel, y from driveJoystick

	//drive nudge flags
	bool nudgeLeft, nudgeRight;



	// Linear Drive PID controllers for Autonomous
	PIDController *controlPosRight;
	PIDController *controlPosLeft;
	enum {start, fork_in, drive, release, hold} autonomousDriveState;

	// Rotation PID controllers
	PIDController *controlRotRight;
	PIDController *controlRotLeft;

	void RobotInit()
	{
		//IMU init
		table = NetworkTable::GetTable("datatable");
		serial_port = new SerialPort(57600,SerialPort::kMXP);
        uint8_t update_rate_hz = 50;
 //       imu = new AHRS(serial_port,update_rate_hz);
        imu = new IMU(serial_port,update_rate_hz);
        if ( imu ) {
        	LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", imu);
        }
        first_iteration = true;

        //Camera
		CameraServer::GetInstance()->SetQuality(50);
		//the camera name (ex "cam0") can be found through the roborio web interface
		CameraServer::GetInstance()->StartAutomaticCapture("cam1");

	// instantiate and initialize the drive PID controllers
			controlRotLeft = new PIDController(ROT_PROPORTIONAL_TERM, ROT_INTEGRAL_TERM,
					ROT_DIFFERENTIAL_TERM, 0, imu, leftDrive);
	//        controlRotLeft->SetTolerance (2.0);
			controlRotLeft->SetContinuous(false);
			controlRotLeft->SetInputRange(-180.0, 180.0);
			controlRotLeft->SetOutputRange(MAX_REV_SPEED, MAX_SPEED);
			LiveWindow::GetInstance()->AddSensor("L RD", "L RDrive", controlRotLeft);
	//        controlLeft->Enable();

			controlRotRight = new PIDController(ROT_PROPORTIONAL_TERM, ROT_INTEGRAL_TERM,
					ROT_DIFFERENTIAL_TERM, 0, imu, rightDrive);
	//        controlRotRight->SetTolerance (2.0);
			controlRotRight->SetContinuous(false);
			controlRotRight->SetInputRange(-180.0, 180.0);
			controlRotRight->SetOutputRange(MAX_REV_SPEED, MAX_SPEED);
			LiveWindow::GetInstance()->AddSensor("R RD", "R RDrive", controlRotRight);

//		driveSystem = new DriveSystem(leftEncoder, rightEncoder, leftDrive, rightDrive);

		//Initialize PID
//		driveSystem->SetPIDDrive(PID_CONFIG);
		//Set Wheel Diameter
//		driveSystem->SetWheelDiameter(WHEEL_DIAMETER);
		// start recording the distance traveled
//		driveSystem->StartRecordingDistance();
	}

	void AutonomousInit()
	{

		controlRotLeft->Enable(); // enable the rotation PID controllers
		controlRotRight->Enable();
	}

	void AutonomousPeriodic()
	{

		int reset_yaw_count = 0;
		int reset_displacement_count = 0;

		bool hasReachedTarget = false;
		float target = 0.0;
		float autoSpeed = 0.0;
		float turnSpeed = 0.3;
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
				imu->ZeroYaw();
				turn_dir = -1;
			//	target = imu->GetAngle() + 30;
				target = imu->GetYaw() + 30;
				controlRotLeft->SetSetpoint(target);
				controlRotRight->SetSetpoint(target);

			}
			bool yaw_axis_up;
			uint8_t yaw_axis = imu->GetBoardYawAxis(yaw_axis_up);
#if 0
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
#endif

//			float offFromTarget = fabs(imu->GetAngle() - target);
			float offFromTarget = fabs(imu->GetYaw() - target);
/*
			if(offFromTarget < margin) {
				hasReachedTarget = true;
				driveSystem->SetDriveInstruction(0.0,0.0);
				driveSystem->Update();
				if (turn_dir<0) {
					target = imu->GetAngle() - 30;
					turn_dir = 1;
					hasReachedTarget = false;
				}
			}


			if(!hasReachedTarget) {
				driveSystem->SetDriveInstruction(autoSpeed * MAX_RPS, turn_dir * turnSpeed * MAX_RPS);
				driveSystem->Update();
			}
*/
			if ((myOnTarget(controlRotLeft, imu)) && (myOnTarget(controlRotRight, imu)))
			{
				hasReachedTarget = true;

				if (turn_dir<0)
				{
					target = imu->GetYaw() - 30;
					turn_dir = 1;
					hasReachedTarget = false;
					controlRotLeft->SetSetpoint(target);
					controlRotRight->SetSetpoint(target);
				}
				else
				{
					controlRotLeft->Disable();
					controlRotRight->Disable();
				}
				Wait( 0.4 );
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
//			SmartDashboard::PutNumber(   "IMU_TotalYaw", 		imu->GetAngle());
			SmartDashboard::PutNumber(   "IMU_TotalYaw", 		imu->GetYaw());
			SmartDashboard::PutNumber(   "IMU_YawRateDPS",		imu->GetRate());
#if 0
			SmartDashboard::PutNumber("IMU_Accel_X", imu->GetWorldLinearAccelX());
			SmartDashboard::PutNumber("IMU_Accel_Y", imu->GetWorldLinearAccelY());
			SmartDashboard::PutBoolean("IMU_IsMoving", imu->IsMoving());
			SmartDashboard::PutNumber("IMU_Temp_C", imu->GetTempC());
			SmartDashboard::PutBoolean("IMU_IsCalibrating", imu->IsCalibrating());

			SmartDashboard::PutNumber("Velocity_X",       	imu->GetVelocityX() );
			SmartDashboard::PutNumber("Velocity_Y",       	imu->GetVelocityY() );
			SmartDashboard::PutNumber("Displacement_X",     imu->GetDisplacementX() );
			SmartDashboard::PutNumber("Displacement_Y",     imu->GetDisplacementY() );
#endif
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
#if 0
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
#endif
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
//		if(enteredTelopInit)
//			delete driveSystem;
		// Linear Drive PID controllers for nudging
		delete controlPosNudgeLeft;
		delete controlPosNudgeRight;

		//driver controls
		delete driveJoystick;
		delete steeringWheel;

		// Linear Drive PID controllers for autonomous
		delete controlPosLeft;
		delete controlPosRight;
	}
};

START_ROBOT_CLASS(Robot);
