#include "WPILib.h"
#include "Constants.h"
#include "LiftSystem.h"
#include "DriveSystem.cpp"
#include "math.h"
#include "IMU.h"
#include "IMUAdvanced.h"
#include "AHRS.h"

#define MAX_SPEED 0.35
#define MAX_REV_SPEED -0.35

char myString[64]; //for debugging

//local function
bool myOnTarget(PIDController *controller, PIDSource *source)
{
	float error = controller->GetSetpoint() - source->PIDGet();
	sprintf(myString, "N: %5.2f|%5.2f\n", controller->GetSetpoint(), source->PIDGet());
	SmartDashboard::PutString("DB/String 7", myString);
//	float tolerance = error * POS_ERR_TOL; //any percentage value
//	return (fabs(tolerance) < POS_TOL_COMP); // a tuned value
	return (fabs(error) < POS_ERR_TOL);
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
private:
	//memory management
	bool enteredTelopInit; //drive system is created in teleop init so only delete the drive system in the destructor if entered teleop init

	//drive system
#if BUILD_VER == COMPETITION || BUILD_VER == PARADE
	Talon * leftDrive;
	Talon * rightDrive;
#else //practice
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
#if BUILD_VER == COMPETITION || BUILD_VER == PRACTICE
	Joystick *steeringWheel;
#endif
	float driveX, driveY; //values from driver controls: x from steeringWheel, y from driveJoystick
	bool drivePID;

	//drive nudge flags
	bool nudgeLeft, nudgeRight;

	//lift system
#if BUILD_VER == COMPETITION || BUILD_VER == PARADE
	CANTalon *forkMotor;
	CANTalon *liftMotorBack;
	CANTalon *liftMotorFront;
	CANTalon *leftIntakeMotor;
	CANTalon *rightIntakeMotor;
#else //practice
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
	// Rotation PID controllers
	PIDController *controlRotRight;
	PIDController *controlRotLeft;
	enum {start, move_1, rot_1, rot_2, move_2, rot_3} autonomousDriveState;

	NetworkTable *table;
//	AHRS *imu;
	IMU *imu;
	SerialPort *serial_port;
	bool first_iteration;

	void RobotInit()
	{
		CameraServer::GetInstance()->SetQuality(50);
		//the camera name (ex "cam0") can be found through the roborio web interface
		CameraServer::GetInstance()->StartAutomaticCapture("cam1");
	}

	void AutonomousInit()
	{
#if BUILD_VER == COMPETITION || BUILD_VER == PRACTICE
		liftSystem->StartForksInAuto();  // start the forks moving in
		autonomousDriveState = start;
#endif
		imu->ZeroYaw();
	}

	void AutonomousPeriodic()
	{
#if BUILD_VER == COMPETITION || BUILD_VER == PRACTICE
		char myString[64];
		float curLiftPos;
		float targetLiftPos;
		bool hasReachedTarget = false;
		float target = 0.0;

//		liftSystem->UpdateAuto();

		if (autonomousDriveState == start)
		{
				controlPosLeft->SetSetpoint(-AUTONMOUS_MOVE_DIST);
				controlPosRight->SetSetpoint(AUTONMOUS_MOVE_DIST);
				controlPosLeft->Enable();
				controlPosRight->Enable();
				autonomousDriveState = move_1;
		}
#endif
		if (autonomousDriveState == move_1)
		{
    		sprintf(myString, "Driving\n");
    		SmartDashboard::PutString("DB/String 0", myString);
    		sprintf(myString, "L Setpoint: %f\n", controlPosLeft->GetSetpoint());
    		SmartDashboard::PutString("DB/String 6", myString);
       		sprintf(myString, "L PID: %f\n", leftEncoder->PIDGet());
    		SmartDashboard::PutString("DB/String 7", myString);
    		sprintf(myString, "R Setpoint: %f\n", controlPosRight->GetSetpoint());
    		SmartDashboard::PutString("DB/String 8", myString);
       		sprintf(myString, "R PID: %f\n", rightEncoder->PIDGet());
    		SmartDashboard::PutString("DB/String 9", myString);

			if ((myOnTarget(controlPosLeft, leftEncoder)) && (myOnTarget(controlPosRight, rightEncoder)))
			{
				controlPosLeft->Disable();  // disable the drive PID controllers
				controlPosRight->Disable();
				target = -30.0;
				controlRotLeft->SetSetpoint(target);
				controlRotRight->SetSetpoint(target);
				controlRotLeft->Enable(); // enable the rotation PID controllers
				controlRotRight->Enable();
				autonomousDriveState = rot_1;
			}
		}
		if (autonomousDriveState == rot_1)
		{
			SmartDashboard::PutString("Auto mode", "rot_1");
			SmartDashboard::PutNumber("target", target);
			SmartDashboard::PutNumber("offFromTarget", (imu->GetYaw() - target));
			SmartDashboard::PutNumber("IMU_Yaw", imu->GetYaw());

			if ((myOnTarget(controlRotLeft, imu)) && (myOnTarget(controlRotRight, imu)))
			{
				//controlRotLeft->Disable();
				//controlRotRight->Disable();
				target = 0.0f;
				controlRotLeft->SetSetpoint(target);
				controlRotRight->SetSetpoint(target);
				//controlRotLeft->Enable();
				//controlRotRight->Enable();
				autonomousDriveState = rot_2;
			}
		}
		if (autonomousDriveState == rot_2)
		{
			SmartDashboard::PutString("Auto mode", "rot_2");
			SmartDashboard::PutNumber("target", target);
			SmartDashboard::PutNumber("offFromTarget", (imu->GetYaw() - target));
			SmartDashboard::PutNumber("IMU_Yaw", imu->GetYaw());

			if ((myOnTarget(controlRotLeft, imu)) && (myOnTarget(controlRotRight, imu)))
			{
				controlRotLeft->Disable();  // disable the drive PID controllers
				controlRotRight->Disable();
				leftEncoder->Reset();
				rightEncoder->Reset();
				controlPosLeft->SetSetpoint(-AUTONMOUS_MOVE_DIST);
				controlPosRight->SetSetpoint(AUTONMOUS_MOVE_DIST);
				controlPosLeft->Enable();
				controlPosRight->Enable();
				autonomousDriveState = move_2;
			}
		}
		if (autonomousDriveState == move_2)
		{
    		sprintf(myString, "Driving\n");
    		SmartDashboard::PutString("DB/String 0", myString);
    		sprintf(myString, "L Setpoint: %f\n", controlPosLeft->GetSetpoint());
    		SmartDashboard::PutString("DB/String 6", myString);
       		sprintf(myString, "L PID: %f\n", leftEncoder->PIDGet());
    		SmartDashboard::PutString("DB/String 7", myString);
    		sprintf(myString, "R Setpoint: %f\n", controlPosRight->GetSetpoint());
    		SmartDashboard::PutString("DB/String 8", myString);
       		sprintf(myString, "R PID: %f\n", rightEncoder->PIDGet());
    		SmartDashboard::PutString("DB/String 9", myString);

			if ((myOnTarget(controlPosLeft, leftEncoder)) && (myOnTarget(controlPosRight, rightEncoder)))
			{
				controlPosLeft->Disable();  // disable the drive PID controllers
				controlPosRight->Disable();
//				target = -30.0f;
//				controlRotLeft->SetSetpoint(target);
//				controlRotRight->SetSetpoint(target);
//				controlRotLeft->Enable(); // enable the rotation PID controllers
//				controlRotRight->Enable();
				autonomousDriveState = rot_3;
			}
		}
	}

	void TeleopInit()
	{
		controlPosLeft->Disable();
		controlPosRight->Disable();
		enteredTelopInit = true;
		driveSystem = new DriveSystem(leftEncoder, rightEncoder, leftDrive, rightDrive);

		//to disable nudge position control to start
		controlPosNudgeLeft->Disable();
		controlPosNudgeRight->Disable();
		nudgeRight = false;
		nudgeLeft = false;

		//Initialize PID
		driveSystem->SetPIDDrive(PID_CONFIG);
		drivePID = true;
		//Set Wheel Diameter
		driveSystem->SetWheelDiameter(WHEEL_DIAMETER);
		// start recording the distance traveled
		driveSystem->StartRecordingDistance();
	}

	void TeleopPeriodic()
	{
		//check for PID button on/off
		//By default, PID is true at startup.
		if(driveJoystick->GetRawButton(DRIVE_PID_OFF_BUTTON))
		{
			driveSystem->SetPIDDrive(false);
			drivePID = false;
		}
		else if(driveJoystick->GetRawButton(DRIVE_PID_ON_BUTTON))
		{
			driveSystem->SetPIDDrive(true);
			drivePID = true;
		}

#if BUILD_VER == COMPETITION || BUILD_VER == PRACTICE
		//check for nudge button presses
		if((steeringWheel->GetRawButton(DRIVE_NUDGE_WHEEL_LEFT_BUTTON)) && (!nudgeLeft && !nudgeRight))
		{
			nudgeLeft = true;
			nudgeRight = false;
		}
		if((steeringWheel->GetRawButton(DRIVE_NUDGE_WHEEL_RIGHT_BUTTON)) && (!nudgeLeft && !nudgeRight))
		{
			nudgeLeft = false;
			nudgeRight = true;
		}
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
			sprintf(myString, "NL: %d|%d\n", myOnTarget(controlPosNudgeLeft, leftEncoder),
					myOnTarget(controlPosNudgeRight, rightEncoder));
			SmartDashboard::PutString("DB/String 7", myString);

			sprintf(myString, "SP: %5.3f|%5.3f\n", controlPosNudgeLeft->GetSetpoint(), controlPosNudgeRight->GetSetpoint());
			SmartDashboard::PutString("DB/String 2", myString);
			sprintf(myString, "Out: %5.3f|%5.3f\n", controlPosNudgeLeft->Get(), controlPosNudgeRight->Get());
			SmartDashboard::PutString("DB/String 3", myString);


			if(!(controlPosNudgeLeft->IsEnabled()))
			{
				driveSystem->SetPIDDrive(false); //disabling main velocity controlled drive system
		        leftEncoder->SetPIDSourceParameter(leftEncoder->kDistance);
		        rightEncoder->SetPIDSourceParameter(rightEncoder->kDistance);
				controlPosNudgeLeft->Enable();
				controlPosNudgeLeft->SetSetpoint(-NUDGE_MOVE_DIST);
				controlPosNudgeRight->Enable();
				controlPosNudgeRight->SetSetpoint(-NUDGE_MOVE_DIST);
			}

			if ((myOnTarget(controlPosNudgeLeft, leftEncoder)) || (myOnTarget(controlPosNudgeRight, rightEncoder)))
			{
				sprintf(myString, "On Target\n");
				SmartDashboard::PutString("DB/String 3", myString);

				controlPosNudgeLeft->Disable();
				controlPosNudgeRight->Disable();
				nudgeLeft = false;
				nudgeRight = false;
				if(drivePID)
				{
					driveSystem->SetPIDDrive(PID_CONFIG); //enabling main velocity controlled drive system
				}
			}
		}
		else if(nudgeRight)
		{
			sprintf(myString, "NR: %d|%d\n", myOnTarget(controlPosNudgeLeft, leftEncoder),
					myOnTarget(controlPosNudgeRight, rightEncoder));
			SmartDashboard::PutString("DB/String 7", myString);

			sprintf(myString, "SP: %5.3f|%5.3f\n", controlPosNudgeLeft->GetSetpoint(), controlPosNudgeRight->GetSetpoint());
			SmartDashboard::PutString("DB/String 2", myString);
			sprintf(myString, "Out: %5.3f|%5.3f\n", controlPosNudgeLeft->Get(), controlPosNudgeRight->Get());
			SmartDashboard::PutString("DB/String 3", myString);

			if(!(controlPosNudgeLeft->IsEnabled()))
			{
				driveSystem->SetPIDDrive(false); //disabling main velocity controlled drive system
		        leftEncoder->SetPIDSourceParameter(leftEncoder->kDistance);
		        rightEncoder->SetPIDSourceParameter(rightEncoder->kDistance);
				controlPosNudgeLeft->Enable();
				controlPosNudgeLeft->SetSetpoint(NUDGE_MOVE_DIST);
				controlPosNudgeRight->Enable();
				controlPosNudgeRight->SetSetpoint(NUDGE_MOVE_DIST);
			}

			if ((myOnTarget(controlPosNudgeLeft, leftEncoder)) || (myOnTarget(controlPosNudgeRight, rightEncoder)))
			{
				sprintf(myString, "On Target\n");
				SmartDashboard::PutString("DB/String 3", myString);
				controlPosNudgeLeft->Disable();
				controlPosNudgeRight->Disable();
				nudgeRight = false;
				nudgeLeft = false;
				if(drivePID)
				{
					driveSystem->SetPIDDrive(PID_CONFIG); //enabling main velocity controlled drive system
				}
			}
		}
#endif
#if BUILD_VER == PARADE
		if (false); //no nudging in parade
#endif
		else
		{
			//get driver controls values
#if BUILD_VER == COMPETITION || BUILD_VER == PRACTICE
			driveX = -steeringWheel->GetX();
#else //parade
			driveX = -driveJoystick->GetX();
#endif
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

//			sprintf(myString, "SP: %5.3f|%5.3f\n", driveSystem->GetLeftPIDSetpoint(), driveSystem->GetRightPIDSetpoint());
//			SmartDashboard::PutString("DB/String 2", myString);
//			sprintf(myString, "Out: %5.3f|%5.3f\n", driveSystem->GetLeftPIDOutput(), driveSystem->GetRightPIDOutput());
//			SmartDashboard::PutString("DB/String 3", myString);
			sprintf(myString, "PID: %d", drivePID);
			SmartDashboard::PutString("DB/String 4", myString);
			sprintf(myString, "L curr: %5.3f|%5.3f\n", liftMotorBack->GetOutputCurrent(), liftMotorBack->GetOutputCurrent());
			SmartDashboard::PutString("DB/String 5", myString);

			sprintf(myString, "enc: %5.3f|%5.3f\n", leftEncoder->GetRate(), rightEncoder->GetRate());
			SmartDashboard::PutString("DB/String 6", myString);

			sprintf(myString, "enc: %d|%d\n", leftEncoder->Get(), rightEncoder->Get());
			SmartDashboard::PutString("DB/String 8", myString);

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
        bool is_calibrating = imu->IsCalibrating();
		if ( !is_calibrating ) {
			Wait( 0.3 );
			imu->ZeroYaw();
			first_iteration = false;
		}
		imu->ZeroYaw();

		//drive system
#if BUILD_VER == COMPETITION || BUILD_VER == PARADE
		leftDrive = new Talon(CHAN_LEFT_DRIVE);
		rightDrive = new Talon(CHAN_RIGHT_DRIVE);
#else //practice
		leftDrive = new Victor(CHAN_LEFT_DRIVE);
		rightDrive = new Victor(CHAN_RIGHT_DRIVE);
#endif
		leftEncoder = new Encoder(CHAN_ENCODER_LEFT_A, CHAN_ENCODER_LEFT_B, false, Encoder::EncodingType::k4X);
		leftEncoder->Reset();
		leftEncoder->SetDistancePerPulse(ENCODER_DIST_PER_PULSE);
        leftEncoder->SetPIDSourceParameter(leftEncoder->kDistance);
		rightEncoder = new Encoder(CHAN_ENCODER_RIGHT_A, CHAN_ENCODER_RIGHT_B, false, Encoder::EncodingType::k4X);
		rightEncoder->Reset();
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
#if BUILD_VER == COMPETITION || BUILD_VER == PRACTICE
		steeringWheel = new Joystick(CHAN_STEERING_WHEEL);
#endif

		//lift system
#if BUILD_VER == COMPETITION || BUILD_VER == PARADE
		forkMotor = new CANTalon(CHAN_FORK_MOTOR);
		liftMotorBack = new CANTalon(CHAN_LIFT_MOTOR_BACK);
		liftMotorFront = new CANTalon(CHAN_LIFT_MOTOR_FRONT);
		leftIntakeMotor = new CANTalon(CHAN_L_INTAKE_MOTOR);
		rightIntakeMotor = new CANTalon(CHAN_R_INTAKE_MOTOR);
#else //practice
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
#if BUILD_VER == COMPETITION || BUILD_VER == PARADE
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

#if BUILD_VER == COMPETITION || BUILD_VER == PRACTICE
		liftSysJoystick = new Joystick(CHAN_LIFT_SYS_JS);
#else
		liftSysJoystick = driveJoystick;
#endif
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

		controlRotLeft = new PIDController(ROT_PROPORTIONAL_TERM, ROT_INTEGRAL_TERM,
				ROT_DIFFERENTIAL_TERM, 0, imu, leftDrive);
		controlRotLeft->SetContinuous(false);
		controlRotLeft->SetInputRange(-180.0, 180.0);
		controlRotLeft->SetOutputRange(AUTONOMOUS_MAX_REVERSE_SPEED, AUTONOMOUS_MAX_FORWARD_SPEED);
		LiveWindow::GetInstance()->AddSensor("L RD", "L RDrive", controlRotLeft);

		controlRotRight = new PIDController(ROT_PROPORTIONAL_TERM, ROT_INTEGRAL_TERM,
				ROT_DIFFERENTIAL_TERM, 0, imu, rightDrive);
		controlRotRight->SetContinuous(false);
		controlRotRight->SetInputRange(-180.0, 180.0);
		controlRotRight->SetOutputRange(AUTONOMOUS_MAX_REVERSE_SPEED, AUTONOMOUS_MAX_FORWARD_SPEED);
		LiveWindow::GetInstance()->AddSensor("R RD", "R RDrive", controlRotRight);
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
#if BUILD_VER == COMPETITION || BUILD_VER == PRACTICE
		delete steeringWheel;
#endif

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
#if BUILD_VER == COMPETITION || BUILD_VER == PRACTICE
		delete liftSysJoystick; //if parade, deletion of driveJoystick takes care of this
#endif
		delete liftSystem;

		// Linear Drive PID controllers for autonomous
		delete controlPosLeft;
		delete controlPosRight;
	}
};

START_ROBOT_CLASS(Robot);
