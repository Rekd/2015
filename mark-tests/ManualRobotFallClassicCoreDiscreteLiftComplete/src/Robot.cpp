#include "math.h"
#include "WPILib.h"
#include "Constants.h"
#include "LiftSystem.h"
#include "DriveSystem.cpp"
#include "IMU.h"
#include "IMUAdvanced.h"
#include "AHRS.h"

char myString[64]; //for debugging

//local functions
bool myOnTarget(PIDController *controller, PIDSource *source)
{
	float error = controller->GetSetpoint() - source->PIDGet();
	sprintf(myString, "N: %5.2f|%5.2f\n", controller->GetSetpoint(), source->PIDGet());
	SmartDashboard::PutString("DB/String 7", myString);
//	float tolerance = error * POS_ERR_TOL; //any percentage value
//	return (fabs(tolerance) < POS_TOL_COMP); // a tuned value
	return (fabs(error) < POS_ERR_TOL);
}

//function to determine if have reached a control setpoint (target)
//used by PID controllers for nudging
bool nudgeOnTarget(PIDController *controller, PIDSource *source)
{
	float error = controller->GetSetpoint() - source->PIDGet();
	return (fabs(error) < NUDGE_PID_ERR_TOL);
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
	float rightIntakeMaxCurr, leftIntakeMaxCurr;
	float rightIntakeCurr, leftIntakeCurr;

	// Linear Drive PID controllers for Autonomous
	PIDController *controlPosRight;
	PIDController *controlPosLeft;
	// Rotation PID controllers
	PIDController *controlRotRight;
	PIDController *controlRotLeft;

	/*
	 * These are the autonomous states:
	 * start:  Starting condition, start intakes in
	 * move_1: move forward injesting the 1st tote
	 * rot_1: knock the 1st recycle container away while picking up tote, intake out
	 * rot_2: point back at totes, intakes in.
	 * move_2: move forward injecting the 2nd tote
	 * rot_3: knock the 2nd recycle container away while picking up tote, intakes out
	 * rot_4: point back at the last tote, intakes in
	 * move_3: move forward injecting the 3rd tote
	 * rot_5: rotate towards the auto zone while picking up the tote, stop intakes
	 * move_4:  move to the auto zone
	 * rot_6: rotate 90 deg while lowering lift to "low" position
	 * move_5:  start intakes out, eject totes while moving backwards
	 * end:  stop motion
	 */
		enum {start, move_0, rot_0, move_1, rot_1, rot_2, move_2, rot_3, rot_4,
			move_3, rot_5, move_4, rot_6, move_5, end} autonomousDriveState;
		enum commandLiftDir {raise, lower} ;
		float target;

		NetworkTable *table;
		SerialPort *serial_port;
		IMU *imu;
		bool first_iteration;

	void RobotInit()
	{
//		CameraServer::GetInstance()->SetQuality(50);
		//the camera name (e.g. "cam0") can be found through the roborio web interface
//		CameraServer::GetInstance()->StartAutomaticCapture("cam1");
	}

	void AutonomousInit()
	{
#if AUTONOMOUS_ON
		autonomousDriveState = start;
		imu->ZeroYaw();
		target = 0.0;
#endif
	}

	void outputAutoStatus(const char* state)
	{
		SmartDashboard::PutString("State:", state);
		SmartDashboard::PutNumber("Left Setpoint: ", controlPosLeft->GetSetpoint());
		SmartDashboard::PutNumber("Left Encoder: ", leftEncoder->PIDGet());
		SmartDashboard::PutNumber("Right Setpoint", controlPosRight->GetSetpoint());
		SmartDashboard::PutNumber("Right Encoder: ", rightEncoder->PIDGet());
	}

	void outputAutoRotStatus(const char* state, float target)
	{
		SmartDashboard::PutString("State:", state);
		SmartDashboard::PutNumber("target", target);
		SmartDashboard::PutNumber("offFromTarget", (imu->GetYaw() - target));
		SmartDashboard::PutNumber("IMU_Yaw", imu->GetYaw());
	}

	void rotLeft30(float *target)
	{
		//liftSystem->IntakesOut(); // start intakes rolling out
		liftSystem->IntakesOff(); // stop intakes
		controlPosLeft->Disable();  // disable the drive PID controllers
		controlPosRight->Disable();
		*target = -30.0;
		controlRotLeft->SetSetpoint(*target);
		controlRotRight->SetSetpoint(*target);
		controlRotLeft->Enable(); // enable the rotation PID controllers
		controlRotRight->Enable();
		return;
	}

	void rotRight30(float *target)
	{
		//liftSystem->IntakesOff(); // stop intakes
		//controlRotLeft->Disable();
		//controlRotRight->Disable();
		*target = 0.0f;
		controlRotLeft->SetSetpoint(*target);
		controlRotRight->SetSetpoint(*target);
		//controlRotLeft->Enable();
		//controlRotRight->Enable();
		return;
	}

	void rotRight90(commandLiftDir dir, float *target)
	{
		liftSystem->IntakesOff(); // stop intakes
		if (dir == raise)
		{
			liftSystem->setLiftStatePickup();
			*target = 90.0f;

		}
		else
		{
			liftSystem->setLiftStateLow();
// not right			*target = 179.0f; //set to a number <180 to avoid rollovers
			*target = 180.0f - imu->GetYaw();
			*target = 90.0f;  // temporary
			imu->ZeroYaw();  // temporary

		}

		controlPosLeft->Disable();  // disable the drive PID controllers
		controlPosRight->Disable();
		controlRotLeft->SetSetpoint(*target);
		controlRotRight->SetSetpoint(*target);
		controlRotLeft->Enable();
		controlRotRight->Enable();
		return;
	}

	void moveToNextTote(float dist)
	{
		liftSystem->IntakesIn(); // start intakes in
		controlRotLeft->Disable();  // disable the drive PID controllers
		controlRotRight->Disable();
		leftEncoder->Reset();
		rightEncoder->Reset();
		controlPosLeft->SetSetpoint(-dist);
		controlPosRight->SetSetpoint(dist);
		controlPosLeft->Enable();
		controlPosRight->Enable();
		return;
	}

	void AutonomousPeriodic()
	{
#if AUTONOMOUS_ON
		if(!(liftSystem->IsLiftInitDone()))
			liftSystem->Update(); //perform lift initialization
		else
		{
			// Autonomous state machine
			switch(autonomousDriveState)
			{
				case start:
#if 0
					moveToNextTote(AUTONOMOUS_MOVE_0_DIST);
					autonomousDriveState = move_0;
#endif
// temporarily only move into auto zone
					controlRotLeft->Disable();  // disable the drive PID controllers
					controlRotRight->Disable();
					leftEncoder->Reset();
					rightEncoder->Reset();
					controlPosLeft->SetSetpoint(-AUTONOMOUS_MOVE_4_DIST/2.0);  // move forwards
					controlPosRight->SetSetpoint(AUTONOMOUS_MOVE_4_DIST/2.0);  // change back to 4 later
					controlPosLeft->Enable();
					controlPosRight->Enable();
					autonomousDriveState = move_4;
					break;

				case move_0:
					outputAutoStatus("move_0");
					if ((myOnTarget(controlPosLeft, leftEncoder)) || (myOnTarget(controlPosRight, rightEncoder)))
					{
						//move this into a function later
						liftSystem->IntakesOff(); // stop intakes
						controlPosLeft->Disable();  // disable the drive PID controllers
						controlPosRight->Disable();
						target = 35.0; //deg
						controlRotLeft->SetSetpoint(target);
						controlRotRight->SetSetpoint(target);
						controlRotLeft->Enable(); // enable the rotation PID controllers
						controlRotRight->Enable();
						autonomousDriveState = rot_0;
					}
					break;

				case rot_0:
					outputAutoRotStatus("rot_0", target);
					if ((myOnTarget(controlRotLeft, imu)) || (myOnTarget(controlRotRight, imu)))
					{
						moveToNextTote(AUTONOMOUS_MOVE_1_DIST);
						autonomousDriveState = move_1;
						imu->ZeroYaw(); //this is a hack so that the existing rotate functions (e.g. rotLeft30) will continue to work
					}
					break;

				case move_1:
					outputAutoStatus("move_1");
					if ((myOnTarget(controlPosLeft, leftEncoder)) || (myOnTarget(controlPosRight, rightEncoder)))
					{
						rotLeft30(&target);
						autonomousDriveState = rot_1;
					}
					break;

				case rot_1:
					outputAutoRotStatus("rot_1", target);
					if ((myOnTarget(controlRotLeft, imu)) || (myOnTarget(controlRotRight, imu)))
					{
						rotRight30(&target);
						liftSystem->setLiftStatePickup();
						autonomousDriveState = rot_2;
					}
					break;

				case rot_2:
					outputAutoRotStatus("rot_2", target);
					if ((myOnTarget(controlRotLeft, imu)) || (myOnTarget(controlRotRight, imu)))
					{
						moveToNextTote(AUTONOMOUS_MOVE_2_DIST);
						autonomousDriveState = move_2;
					}
					break;

				case move_2:
					outputAutoStatus("move_2");
					if ((myOnTarget(controlPosLeft, leftEncoder)) || (myOnTarget(controlPosRight, rightEncoder)))
					{
						rotLeft30(&target);
						autonomousDriveState = rot_3;
					}
					break;

				case rot_3:
					outputAutoRotStatus("rot_3", target);
					if ((myOnTarget(controlRotLeft, imu)) || (myOnTarget(controlRotRight, imu)))
					{
						rotRight30(&target);
						liftSystem->setLiftStatePickup();
						autonomousDriveState = rot_4;
					}
					break;

				case rot_4:
					outputAutoRotStatus("rot_4", target);
					if ((myOnTarget(controlRotLeft, imu)) || (myOnTarget(controlRotRight, imu)))
					{
						moveToNextTote(AUTONOMOUS_MOVE_3_DIST);
						autonomousDriveState = move_3;
					}
					break;

				case move_3:
					outputAutoStatus("move_3");
					if ((myOnTarget(controlPosLeft, leftEncoder)) || (myOnTarget(controlPosRight, rightEncoder)))
					{
						rotRight90(raise, &target); // rotate 90 deg and do pickup
						autonomousDriveState = rot_5;
					}
					break;

				case rot_5:
					outputAutoRotStatus("rot_5", target);
					if ((myOnTarget(controlRotLeft, imu)) || (myOnTarget(controlRotRight, imu)))
					{
						controlRotLeft->Disable();  // disable the drive PID controllers
						controlRotRight->Disable();
						leftEncoder->Reset();
						rightEncoder->Reset();
						controlPosLeft->SetSetpoint(-AUTONOMOUS_MOVE_4_DIST);  // move forwards
						controlPosRight->SetSetpoint(AUTONOMOUS_MOVE_4_DIST);
						controlPosLeft->Enable();
						controlPosRight->Enable();
						autonomousDriveState = move_4;
					}
					break;

				case move_4:
					outputAutoStatus("move_4");
					if ((myOnTarget(controlPosLeft, leftEncoder)) || (myOnTarget(controlPosRight, rightEncoder)))
					{
						controlPosLeft->Disable(); // temp
						controlPosRight->Disable();  // temp
						rotRight90(lower, &target); // rotate 90 deg and lower stack
						autonomousDriveState = rot_6;
					}
					break;

				case rot_6:
					outputAutoRotStatus("rot_6", target);
					if ((myOnTarget(controlRotLeft, imu)) || (myOnTarget(controlRotRight, imu)))
					{
						//liftSystem->IntakesOut();  // eject the stack and back up - temp
						liftSystem->IntakesOff();
						controlRotLeft->Disable();  // disable the drive PID controllers
						controlRotRight->Disable();
						leftEncoder->Reset();
						rightEncoder->Reset();
						controlPosLeft->SetSetpoint(AUTONOMOUS_MOVE_5_DIST);  // move backwards
						controlPosRight->SetSetpoint(-AUTONOMOUS_MOVE_5_DIST);
						//controlPosLeft->Enable();
						// controlPosRight->Enable();
						controlPosLeft->Disable(); // temp
						controlPosRight->Disable();  // temp
						autonomousDriveState = move_5;
						autonomousDriveState = end; // temp
					}
					break;

				case move_5:
					outputAutoStatus("move_5");
					if ((myOnTarget(controlPosLeft, leftEncoder)) || (myOnTarget(controlPosRight, rightEncoder)))
					{
						liftSystem->IntakesOff();
						controlPosLeft->Disable(); // stop all motion
						controlPosRight->Disable();
						liftSystem->setLiftStateHigh();
						autonomousDriveState = end;
					}
					break;

				case end:
					outputAutoStatus("end");
					break;  // do nothing
			} //end switch on autonomous state

			liftSystem->Update();
		} //end check for lift init
#endif
	} //end autonomous periodic

	void TeleopInit()
	{
		enteredTelopInit = true;
		driveSystem = new DriveSystem(leftEncoder, rightEncoder, leftDrive, rightDrive);

		controlPosLeft->Disable(); // stop all motion
		controlPosRight->Disable();
		controlRotLeft->Disable();  // disable the drive PID controllers
		controlRotRight->Disable();

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
		leftIntakeMaxCurr = 0.0;
		rightIntakeMaxCurr = 0.0;
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
				controlPosNudgeRight->SetSetpoint(NUDGE_MOVE_DIST);  // was negative - not working
			}
			//determine if the nudge is complete
			if ((nudgeOnTarget(controlPosNudgeLeft, leftEncoder)) || (nudgeOnTarget(controlPosNudgeRight, rightEncoder)))
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
				controlPosNudgeRight->SetSetpoint(-NUDGE_MOVE_DIST);  // was positive, not working
			}
			//determine if the nudge is complete
			if ((nudgeOnTarget(controlPosNudgeLeft, leftEncoder)) || (nudgeOnTarget(controlPosNudgeRight, rightEncoder)))
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

		// show current data
		leftIntakeCurr = leftIntakeMotor->GetOutputCurrent();
		rightIntakeCurr = rightIntakeMotor->GetOutputCurrent();
		if (leftIntakeCurr > leftIntakeMaxCurr)
			leftIntakeMaxCurr = leftIntakeCurr;
		if (rightIntakeCurr > rightIntakeMaxCurr)
			rightIntakeMaxCurr = rightIntakeCurr;
		SmartDashboard::PutNumber("Left intake:",leftIntakeCurr);
		SmartDashboard::PutNumber("Right intake:",rightIntakeCurr);
		SmartDashboard::PutNumber("Left intake max:",leftIntakeMaxCurr);
		SmartDashboard::PutNumber("Right intake max:",rightIntakeMaxCurr);
		SmartDashboard::PutNumber("Lift B:",liftMotorBack->GetOutputCurrent());
		SmartDashboard::PutNumber("Lift F:",liftMotorFront->GetOutputCurrent());
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
//			Wait( 0.3 );
			imu->ZeroYaw();
			first_iteration = false;
		}
		imu->ZeroYaw();

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
		controlRotLeft->Disable();
		LiveWindow::GetInstance()->AddSensor("L RD", "L RDrive", controlRotLeft);

		controlRotRight = new PIDController(ROT_PROPORTIONAL_TERM, ROT_INTEGRAL_TERM,
				ROT_DIFFERENTIAL_TERM, 0, imu, rightDrive);
		controlRotRight->SetContinuous(false);
		controlRotRight->SetInputRange(-180.0, 180.0);
		controlRotRight->SetOutputRange(AUTONOMOUS_MAX_REVERSE_SPEED, AUTONOMOUS_MAX_FORWARD_SPEED);
		controlRotRight->Disable();
		LiveWindow::GetInstance()->AddSensor("R RD", "R RDrive", controlRotRight);
        autonomousDriveState = start;
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

		// Linear Drive PID controllers for Autonomous
		delete controlPosRight;
		delete controlPosLeft;
		delete controlRotRight;
		delete controlRotLeft;
	}
};

START_ROBOT_CLASS(Robot);
