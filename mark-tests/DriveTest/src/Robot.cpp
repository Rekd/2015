#include "WPILib.h"
#include "Robot.h"
#include "Constants.h"


class Robot: public IterativeRobot
{
private:
	Joystick *joystick;
	Joystick *steeringWheel;

#if BUILD_VERSION == COMPETITION
	Talon * leftDrive;
	Talon * rightDrive;
#else
	Victor * leftDrive;
	Victor * rightDrive;
#endif

	Encoder *leftEncoder;
	Encoder *rightEncoder;
	DriveSystem *driveSystem;
	char myString[64];

	void RobotInit()
	{
	//Instantiate Joystick and Steering Wheel
		joystick = new Joystick(0);
		steeringWheel = new Joystick(1);

// Instantiate the drive controllers
#if BUILD_VERSION == COMPETITION
		leftDrive = new Talon(CHAN_LEFT_DRIVE_TALONSR);
		rightDrive = new Talon(CHAN_RIGHT_DRIVE_TALONSR);
#else
		leftDrive = new Victor(CHAN_LEFT_DRIVE_TALONSR);
		rightDrive = new Victor(CHAN_RIGHT_DRIVE_TALONSR);
#endif

// Instantiate and initialize the encoders
		leftEncoder = new Encoder(CHAN_ENCODER_LEFT_A, CHAN_ENCODER_LEFT_B, false, Encoder::EncodingType::k4X);

		rightEncoder = new Encoder(CHAN_ENCODER_RIGHT_A, CHAN_ENCODER_RIGHT_B, false, Encoder::EncodingType::k4X);


//Instantiate DriveSystem
		driveSystem = new DriveSystem(leftEncoder, rightEncoder, leftDrive, rightDrive);
//		driveSystem = new DriveSystem(leftDrive, rightDrive);
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{
		//Initialize PID
		driveSystem->SetPIDDrive(PID_CONFIG);

		//Set Wheel Diameter
		driveSystem->SetWheelDiameter(WHEEL_DIAMETER);

		// start recording the distance traveled
		driveSystem->StartRecordingDistance();
	}

	void TeleopPeriodic()
	{
		char myString[64];
		float x, y;

// Get the joystick and steering position
		x = -steeringWheel->GetX();
		y = joystick->GetY();

//Filter deadband
		if (x > -0.1 && x < 0.1)
			x = 0;

		if (y > -0.1 && y < 0.1)
			y = 0;

		sprintf(myString, "R enc cnt: %d\n", rightEncoder->Get());
		SmartDashboard::PutString("DB/String 0", myString);

		sprintf(myString, "dist: %f\n", driveSystem->GetDistanceTraveledFeet());
		SmartDashboard::PutString("DB/String 1", myString);

		sprintf(myString, "RPS: %f\n", driveSystem->GetRobotSpeedInRPS());
		SmartDashboard::PutString("DB/String 2", myString);

		sprintf(myString, "FPS: %f\n", driveSystem->GetRobotSpeedInFPS());
		SmartDashboard::PutString("DB/String 3", myString);

		sprintf(myString, "L PID output: %5.2f\n", driveSystem->GetLeftPIDOutput());
		SmartDashboard::PutString("DB/String 4", myString);

		sprintf(myString, "R PID output: %5.2f\n", driveSystem->GetRightPIDOutput());
		SmartDashboard::PutString("DB/String 5", myString);


		sprintf(myString, "L PID error: %5.2f\n", driveSystem->GetLeftPIDError());
		SmartDashboard::PutString("DB/String 6", myString);

		sprintf(myString, "R PID error: %5.2f\n", driveSystem->GetRightPIDError());
		SmartDashboard::PutString("DB/String 7", myString);

//Give drive instructions
		driveSystem->SetDriveInstruction(y * MAX_RPS, x * MAX_RPS);
		driveSystem->Update();

	}

	void TestPeriodic()
	{
	}
};

START_ROBOT_CLASS(Robot);
