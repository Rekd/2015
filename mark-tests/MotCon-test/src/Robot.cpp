#include "WPILib.h"

#define MAX_RPS						8

class Robot: public IterativeRobot
{
private:
	Joystick *joystick;
//	Victor *motorV1;
//	Victor *motorV2;
	CANJaguar *motorJ1;
	CANJaguar *motorJ2;
	CANTalon *motorT1;

	void RobotInit()
	{

	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{

// Initialize the joystick
		joystick = new Joystick(0);

// Initialize the motor2
//		motorV1 = new Victor(8);
//		motorV2 = new Victor(9);
		motorJ1 = new CANJaguar(12);
		motorJ1->SetPercentMode();
		motorJ1->EnableControl();
		motorJ2 = new CANJaguar(13);
		motorJ2->SetPercentMode();
		motorJ2->EnableControl();
		motorT1 = new CANTalon(10);
	}

	void TeleopPeriodic()
	{
		char myString[64];
		float   y;

		SmartDashboard::PutString("DB/String 0", "Starting test");


// get the joystick position and filter the deadband
		y = -joystick->GetY();

		if (y > -0.1 && y < 0.1)
			y = 0;

		sprintf(myString, "joystick setting: %f\n", y);
		SmartDashboard::PutString("DB/String 2", myString);
//		motorV1->Set(y, 0); // set the speed of the motor
//		motorV2->Set(y, 0); // set the speed of the motor
		motorJ1->Set(y, 0); // set the speed of the motor
		motorJ2->Set(y, 0); // set the speed of the motor
		motorT1->Set(y, 0); // set the speed of the motor
	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot);
