#include "WPILib.h"
#include "ITG3200.h"

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;

	void RobotInit()
	{
		gyro.begin();
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		char  myString[64];

		gyro.update();
		sprintf(myString, "Gyro X: %f\n", gyro.getX());
		SmartDashboard::PutString("DB/String 0", myString);
		sprintf(myString, "Gyro Y: %f\n", gyro.getY());
		SmartDashboard::PutString("DB/String 1", myString);
		sprintf(myString, "Gyro Z: %f\n", gyro.getZ());
		SmartDashboard::PutString("DB/String 2", myString);
	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot);
