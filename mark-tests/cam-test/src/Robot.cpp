#include "WPILib.h"
#include "CamSystem.h"

class Robot: public IterativeRobot
{
private:



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
		CamSystem  cameraSystem;
		char myString[64];


		cameraSystem.Scan();
//		sprintf(myString, "Distance: %f units?", cameraSystem.GetDistance());
//		SmartDashboard::PutString("DB/String 0", myString);
	}

	void TeleopPeriodic()
	{

	}

	void TestPeriodic()
	{
//		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
