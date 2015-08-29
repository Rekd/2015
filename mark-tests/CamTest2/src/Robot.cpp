#include "WPILib.h"
#include "CamSystem.h"

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;
	CamSystem  cameraSystem;

	void RobotInit()
	{
//		lw = LiveWindow::GetInstance();
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{

		char myString[64];
		sprintf(myString, "In TeleopInit");
		SmartDashboard::PutString("DB/String 1", myString);

		cameraSystem.Scan();
//		sprintf(myString, "Distance: %f units?", cameraSystem.GetDistance());
//		SmartDashboard::PutString("DB/String 0", myString);
	}

	void TeleopPeriodic()
	{
//		char myString[64];
//		sprintf(myString, "In TeleopPeriodic");
//		SmartDashboard::PutString("DB/String 2", myString);
	}

	void TestPeriodic()
	{
//		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
