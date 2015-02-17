#include "WPILib.h"
#include CamSystem.h

class Robot: public IterativeRobot
{
private:
	CameraSystem  *cameraSystem;


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

	}

	void TeleopPeriodic()
	{

	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
