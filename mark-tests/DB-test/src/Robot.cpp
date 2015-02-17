#include "WPILib.h"
#include <AutonomousProgram1.h>
#include <AutonomousProgram2.h>

/**
 * Uses the CameraServer class to automatically capture video from a USB webcam
 * and send it to the FRC dashboard without doing any vision processing. This
 * is the easiest way to get camera images to the dashboard. Just add this to the
 * RobotInit() method in your program.
 */
class Robot: public IterativeRobot
{
// some setup to allow for choosing autonomous program from the smart dashboard
	Command *autonomousCommand;
	SendableChooser  *autoChooser;


private:
	Encoder *dynoEncoder;
	Joystick *joystick;
	CANTalon *dynoMC; //jeff testing


public:
	void RobotInit()
	{
		char myString[64];

// initialize the USB camera
		CameraServer::GetInstance()->SetQuality(50);
		//the camera name (ex "cam0") can be found through the roborio web interface
		CameraServer::GetInstance()->StartAutomaticCapture("cam1");

// output a sample string to the dashboard (string 0)
		int  i;
		sprintf(myString, "i = %d/n", i);
		SmartDashboard::PutString("DB/String 0", myString);

// Sample read a string from the same dashboard (string 5)
		std::string dashData;
		dashData = SmartDashboard::GetString("DB/String 5", "myDefaultData");

// Sample write to a button (button 0)
		SmartDashboard::PutBoolean("DB/Button 0", true);

// Sample read a button value (button 1) and output it to a string
		bool buttonValue;
		buttonValue = SmartDashboard::GetBoolean("DB/Button 0", false);
		sprintf(myString, "button 1 = %d/n", buttonValue);
		SmartDashboard::PutString("DB/String 1", myString);

// Sample put a value into a slider
		SmartDashboard::PutNumber("DB/Slider 0", (double).58);

// Sample read a value from a slider
		double dashData2;
		dashData2 = SmartDashboard::GetNumber("DB/Slider 1", 0.0);
		sprintf(myString, "slider 1 = %f/n", dashData2);
		SmartDashboard::PutString("DB/String 2", myString);

// some additional setups for teleop mode
		dynoEncoder = new Encoder(9, 8, false, Encoder::EncodingType::k4X);
		dynoEncoder->SetMaxPeriod(.1);
		dynoEncoder->SetMinRate(10);
		dynoEncoder->SetDistancePerPulse(5);
		dynoEncoder->SetReverseDirection(true);
		dynoEncoder->SetSamplesToAverage(7);
// Initialize the joystick and motor controller
		joystick = new Joystick(0);
		dynoMC = new CANTalon(10);

// Set up the SendableChooser for choosing which autonomous mode
		autoChooser = new SendableChooser();
		autoChooser->AddDefault("Autonomous Program #1", new AutonomousProgram1());
		autoChooser->AddObject("Autonomous Program #2", new AutonomousProgram2());
		SmartDashboard::PutData("Autonomous mode chooser", autoChooser);

	}
	void AutonomousInit()
	{
// get set up for the autonomous mode indicated on the smart dashboard
		autonomousCommand = (Command*) autoChooser->GetSelected();
		autonomousCommand->Start();
	}

	void AutonomousPeriodic()
	{
// Run the chosen autonomous program
		Scheduler::GetInstance()->Run();
	}


	void TeleopInit()
	{
	}

	void TeleopPeriodic()
	{
		float  y;
		int32_t  encoderCount, encoderRaw;
		double  motorVolts, motorAmps;


// Here are some smart dashboard examples - this is designed to work with the testbench
		y = joystick->GetY();
		SmartDashboard::PutNumber("Joystick Value", (double)y);

		encoderCount = dynoEncoder->Get();
		SmartDashboard::PutNumber("Current Encoder Count", (double)encoderCount);

		encoderRaw = dynoEncoder->GetRaw();
		SmartDashboard::PutNumber("Raw Encoder Count", (double)encoderRaw);

		motorVolts = dynoMC->GetOutputVoltage();
		motorAmps = dynoMC->GetOutputCurrent();
		SmartDashboard::PutNumber("Motor Voltage", motorVolts);
		SmartDashboard::PutNumber("Motor Amperage", motorAmps);

};

START_ROBOT_CLASS(Robot);

