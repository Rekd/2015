#include "WPILib.h"

#define MAX_RPS						8
#define GEAR_TOOTH_CHAN 9

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;
	Encoder *sampleEncoder;
	Encoder *dynoEncoder;
	Joystick *joystick;
	Victor *motor;
	Counter *gearToothCounter;
	AnalogTrigger *toothTrigger;
//	AnalogInput *toothInput;

	int     encoderCount;
	double  raw, distance, period, rate;
	float	dynoVolts, dynoAmps;
	bool direction, stopped;

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{
// Initialize the encoder
		sampleEncoder = new Encoder(0, 1, false, Encoder::EncodingType::k4X);
		sampleEncoder->SetMaxPeriod(.1);
		sampleEncoder->SetMinRate(10);
		sampleEncoder->SetDistancePerPulse(5);
		sampleEncoder->SetReverseDirection(true);
		sampleEncoder->SetSamplesToAverage(7);

// Initialize the joystick
		joystick = new Joystick(0);

// Initialize the motor
		motor = new Victor(9);

// Initialize the gear tooth counter
		toothTrigger = new AnalogTrigger(3);
		toothTrigger->SetLimitsRaw(250, 3600);
		gearToothCounter = new Counter(toothTrigger);
//		gearToothCounter->SetUpDownCounterMode();
	}

	void TeleopPeriodic()
	{
		char myString[64];
		float   y;


// get the joystick position and filter the deadband
		y = -joystick->GetY();

		if (y > -0.1 && y < 0.1)
			y = 0;

		sprintf(myString, "joystick setting: %f\n", y);
		SmartDashboard::PutString("DB/String 1", myString);
		motor->Set(y, 0); // set the speed of the motor
		sprintf(myString, "gear count: %d\n", gearToothCounter->Get());
		SmartDashboard::PutString("DB/String 2", myString);
		sprintf(myString, "gear stopped: %d\n", gearToothCounter->GetStopped());
		SmartDashboard::PutString("DB/String 3", myString);

//		sprintf(myString, "In val: %d\n", toothInput->GetValue());
//		SmartDashboard::PutString("DB/String 2", myString);
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
