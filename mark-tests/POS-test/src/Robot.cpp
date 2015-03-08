#include "WPILib.h"
#include "Robot.h"
#include "Constants.h"

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;
	Victor *leftDrive;
	Encoder *leftEncoder;
    PIDController *controlLeft;
    bool haveSet = false;
    float distPerRev;
    double distanceMultiplier;
    int startingTick;

	void RobotInit()
	{

		leftDrive = new Victor(CHAN_LEFT_DRIVE_TALONSR);
		leftEncoder = new Encoder(CHAN_ENCODER_LEFT_A, CHAN_ENCODER_LEFT_B, false, Encoder::EncodingType::k4X);
        leftEncoder->SetPIDSourceParameter(leftEncoder->kDistance);
        double distPerPulse  = 1.0 / ENCODER_RESOLUTION;
        leftEncoder->SetDistancePerPulse(distPerPulse);
        leftEncoder->SetSamplesToAverage(5);
		leftEncoder->SetMinRate(0.25);
		leftEncoder->Reset();
        controlLeft = new PIDController(0.5, 0.05, 0, 0, leftEncoder, leftDrive);
 //       controlLeft->SetAbsoluteTolerance (0.5);
        controlLeft->SetContinuous(true);
        controlLeft->SetOutputRange(-1.0, 1.0);
        controlLeft->Enable();
        distPerRev = WHEEL_DIAMETER * PI;
        distanceMultiplier = distPerRev / 1024.0;
        startingTick = leftEncoder->Get();

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
		char myString[64];

		if (!haveSet) // have not set the setpoint yet
		{
			controlLeft->SetSetpoint(5.0);
			haveSet = true;
		}
		sprintf(myString, "enc cnt: %d\n", leftEncoder->Get());
		SmartDashboard::PutString("DB/String 0", myString);
		sprintf(myString, "Dist: %f\n", fabs((double)leftEncoder->Get() - (double)startingTick) * distanceMultiplier);
		SmartDashboard::PutString("DB/String 1", myString);
	}

	void TestPeriodic()
	{




	}
};

START_ROBOT_CLASS(Robot);
