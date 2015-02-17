#include "WPILib.h"
#include "time.h"
#include <math.h>

#define MAX_RPS	8
#define MOTOR_TEST_TIME 10.0
#define OUTPUT_DELAY_PERIOD 0

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;
	Encoder    *dynoEncoder;
	CANTalon   *dynoMC;
	int        motorState = 0;
	int        outputDelayCount = 0;
    long       startMs; // Milliseconds
    time_t     startS;  // Seconds
	FILE       *pFile;


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

		dynoEncoder = new Encoder(9, 8, false, Encoder::EncodingType::k4X);
		dynoEncoder->SetMinRate(0.25);
		dynoEncoder->SetDistancePerPulse((double)(1.0/1024));
		dynoEncoder->SetSamplesToAverage(7);

// Initialize the TalonSRX for the dyno
		dynoMC = new CANTalon(10);
	}

	void TeleopPeriodic()
	{
		char            myString[64];  // general string for output
	    long            deltaMs; // delta time in Milliseconds
	    time_t          deltaS;  // delta time in Seconds
	    struct timespec spec;    // structure used by clock_gettime()
		double          rate, elapsedTime;  // encoder rotation rate and elapsed time
		float	        dynoVolts, dynoAmps; // measured motor volts and amps

		switch (motorState)
		{

			case 0: // initial zero state
				sprintf(myString, "Starting test");
				SmartDashboard::PutString("DB/String 0", myString);

				pFile = fopen ("/home/lvuser/dyno-forward.txt","w");
				if (pFile!=NULL)
				{
					clock_gettime(CLOCK_REALTIME, &spec);  // get the current time
					startS  = spec.tv_sec;
					startMs = round(spec.tv_nsec / 1.0e6); // Convert nanoseconds to milliseconds
					dynoMC->Set(1.0);  // set motor to forward 100%
					motorState = 1;    // increment motor state
				}
				else
				{
					sprintf(myString, "fopen failed");
					SmartDashboard::PutString("DB/String 1", myString);
				}
				break;

			case 1: // accelerating forward and sampling
// only execute if adequate sample delay is expired
				if (outputDelayCount == OUTPUT_DELAY_PERIOD)  // ok, output sample
				{
	// get elapsed time
					clock_gettime(CLOCK_REALTIME, &spec);  // get the current time
					deltaS  = spec.tv_sec-startS;
					deltaMs = round(spec.tv_nsec / 1.0e6)-startMs; // Convert nanoseconds to milliseconds
	// get motor controller values
					rate = dynoEncoder->GetRate() * 60.0;
					dynoVolts = dynoMC->GetOutputVoltage();
					dynoAmps = dynoMC->GetOutputCurrent();
	// convert to time in seconds
					elapsedTime = deltaS + (deltaMs/1000.0);
	// output to file
					sprintf(myString, "%f, %f, %f, %f\n", elapsedTime, rate, dynoVolts, dynoAmps);
					fputs (myString,pFile);
					outputDelayCount = 0;  // reset the counter
				}
				else  // increment the counter and don't output a sample
					outputDelayCount++;
// test to see if we're through sampling
				if (elapsedTime > MOTOR_TEST_TIME)
				{
					dynoMC->Set(0.0);  // set motor to forward 100%
					fclose(pFile);
					motorState = 2;    // increment motor state
				}
				break;

			case 2: // stopping forward
// test to see if we've stopped moving
				sprintf(myString, "Stopping");
				SmartDashboard::PutString("DB/String 0", myString);
				if (dynoEncoder->GetStopped())
				{
					pFile = fopen ("/home/lvuser/dyno-reverse.txt","w");
					if (pFile!=NULL)
					{
						clock_gettime(CLOCK_REALTIME, &spec);  // get the current time
						startS  = spec.tv_sec;
						startMs = round(spec.tv_nsec / 1.0e6); // Convert nanoseconds to milliseconds
						dynoMC->Set(-1.0);  // set motor to backward 100%
						motorState = 3;
					}
					else
					{
						sprintf(myString, "fopen failed");
						SmartDashboard::PutString("DB/String 1", myString);
					}
				}
				break;

			case 3: // accelerating backward and sampling
// only execute if adequate sample delay is expired
				if (outputDelayCount == OUTPUT_DELAY_PERIOD)  // ok, output sample
				{
	// get elapsed time
					clock_gettime(CLOCK_REALTIME, &spec);  // get the current time
					deltaS  = spec.tv_sec-startS;
					deltaMs = round(spec.tv_nsec / 1.0e6)-startMs; // Convert nanoseconds to milliseconds
	// get motor controller values
					rate = dynoEncoder->GetRate() * 60.0;
					dynoVolts = dynoMC->GetOutputVoltage();
					dynoAmps = dynoMC->GetOutputCurrent();
	// convert to time in seconds
					elapsedTime = deltaS + (deltaMs/1000.0);
	// output to file
					sprintf(myString, "%f, %f, %f, %f\n", elapsedTime, rate, dynoVolts, dynoAmps);
					fputs (myString,pFile);
				}
				else  // increment the counter and don't output a sample
					outputDelayCount++;
// test to see if we're through sampling
				if (elapsedTime > MOTOR_TEST_TIME)
				{
					dynoMC->Set(0.0);  // set motor to forward 100%
					motorState = 4;    // increment motor state
					fclose (pFile);    // close the file
				}
				break;

			case 4: //  stopping

				sprintf(myString, "Test complete");
				SmartDashboard::PutString("DB/String 0", myString);
				motorState=5;
				break;

			case 5:
				sprintf(myString, "Test complete/n");
				SmartDashboard::PutString("DB/String 0", myString);
		}
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
