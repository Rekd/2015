#include "WPILib.h"

//constants
#define MOTOR_REV							-1
#define MOTOR_NOT_REV						1
#define MOTOR_STOP							0.0
#define PRACTICE 							0
#define COMPETITION 						1
#define ALL_LEDS_ON							1023 //in binary turns on 10 LEDs
#define ALL_LEDS_OFF						0
#define STATUS_STR_LEN						64

//configure
#define BUILD_VERSION 						PRACTICE
#define FORK_MOTOR_REV_STATE				MOTOR_REV
#define CH_TOOTH_TRIGGER					3 //analog
#define CH_FORK_MOTOR						13 //can
#define CH_FORK_LS_MIN						0 //dio
#define CH_FORK_LS_MAX						1 //dio
#define CH_JS								3 //usb
#define MOTOR_SPEED							0.3 //should be non-signed
#define MAX_CUR_TH							25.0 //maximum current threshold, amps
#define GEAR_TRIGGER_MIN					450  // was 450 //0 to 4096 representing 0V to 5V
#define GEAR_TRIGGER_MAX					2000

//speed convention for forks is:
//+ is outwards
//- is inwards

class Robot: public IterativeRobot
{
private:
	typedef enum{closing,
		opening} ForkState;

	typedef enum {
		inwards = -1,
		outwards = 1
	} ForkDirection;

#if BUILD_VERSION == COMPETITION
	CANTalon *forkMotor;
#else
	CANJaguar *forkMotor;
#endif

	AnalogTrigger *toothTrigger;
	Counter *gearToothCounter;

//	AnalogInput *aIn;

	DigitalInput *forkLimitSwitchMin;
	DigitalInput *forkLimitSwitchMax;
	Joystick *dsBox;  //this is a separate test running simultaneous with the fork test

	bool running; //false = do not run the test program, true = run the test program

	ForkState forkState;

	// fork position tracking
	ForkDirection forkDirection;
	float curForkSetSpeed; //the current fork speed
	int absGearToothCount; //absolute gear tooth count (not relative)
	int curGearToothCount; //current gear tooth count
	int lastGearToothCount; //last gear tooth count

//	int16_t minAVal = 32767;
//	int16_t maxAVal = -32767;


	bool GetForkLimitSwitchMin()
	{
		//invert so that TRUE when limit switch is closed and FALSE when limit switch is open
		return !(forkLimitSwitchMin->Get());
	}

	bool GetForkLimitSwitchMax()
	{
		//invert so that TRUE when limit switch is closed and FALSE when limit switch is open
		return !(forkLimitSwitchMax->Get());
	}

	void SetForkMotor(float val)
	{
		curForkSetSpeed = FORK_MOTOR_REV_STATE*val;
		forkMotor->Set(curForkSetSpeed);
	}

	void UpdateGearCount ()
	//this function turns the relative gear tooth counter into an absolute counter
	{
		if (curForkSetSpeed < 0)
			forkDirection = inwards;
		else //curForkSetSpeed > 0; this function should not be called when curForkSpeed = 0, should on be called when the forks are moving;
			forkDirection = outwards;

		curGearToothCount = gearToothCounter->Get();
		absGearToothCount += forkDirection*(curGearToothCount-lastGearToothCount);
		lastGearToothCount = curGearToothCount;
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit() {
		forkState = closing;
		SetForkMotor(-MOTOR_SPEED); //start in closing state
		absGearToothCount = 0;
		curGearToothCount = 0;
		lastGearToothCount = 0;
		running = true; //
		//forkDirection initialized when first used
		//curForkSetSpeed initialized when first used
		dsBox->SetOutputs(ALL_LEDS_OFF);
	}

	void TeleopPeriodic()
	{
		char myString [STATUS_STR_LEN];

		if (running)
		{
			switch (forkState)
			{
				case closing:
					UpdateGearCount (); //update whenever the forks are moving
					if (GetForkLimitSwitchMin())
					{
						sprintf(myString, "min gear count: %d\n", absGearToothCount);
						SmartDashboard::PutString("DB/String 8", myString);
						SetForkMotor(MOTOR_STOP);
						SetForkMotor(MOTOR_SPEED);
						dsBox->SetOutputs(ALL_LEDS_ON);
						forkState = opening;
					}
					break;

				case opening:
					UpdateGearCount (); //update whenever the forks are moving
					if (GetForkLimitSwitchMax())
					{
						sprintf(myString, "max gear count: %d\n", absGearToothCount);
						SmartDashboard::PutString("DB/String 9", myString);
						SetForkMotor(MOTOR_STOP);
						SetForkMotor(-MOTOR_SPEED);
						dsBox->SetOutputs(ALL_LEDS_OFF);
						forkState = closing;
					}
					break;
			}
		}

		//current monitor check for safety
		if (forkMotor->GetOutputCurrent() > MAX_CUR_TH)
		{
			SetForkMotor(MOTOR_STOP);
			running = false;
		}

		sprintf(myString, "curr: %f\n", forkMotor->GetOutputCurrent());
		SmartDashboard::PutString("DB/String 0", myString);
		sprintf(myString, "State: %d\n", forkState);
		SmartDashboard::PutString("DB/String 1", myString);
		sprintf(myString, "running: %d\n", running);
		SmartDashboard::PutString("DB/String 2", myString);
		sprintf(myString, "forkDir: %d\n", forkDirection);
		SmartDashboard::PutString("DB/String 3", myString);
		sprintf(myString, "abs gear count: %d\n", absGearToothCount);
		SmartDashboard::PutString("DB/String 4", myString);

#if 0
		if (minAVal > aIn->GetValue())
			minAVal = aIn->GetValue();
		if (maxAVal < aIn->GetValue())
			maxAVal = aIn->GetValue();
		sprintf(myString, "min A val: %d\n", minAVal);
		SmartDashboard::PutString("DB/String 5", myString);
		sprintf(myString, "max A val: %d\n", maxAVal);
		SmartDashboard::PutString("DB/String 6", myString);
#endif


	}

	void TestPeriodic()
	{

	}

public:
	Robot()
	{
#if BUILD_VERSION == COMPETITION
		forkMotor = new CANTalon(CH_FORK_MOTOR);
#else
		forkMotor = new CANJaguar(CH_FORK_MOTOR);
#endif
		toothTrigger = new AnalogTrigger(CH_TOOTH_TRIGGER);
		toothTrigger->SetLimitsRaw(GEAR_TRIGGER_MIN, GEAR_TRIGGER_MAX);
		gearToothCounter = new Counter(toothTrigger);
//		aIn = new AnalogInput(CH_TOOTH_TRIGGER);
		forkLimitSwitchMin = new DigitalInput(CH_FORK_LS_MIN);
		forkLimitSwitchMax = new DigitalInput(CH_FORK_LS_MAX);
		dsBox = new Joystick(CH_JS);
	}

	~Robot()
	{
		delete forkMotor;
		delete toothTrigger;
		delete gearToothCounter;
		delete forkLimitSwitchMin;
		delete forkLimitSwitchMax;
		delete dsBox;
	}
};

START_ROBOT_CLASS(Robot);
