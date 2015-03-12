#include "WPILib.h"
#include "Constants.h"
#include "DriveSystem.cpp"
#include "LiftSystem.h"

class Robot: public IterativeRobot
{
private:
	//drive system
#if BUILD_VERSION == COMPETITION
	Talon * leftDrive;
	Talon * rightDrive;
#else
	Victor * leftDrive;
	Victor * rightDrive;
#endif
	Encoder *leftEncoder;
	Encoder *rightEncoder;
	DriveSystem *driveSystem;

	//driver controls
	Joystick *driveJoystick;
	Joystick *steeringWheel;
	float driveX, driveY; //values from driver controls: x from steeringWheel, y from driveJoystick, initialized on first use

	//lift system
#if BUILD_VERSION == COMPETITION
	CANTalon *forkMotor;
	CANTalon *liftMotor;
	CANTalon *leftIntakeMotor;
	CANTalon *rightIntakeMotor;
#else
	CANJaguar *forkMotor;
	CANJaguar *liftMotor;
	CANJaguar *leftIntakeMotor;
	CANJaguar *rightIntakeMotor;
#endif
	AnalogTrigger *toothTrigger;
	Counter *gearToothCounter;
	Encoder *liftEnc;
	PIDController *liftControl;
	DigitalInput *forkLimitInner;
	DigitalInput *forkLimitOuter;
	DigitalInput *liftLimitLow; //at the bottom of the lift
	DigitalInput *liftLimitHigh; //at the top of the lift
	Joystick *operatorBox;
	LiftSystem *liftSystem;

	void RobotInit()
	{
		//Initialize PID
		driveSystem->SetPIDDrive(PID_CONFIG);
		//Set Wheel Diameter
		driveSystem->SetWheelDiameter(WHEEL_DIAMETER);
		// start recording the distance traveled
		driveSystem->StartRecordingDistance();
	}

	void AutonomousInit()
	{
		//not used
	}

	void AutonomousPeriodic()
	{
		//not used
	}

	void TeleopInit()
	{
		//not used
	}

	void TeleopPeriodic()
	{
		//get driver controls values
		driveX = -steeringWheel->GetX();
		driveY = driveJoystick->GetY();

		//Filter deadband
		if (driveX > DRIVE_DB_LOW && driveX < DRIVE_DB_HIGH)
			driveX = ZERO_FL;
		if (driveY > DRIVE_DB_LOW && driveY < DRIVE_DB_HIGH)
			driveY = ZERO_FL;

		//Give drive instructions
		driveSystem->SetDriveInstruction(driveY * MAX_RPS, driveX * MAX_RPS);
		driveSystem->Update();

		//Give lift instructions
		liftSystem->Update();
	}

	void TestPeriodic()
	{
		liftSystem->MoveToResetState();
	}

public:
	Robot()
	{
		//drive system
#if BUILD_VERSION == COMPETITION
		leftDrive = new Talon(CHAN_LEFT_DRIVE);
		rightDrive = new Talon(CHAN_RIGHT_DRIVE);
#else
		leftDrive = new Victor(CHAN_LEFT_DRIVE);
		rightDrive = new Victor(CHAN_RIGHT_DRIVE);
#endif
		leftEncoder = new Encoder(CHAN_ENCODER_LEFT_A, CHAN_ENCODER_LEFT_B, false, Encoder::EncodingType::k4X);
		rightEncoder = new Encoder(CHAN_ENCODER_RIGHT_A, CHAN_ENCODER_RIGHT_B, false, Encoder::EncodingType::k4X);
		driveSystem = new DriveSystem(leftEncoder, rightEncoder, leftDrive, rightDrive);

		//driver controls
		driveJoystick = new Joystick(CHAN_DRIVE_JS);
		steeringWheel = new Joystick(CHAN_STEERING_WHEEL);

		//lift system
#if BUILD_VERSION == COMPETITION
		forkMotor = new CANTalon(FORK_MOTOR_ID);
		liftMotor = new CANTalon(LIFT_MOTOR_ID);
		leftIntakeMotor = new CANTalon(L_INTAKE_MOTOR_ID);
		rightIntakeMotor = new CANTalon(R_INTAKE_MOTOR_ID);
#else
		forkMotor = new CANJaguar(FORK_MOTOR_ID);
		liftMotor = new CANJaguar(LIFT_MOTOR_ID);
		leftIntakeMotor = new CANJaguar(L_INTAKE_MOTOR_ID);
		rightIntakeMotor = new CANJaguar(R_INTAKE_MOTOR_ID);
#endif
		toothTrigger = new AnalogTrigger(ACHAN_GEAR_COUNT);
		toothTrigger->SetLimitsRaw(GEAR_TRIGGER_MIN, GEAR_TRIGGER_MAX);
		gearToothCounter = new Counter(toothTrigger);
		liftEnc = new Encoder(CHAN_LIFT_ENCODER_A, CHAN_LIFT_ENCODER_B, false, Encoder::EncodingType::k4X);
		liftEnc->SetDistancePerPulse(LIFT_ENCODER_DIST_PER_PULSE);
		liftEnc->SetPIDSourceParameter(liftEnc->kDistance);
		liftControl = new PIDController(LIFT_PROPORTIONAL_TERM, LIFT_INTEGRAL_TERM, LIFT_DIFFERENTIAL_TERM, liftEnc, liftMotor);
		liftControl->SetContinuous(true); //treat input to controller as continuous; true by default
		liftControl->SetOutputRange(LIFT_PID_OUT_MIN, LIFT_PID_OUT_MAX);
		liftControl->Disable(); //do not enable until in holding position mode
		forkLimitInner = new DigitalInput(CHAN_FORK_MIN_LS);
		forkLimitOuter = new DigitalInput(CHAN_FORK_MAX_LS);
		liftLimitLow = new DigitalInput(CHAN_LIFT_LOW_LS);
		liftLimitHigh = new DigitalInput(CHAN_LIFT_HIGH_LS);
		operatorBox = new Joystick(CHAN_OPERATOR_BOX);
		liftSystem = new LiftSystem(forkMotor, liftMotor, leftIntakeMotor, rightIntakeMotor,
				gearToothCounter, liftEnc, liftControl,
				forkLimitInner, forkLimitOuter, liftLimitLow, liftLimitHigh,
				operatorBox);
	}

	~Robot()
	{
		//drive system
		delete leftDrive;
		delete rightDrive;
		delete leftEncoder;
		delete rightEncoder;
		delete driveSystem;

		//driver controls
		delete driveJoystick;
		delete steeringWheel;

		//lift system
		delete forkMotor;
		delete liftMotor;
		delete leftIntakeMotor;
		delete rightIntakeMotor;
		delete toothTrigger;
		delete gearToothCounter;
		delete liftEnc;
		delete liftControl;
		delete forkLimitInner;
		delete forkLimitOuter;
		delete liftLimitLow;
		delete liftLimitHigh;
		delete operatorBox;
		delete liftSystem;
	}
};

START_ROBOT_CLASS(Robot);
