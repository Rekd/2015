#include "WPILib.h"
#include "Robot.h"
#include "Constants.h"
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
	float driveX, driveY; //values from driver controls: x from steeringWheel, y from driveJoystick

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
	DigitalInput *forkLimitInner;
	DigitalInput *forkLimitOuter;
	DigitalInput *liftLimitLow; //at the bottom of the lift
	DigitalInput *liftLimitHigh; //at the top of the lift
	Joystick *liftSysJoystick;
	LiftSystem *liftSystem;

	void RobotInit()
	{
		//not used
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
		//Initialize PID
		driveSystem->SetPIDDrive(PID_CONFIG);
		//Set Wheel Diameter
		driveSystem->SetWheelDiameter(WHEEL_DIAMETER);
		// start recording the distance traveled
		driveSystem->StartRecordingDistance();
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
		//not used
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
		forkMotor = new CANTalon(CHAN_FORK_MOTOR);
		liftMotor = new CANTalon(CHAN_LIFT_MOTOR);
		leftIntakeMotor = new CANTalon(CHAN_L_INTAKE_MOTOR);
		rightIntakeMotor = new CANTalon(CHAN_R_INTAKE_MOTOR);
#else
		forkMotor = new CANJaguar(CHAN_FORK_MOTOR);
		liftMotor = new CANJaguar(CHAN_LIFT_MOTOR);
		leftIntakeMotor = new CANJaguar(CHAN_L_INTAKE_MOTOR);
		rightIntakeMotor = new CANJaguar(CHAN_R_INTAKE_MOTOR);
#endif
		forkLimitInner = new DigitalInput(CHAN_FORK_MIN_LS);
		forkLimitOuter = new DigitalInput(CHAN_FORK_MAX_LS);
		liftLimitLow = new DigitalInput(CHAN_LIFT_LOW_LS);
		liftLimitHigh = new DigitalInput(CHAN_LIFT_HIGH_LS);
		liftSysJoystick = new Joystick(CHAN_LIFT_SYS_JS);
		liftSystem = new LiftSystem(forkMotor, liftMotor, leftIntakeMotor, rightIntakeMotor,
				forkLimitInner, forkLimitOuter, liftLimitLow, liftLimitHigh,
				liftSysJoystick);
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
		delete forkLimitInner;
		delete forkLimitOuter;
		delete liftLimitLow;
		delete liftLimitHigh;
		delete liftSysJoystick;
		delete liftSystem;
	}
};

START_ROBOT_CLASS(Robot);
