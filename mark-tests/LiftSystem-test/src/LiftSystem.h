/*
 * LiftSystem.h
 *
 *  Created on: Feb 15, 2015
 *      Author: mll
 */

#ifndef LIFTSYSTEM_H_
#define LIFTSYSTEM_H_

#include "WPILib.h"
#include "Constants.h"

class LiftSystem {
public:
	LiftSystem(CANSpeedController *pForkMotor, CANSpeedController *pLiftMotor, CANSpeedController *pLeftIntake, CANSpeedController *pRightIntake,
			Counter *pGearToothCounter, Encoder *pLiftEnc, PIDController *pLiftControl,
			DigitalInput *pForkLimitInner, DigitalInput *pForkLimitOuter, DigitalInput *pLiftLimitLow, DigitalInput *pLiftLimitHigh,
			Joystick *pOperatorBox);
	virtual ~LiftSystem();
	void Update();

private:
	typedef enum {opened_narrow_GP,
		opened_wide_GP,
		closed_C_Pos_One,
		closed_C_Pos_Two,
		closed_C_Pos_Three,
		closed_C_Pos_Step,
		released,
		halt_pickup_gantry,
		} RobotState;

	typedef enum {narrow_idle,
		narrow_closing_fork,
		opening_wide,
		narrow_changing_lift,
		narrow_error_recovery,
		} openedNarrowSubState;

	typedef enum {wide_idle,
		wide_closing_fork,
		opening_narrow,
		wide_changing_lift,
		wide_error_recovery,
		} openedWideSubState;

	typedef enum {closed_idle,
		closed_changing_level,
		releasing_lowering,
		releasing_open_forks} ClosedSubState;

	typedef enum {released_idle,
			released_lowering,
			moving_to_open} ReleasedSubState;

	typedef enum {
		inwards = -1,
		outwards = 1
	} ForkDirection;

// local motors, encoders, and switches
	CANSpeedController	*forkMotor, *liftMotor, *leftIntake, *rightIntake;
	Counter     *gearToothCounter;
	Encoder     *liftEnc;
	PIDController *liftControl;
	DigitalInput *forkLimitInner, *forkLimitOuter, *liftLimitLow, *liftLimitHigh;
	Joystick    *operatorBox;

// state variables
	RobotState  robotState;
	openedNarrowSubState  openedNarrowSS;
	openedWideSubState openedWideSS;
	ClosedSubState closedSS;
	ReleasedSubState releasedSS;


// other private variables
	int  targetForkGearCount; //target is an integer gear count
	int  targetLiftEncoderCount; //target is an integer encoder count
// fork position tracking
	ForkDirection forkDirection;
	float curForkSetSpeed; //the current fork speed
	int absGearToothCount; //absolute gear tooth count (not relative)
	int curGearToothCount; //current gear tooth count
	int lastGearToothCount; //last gear tooth count

	//prototypes
	bool GetForkLimitSwitchInner();
	bool GetForkLimitSwitchOuter();
	bool GetLiftLimitSwitchLow();
	bool GetLiftLimitSwitchHigh();
	void CheckAndHandleHaltPickupGantry();
	void SetForkMotor(float val);
	bool CheckForkMotorCurrentSpike();
	void UpdateGearToothCount();
	void SetForkTarget(int target);
	void SetLiftTarget(int target);
	bool CheckForkHasReachedTarget();
	bool CheckLiftHasReachedTarget();
	bool IsButtonPressed(int button);
	void TurnLedOn(int led);
	void TurnLedOff(int led);
	void TurnForkLedsOff();
	void TurnLiftLedsOff();

};

#endif /* LIFTSYSTEM_H_ */
