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
	LiftSystem(CANTalon *pforkMotor, CANTalon *pliftMotor, Counter *gearToothCounter, Encoder *liftEnc,
			DigitalInput *forkLimitMin, DigitalInput *forkLimitMax, DigitalInput *liftLimitMin,
			DigitalInput *liftLimitMax, Joystick *pjoystick);

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
		lift_error,
		} RobotState;

	typedef enum {narrow_idle,
		narrow_closing_fork,
		narrow_changing_lift,
		narrow_error_recovery,
		opening_wide} OpenedNarrowSubState;

	typedef enum {wide_idle,
		wide_closing_fork,
		wide_changing_lift,
		wide_error_recovery,
		opening_narrow} OpenedWideSubState;

	typedef enum {closed_idle,
		closed_changing_level,
		releasing_lowering,
		releasing_open_forks} ClosedSubState;

	typedef enum {released_idle,
			moving_to_open} ReleasedSubState;

// local motors, encoders, and switches
	CANTalon    *forkMotor, *liftMotor;
	Encoder     *liftEncoder;
	Counter     *gearToothCounter;
	DigitalInput *forkLimitSwitchMin, *forkLimitSwitchMax, *liftLimitSwitchMin, *liftLimitSwitchMax;
	Joystick    *joystick;

// state variables
	RobotState  robotState;
	OpenedNarrowSubState  openedNarrowSubState;
	OpenedWideSubState openedWideSubState;


// other local values
	int  targetForkGearCount;
	int  targetLiftEncoderCount;


	bool GetForkLimitSwitchMin();
	bool GetForkLimitSwitchMax();
	bool GetLiftLimitSwitchMin();
	bool GetLiftLimitSwitchMax();
	void SetForkMotor(float val);
	void SetLiftMotor(float val);
	bool CheckForkMotorCurrentSpike();
	void SetForkTarget(int target);
	void SetLiftTarget(float target);
	bool CheckForkHasReachedTarget();
	void ResetGearCounter();
	bool IsOpenWideButtonPressed();
	bool IsCloseButtonPressed();
};

#endif /* LIFTSYSTEM_H_ */
