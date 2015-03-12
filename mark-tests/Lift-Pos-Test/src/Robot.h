/*
 * Robot.h
 *
 *  Created on: Jan 27, 2015
 *      Author: afishershin
 */
//Adding a line to get Git to sync

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#define PRACTICE 0
#define COMPETITION 1
#define BUILD_VERSION PRACTICE

#define CHAN_LIFT_ENCODER_A 5 //dio
#define CHAN_LIFT_ENCODER_B 4 //dio
#define LIFT_ENCODER_RESOLUTION 1024
#define LIFT_ENCODER_DIST_PER_PULSE (1.0/LIFT_ENCODER_RESOLUTION)
#define LIFT_PROPORTIONAL_TERM           	2.0
#define LIFT_INTEGRAL_TERM               	0.5
#define LIFT_DIFFERENTIAL_TERM           	0.001
#define LIFT_PID_OUT_MIN -0.1 //minimum controller output
#define LIFT_PID_OUT_MAX 0.7 //maximum controller output

#if 0
original values
#define LIFT_PROPORTIONAL_TERM           	0.5
#define LIFT_INTEGRAL_TERM               	0.05
#define LIFT_DIFFERENTIAL_TERM           	0.000
#endif

#define CHAN_LIFT_LOW_LS 3 //dio
#define CHAN_LIFT_HIGH_LS 2 //dio

#define CHAN_LIFT_MOTOR 12 //can
//
//convention in the code is the a negative motor speed is down and a positive motor speed is up
#define FWD 1 //multiply by the motor set value to set the direction in SetLiftMotor
#define REV -1
#define LIFT_MOTOR_DIR FWD

#define CHAN_JS 0 //usb
#define BUT_JS_RES_EN 4

#define MOTOR_SPEED_UP 0.6 //unsigned speed
#define MOTOR_SPEED_DOWN 0.1 //unsigned speed
#define MOTOR_SPEED_STOP 0.0

#define STAT_STR_LEN 64 //printing to dashboard
#define UNINIT_VAL -1 //uninitialized valued

//gear tooth counts (integers)
//inner limit is the zero point
#define OPEN_NARROW_COUNT           		67 //distance from zero point
#define OPEN_WIDE_COUNT           			310 //distance from zero point
#define CLOSE_COUNT               			0 //distance from zero point, this is not actually the close position which will vary, this will just get the forks to close
#define FORK_OFFSET							55 //this is relative to the release position, not absolute
#define FORK_POS_TOL						3 //this is relative to a set position, not absolute

//lift encoder positions (float number of rotations)
//lower limit is the zero point
#define	PICKUP_POS							0.1 //distance from zero point
#define CARRY_ONE_POS						0.663 //distance from zero point
#define CARRY_TWO_POS						2.048 //distance from zero point
#define CARRY_THREE_POS						2.048 //distance from zero point
#define CARRY_STEP_POS						1.089 //distance from zero point

#if 0
original positions
#define	PICKUP_POS							0.1 //distance from zero point
#define CARRY_ONE_POS						0.663 //distance from zero point
#define CARRY_TWO_POS						2.048 //distance from zero point
#define CARRY_THREE_POS						2.048 //distance from zero point
#define CARRY_STEP_POS						1.089 //distance from zero point
#endif

#define LIFT_OFFSET							0.5 //this is relative to the release position, not absolute
#define LIFT_POS_TOL						0.1 //number of encoder rotations relative to a set position, not absolute

//current limits
#define FORK_CURRENT_LIMIT  				25.0


#endif /* SRC_ROBOT_H_ */
