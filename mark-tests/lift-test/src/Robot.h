/*
 * Robot.h
 *
 *  Created on: Jan 27, 2015
 *      Author: afishershin
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#define PRACTICE 0
#define COMPETITION 1
#define BUILD_VERSION PRACTICE

#define CHAN_LIFT_ENCODER_LEFT_A 8 //dio
#define CHAN_LIFT_ENCODER_LEFT_B 9 //dio
#define LIFT_ENCODER_RESOLUTION 1024
#define MIN_ENCODER_RATE 0.25 //Minimum encoder rate before motion is considered stopped

#define CHAN_LIFT_LOW_LS 0 //dio
#define CHAN_LIFT_HIGH_LS 1 //dio

#define CHAN_LIFT_MOTOR 13 //can
//
//convention in the code is the a negative motor speed is down and a positive motor speed is up
#define FWD 1 //multiply by the motor set value to set the direction in SetLiftMotor
#define REV -1
#define LIFT_MOTOR_DIR FWD

#define CHAN_JS 0 //usb
#define BUT_JS_ENT_POS_HOLD 1 //enter position holding state
#define BUT_JS_EXIT_POS_HOLD 2 //exit position holding state

#define MOTOR_SPEED_GO 0.25
#define MOTOR_SPEED_STOP 0.0

//#define PID_POS_TOL 25.0; //must be within this times the encoder distance per pulse to be at the target
#define PID_P 0.5
#define PID_I 0.05
#define PID_D 0.0
#define PID_F 0.0
#define PID_OUT_MIN -1.0 //minimum controller output
#define PID_OUT_MAX 1.0 //maximum controller output
#define SP_RANGE_FRACTION 0.5 //fraction of the encoder range at which to establish the hold position state
#define PID_POS_TOL 25.0 //multiply by the encoder distance per pulse to get the distance around the setpoint that is considered to be at the setpoint (setpoint tolerance)

#define UNINIT_VAL -1 //uninitialized valued

#define STAT_STR_LEN 64 //printing to dashboard

#endif /* SRC_ROBOT_H_ */
