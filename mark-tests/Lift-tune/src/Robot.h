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
#define BUILD_VERSION COMPETITION

#define CHAN_LIFT_ENCODER_A 5 //dio
#define CHAN_LIFT_ENCODER_B 4 //dio
#define LIFT_ENCODER_RESOLUTION 1024
#define LIFT_ENCODER_DIST_PER_PULSE (1.0/LIFT_ENCODER_RESOLUTION)

#define CHAN_LIFT_LOW_LS 3 //dio
#define CHAN_LIFT_HIGH_LS 2 //dio

#define CHAN_LIFT_MOTOR_BACK 				12 //can
#define CHAN_LIFT_MOTOR_FRONT				14 //can
//
//convention in the code is the a negative motor speed is down and a positive motor speed is up
#define FWD 1 //multiply by the motor set value to set the direction in SetLiftMotor
#define REV -1
#define LIFT_MOTOR_DIR FWD //both lift motors have the same direction

#define CHAN_JS 0 //usb
#define BUT_JS_UP 1
#define BUT_JS_DOWN 2
#define BUT_JS_RES_EN 3

#define MOTOR_SPEED_UP 0.4 //unsigned speed
#define MOTOR_SPEED_DOWN 0.2 //unsigned speed
#define MOTOR_SPEED_STOP 0.0

#define STAT_STR_LEN 64 //printing to dashboard

#endif /* SRC_ROBOT_H_ */
