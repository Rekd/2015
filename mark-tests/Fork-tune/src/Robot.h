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

#define CHAN_FORK_MIN_LS 0 //dio, inner limit switch
#define CHAN_FORK_MAX_LS 1 //dio, outer limit switch

#define CHAN_GTC 3 //analog, for the gear tooth counter
//for the analog trigger, below the min is false and above the max is true; in between is keep last state
//the values for the analog trigger are 0 to 4096 representing 0V to 5V
#define ANALOG_TRIG_MIN 450
#define ANALOG_TRIG_MAX 2400

#define CHAN_FORK_MOTOR 13 //can
//
//convention in the code is the a negative motor speed is inwards and a positive motor speed is outwards
#define FWD 1 //multiply by the motor set value to set the direction in SetForkMotor
#define REV -1
#define LIFT_MOTOR_DIR FWD

#define CHAN_JS 0 //usb
#define BUT_JS_IN 1
#define BUT_JS_OUT 2
#define BUT_JS_RES_GTC 3 //gear tooth counter

#define MOTOR_SPEED_GO 0.25
#define MOTOR_SPEED_STOP 0.0

#define STAT_STR_LEN 64 //printing to dashboard

#endif /* SRC_ROBOT_H_ */
