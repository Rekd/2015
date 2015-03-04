/*
 * Constants.h
 *
 *  Created on: Feb 16, 2015
 *      Author: mll
 */

#ifndef SRC_CONSTANTS_H_
#define SRC_CONSTANTS_H_

#define PI							3.141592653L

#define CHAN_ENCODER_LEFT_A 		9
#define CHAN_ENCODER_LEFT_B			8
#define CHAN_ENCODER_RIGHT_A		7
#define CHAN_ENCODER_RIGHT_B		6
#define CHAN_FORK_LIMIT_MIN         0
#define CHAN_FORK_LIMIT_MAX         1
#define CHAN_LIFT_LIMIT_MIN         2
#define CHAN_LIFT_LIMIT_MAX         3
#define CHAN_ENCODER_LIFT           4

#define ACHAN_GEAR_COUNT            3

#define FORK_MOTOR_ID               13
#define LIFT_MOTOR_ID               12

// #define ENCODER_RESOLUTION			360.0 optical encoders
#define ENCODER_RESOLUTION			1024.0
#define CHAN_LEFT_DRIVE_TALONSR		8
#define CHAN_RIGHT_DRIVE_TALONSR	9

#define WHEEL_DIAMETER				4.0L
#define WHEEL_CIRCUMFERENCE         (PI*WHEEL_DIAMETER)
#define DRIVE_ENCODER_CPR           360

#define PROPORTIONAL_TERM           0.005f
#define INTEGRAL_TERM               0.1f
#define DIFFERENTIAL_TERM           0.001f
// 1/8 for feed forward
#define FEED_FORWARD_TERM           0.125
#define FORK_MOTOR_IN_SPEED         0.4
#define FORK_MOTOR_OUT_SPEED        -0.4
#define LIFT_MOTOR_IN_SPEED         0.4
#define LIFT_MOTOR_OUT_SPEED        -0.4
#define OPEN_NARROW_COUNT           100
#define OPEN_WIDE_COUNT             365
#define WIDE_NARROW_DIFF            265
#define NARROW_WIDE_DIFF            -265
#define CLOSING_COUNT               400
#define LIFT_MOTOR_UP_SPEED         0.4  //NOT ACCURATE, RANDOM
#define LIFT_MOTOR_DOWN_SPEED       -0.4 //NOT ACCURATE, RANDOM
#define POS_ONE						200  //NOT ACCURATE, RANDOM
#define POS_TWO						400  //NOT ACCURATE, RANDOM
#define POS_THREE					800  //NOT ACCURATE, RANDOM
#define POS_STEP					300  //NOT ACCURATE, RANDOM

#define MAX_RPS						8

#define FORK_CURRENT_LIMIT  7.0f



#endif /* SRC_CONSTANTS_H_ */
