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

#define MAX_RPS						8

#define	MOTOR_STOP							0.0
#define FORK_MOTOR_OUT_SPEED        		0.4 //out is positive
#define FORK_MOTOR_IN_SPEED         		-0.4 //in is negative
#define LIFT_MOTOR_UP_SPEED         		0.4 //up is positive
#define LIFT_MOTOR_DOWN_SPEED        		-0.4 //down is negative
#define OPEN_NARROW_COUNT           		100 //distance from the inner limit
#define OPEN_WIDE_COUNT           			365 //distance from the inner limit
#define CLOSE_COUNT               			50 //distance from the inner limit, this is not actually the close position which will vary, this will just get the forks to close
#define	PICKUP_POS							50
#define CARRY_ONE_POS						200
#define CARRY_TWO_POS						400
#define CARRY_THREE_POS						800
#define CARRY_STEP_POS						300
#define LIFT_OFFSET							30 //this is relative to the release position, not absolute
#define FORK_OFFSET							30 //this is relative to the release position, not absolute
#define FORK_CURRENT_LIMIT  				25.0
#define MOTOR_REV							-1
#define MOTOR_NOT_REV						1
#define FORK_MOTOR_REV_STATE				MOTOR_NOT_REV
#define LIFT_MOTOR_REV_STATE				MOTOR_NOT_REV
#define FORK_POS_TOL						10
#define LIFT_POS_TOL						10

//operator box inputs
#define OPEN_WIDE_BUTTON					2
#define OPEN_NARROW_BUTTON					3
#define INTAKE_BUTTON						4
#define CLOSE_BUTTON						5
#define RELEASE_BUTTON						6
#define CARRY_ONE_BUTTON					7
#define CARRY_TWO_BUTTON					8
#define CARRY_THREE_BUTTON					9
#define CARRY_STEP_BUTTON					10

//operator box outputs
#define OPEN_WIDE_LED						2
#define OPEN_NARROW_LED						3
#define INTAKE_LED							4
#define CLOSE_LED							5
#define RELEASE_LED							6
#define CARRY_ONE_LED						7
#define CARRY_TWO_LED						8
#define CARRY_THREE_LED						9
#define CARRY_STEP_LED						10

#endif /* SRC_CONSTANTS_H_ */
