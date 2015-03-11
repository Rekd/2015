/*
 * Constants.h
 *
 *  Created on: Feb 16, 2015
 *      Author: mll
 */

#ifndef SRC_CONSTANTS_H_
#define SRC_CONSTANTS_H_

#define PI									3.141592653L
#define FLOAT_COMP_TOL						0.001 //must be within this margin for floats to be equal

//dio channels
#define CHAN_ENCODER_LEFT_A 				9
#define CHAN_ENCODER_LEFT_B					8
#define CHAN_ENCODER_RIGHT_A				7
#define CHAN_ENCODER_RIGHT_B				6
#define CHAN_FORK_LIMIT_MIN         		0
#define CHAN_FORK_LIMIT_MAX         		1
#define CHAN_LIFT_LIMIT_MIN        	 		2
#define CHAN_LIFT_LIMIT_MAX         		3
#define CHAN_ENCODER_LIFT           		4
#define CHAN_LEFT_DRIVE_TALONSR				8
#define CHAN_RIGHT_DRIVE_TALONSR			9

//analog channels
#define ACHAN_GEAR_COUNT            		3

//can ids
#define FORK_MOTOR_ID               		13
#define LIFT_MOTOR_ID               		12

//control parameters
#define ENCODER_RESOLUTION					1024.0
#define LIFT_PROPORTIONAL_TERM           	0.005
#define LIFT_INTEGRAL_TERM               	0.1
#define LIFT_DIFFERENTIAL_TERM           	0.001
#define DRIVE_PROPORTIONAL_TERM           	0.7
#define DRIVE_INTEGRAL_TERM               	0.5
#define DRIVE_DIFFERENTIAL_TERM           	0.1

//drive parameters
#define WHEEL_DIAMETER						4.0L
#define WHEEL_CIRCUMFERENCE         		(PI*WHEEL_DIAMETER)
#define DRIVE_ENCODER_CPR           		360
#define MAX_RPS								8

//motor directions and speeds
#define MOTOR_REV							-1
#define MOTOR_NOT_REV						1
#define FORK_MOTOR_REV_STATE				MOTOR_NOT_REV
#define LIFT_MOTOR_REV_STATE				MOTOR_NOT_REV
#define LEFT_INTAKE_MOTOR_REV_STATE			MOTOR_NOT_REV
#define RIGHT_INTAKE_MOTOR_REV_STATE		MOTOR_REV
//
#define	MOTOR_STOP							0.0
#define FORK_MOTOR_OUT_SPEED        		0.4 //out is positive
#define FORK_MOTOR_IN_SPEED         		-0.4 //in is negative
#define LIFT_MOTOR_UP_SPEED         		0.4 //up is positive
#define LIFT_MOTOR_DOWN_SPEED        		-0.4 //down is negative
#define INTAKE_MOTOR_SPEED					0.3 //intakes only move inwards

//gear tooth counts (integers)
//inner limit is the zero point
#define OPEN_NARROW_COUNT           		67 //distance from zero point
#define OPEN_WIDE_COUNT           			310 //distance from zero point
#define CLOSE_COUNT               			0 //distance from zero point, this is not actually the close position which will vary, this will just get the forks to close
#define FORK_OFFSET							55 //this is relative to the release position, not absolute
#define FORK_POS_TOL						3 //this is relative to a set position, not absolute

//lift encoder positions (float number of rotations)
//lower limit is the zero point
#define	PICKUP_POS							0.0 //distance from zero point
#define CARRY_ONE_POS						0.663 //distance from zero point
#define CARRY_TWO_POS						2.048 //distance from zero point
#define CARRY_THREE_POS						2.048 //distance from zero point
#define CARRY_STEP_POS						1.089 //distance from zero point
#define LIFT_OFFSET							0.5 //this is relative to the release position, not absolute
#define LIFT_POS_TOL						0.1 //number of encoder rotations relative to a set position, not absolute

//current limits
#define FORK_CURRENT_LIMIT  				25.0
#define INTAKE_CURRENT_LIMIT				25.0

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
#define ALL_LEDS							1023 //1023 = 2^10-1 to turn all 10 output leds
#define NO_LEDS								0

#endif /* SRC_CONSTANTS_H_ */
