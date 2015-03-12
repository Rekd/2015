/*
 * Constants.h
 *
 *  Created on: Feb 16, 2015
 *      Author: mll
 */

#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#define PRACTICE 							0
#define COMPETITION 						1
#define BUILD_VERSION 						PRACTICE

#define PID_OFF 							false
#define PID_ON 								true
#define PID_CONFIG 							PID_OFF

#define PI									3.141592653L
#define ZERO_FL								0.0 //float zero
#define FLOAT_COMP_TOL						0.001 //must be within this margin for floats to be equal
#define STAT_STR_LEN 						64 //printing to dashboard
#define UNINIT_VAL 							-1 //uninitialized valued

//dio channels
#define CHAN_ENCODER_LEFT_A 				9
#define CHAN_ENCODER_LEFT_B					8
#define CHAN_ENCODER_RIGHT_A				7
#define CHAN_ENCODER_RIGHT_B				6
#define CHAN_LIFT_ENCODER_A                 5
#define CHAN_LIFT_ENCODER_B                 4
#define CHAN_FORK_MIN_LS	         		0
#define CHAN_FORK_MAX_LS	         		1
#define CHAN_LIFT_LOW_LS        	 		2
#define CHAN_LIFT_HIGH_LS	         		3

//pwm channels
#define CHAN_LEFT_DRIVE						8
#define CHAN_RIGHT_DRIVE					9

//analog channels
#define ACHAN_GEAR_COUNT            		3

//can ids
#define FORK_MOTOR_ID               		13
#define LIFT_MOTOR_ID               		12
#define L_INTAKE_MOTOR_ID					11
#define R_INTAKE_MOTOR_ID					14

//usb channels
#define CHAN_DRIVE_JS						0
#define CHAN_STEERING_WHEEL					1
#define CHAN_OPERATOR_BOX					2

//gear tooth analog configuration
#define GEAR_TRIGGER_MIN					450  // was 450 //0 to 4096 representing 0V to 5V
#define GEAR_TRIGGER_MAX					2000

//drive control parameters
#define ENCODER_RESOLUTION					1024.0
#define DRIVE_PROPORTIONAL_TERM           	0.7
#define DRIVE_INTEGRAL_TERM               	0.5
#define DRIVE_DIFFERENTIAL_TERM           	0.1
#define WHEEL_DIAMETER						4.0L
#define WHEEL_CIRCUMFERENCE         		(PI*WHEEL_DIAMETER)
#define DRIVE_ENCODER_CPR           		360
#define MAX_RPS								8
#define DRIVE_DB_LOW 						-0.1 //drive deadband low limit
#define DRIVE_DB_HIGH 						0.1 //drive deadband high limit

//lift control parameters
#define LIFT_ENCODER_RESOLUTION 			1024
#define LIFT_ENCODER_DIST_PER_PULSE 		(1.0/LIFT_ENCODER_RESOLUTION)
#define LIFT_PROPORTIONAL_TERM           	2.0
#define LIFT_INTEGRAL_TERM               	0.5
#define LIFT_DIFFERENTIAL_TERM           	0.001
#define LIFT_PID_OUT_MIN 					-0.1 //minimum controller output
#define LIFT_PID_OUT_MAX 					0.7 //maximum controller output

//motor directions and speeds
#define MOTOR_REV							-1
#define MOTOR_NOT_REV						1
#define FORK_MOTOR_REV_STATE				MOTOR_REV
#define LIFT_MOTOR_REV_STATE				MOTOR_NOT_REV
#define LEFT_INTAKE_MOTOR_REV_STATE			MOTOR_NOT_REV
#define RIGHT_INTAKE_MOTOR_REV_STATE		MOTOR_REV
//
#define	MOTOR_STOP							ZERO_FL
#define FORK_MOTOR_OUT_SPEED        		0.6 //out is positive
#define FORK_MOTOR_IN_SPEED         		-0.6 //in is negative
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
#define	PICKUP_POS							0.1 //distance from zero point
#define CARRY_ONE_POS						0.663 //distance from zero point
#define CARRY_TWO_POS						2.048 //distance from zero point
#define CARRY_THREE_POS						2.048 //distance from zero point
#define CARRY_STEP_POS						1.089 //distance from zero point
#define LARGE_NEG_POS						-99.0 //used for zeroing the lift
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
