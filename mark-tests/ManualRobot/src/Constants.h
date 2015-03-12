#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#define PI									3.141592653L
#define ZERO_FL								0.0 //float zero

//dio channels
#define CHAN_ENCODER_LEFT_A 				9
#define CHAN_ENCODER_LEFT_B					8
#define CHAN_ENCODER_RIGHT_A				7
#define CHAN_ENCODER_RIGHT_B				6
#define CHAN_LIFT_LOW_LS 					3
#define CHAN_LIFT_HIGH_LS 					2
#define CHAN_FORK_MIN_LS 					0
#define CHAN_FORK_MAX_LS 					1

//pwm channels
#define CHAN_LEFT_DRIVE						8
#define CHAN_RIGHT_DRIVE					9

//can ids
#define CHAN_LIFT_MOTOR 					12 //can
#define CHAN_FORK_MOTOR 					13 //can
#define CHAN_L_INTAKE_MOTOR					11 //can
#define CHAN_R_INTAKE_MOTOR					14 //can

//usb channels
#define CHAN_DRIVE_JS						0
#define CHAN_STEERING_WHEEL					1
#define CHAN_LIFT_SYS_JS					2

//buttons
#define BUT_JS_IN							1
#define BUT_JS_OUT							2
#define INTAKE_BUTTON						3

//drive control
#define ENCODER_RESOLUTION					1024.0
#define WHEEL_DIAMETER						4.0L
#define WHEEL_CIRCUMFERENCE         		(PI*WHEEL_DIAMETER)
#define DRIVE_ENCODER_CPR          		 	360
#define PROPORTIONAL_TERM          			0.7f
#define INTEGRAL_TERM               		1.3f
#define DIFFERENTIAL_TERM          			0.1f
#define MAX_RPS								8
#define DRIVE_DB_LOW 						-0.1 //drive deadband low limit
#define DRIVE_DB_HIGH 						0.1 //drive deadband high limit

//current monitoring
#define FORK_CURRENT_LIMIT					25.0
#define INTAKE_CURRENT_LIMIT				25.0

//non-drive motor configuration
#define MOTOR_STOP							ZERO_FL
#define MOTOR_REV							-1
#define MOTOR_NOT_REV						1
#define FORK_MOTOR_REV_STATE				MOTOR_REV
#define LIFT_MOTOR_REV_STATE				MOTOR_NOT_REV
#define LEFT_INTAKE_MOTOR_REV_STATE			MOTOR_NOT_REV
#define RIGHT_INTAKE_MOTOR_REV_STATE		MOTOR_REV

//for intakes + in inwards; intakes only move inwards
//for forks + is outwards and - is inwards
//for lifts + is up and - is down (although can not actively command down as described below)
////for joystick control of lift + and - are up; there is no way to actively drive the joystick down:
////to get the joystick down set to stop moving and gravity will pull down
//
//set all sppeds to unsigned values and set the sign when using the value, do not the sign in the #define
#define INTAKE_MOTOR_SPEED					0.3
#define FORK_MOTOR_SPEED_OUT				0.5
#define FORK_MOTOR_SPEED_IN					0.5
#define LIFT_MOTOR_SPEED_UP					0.8
//there is no LIFT_MOTOR_SPEED_DOWN since the lift can not be actively driven down (see the description above)
#define LIFT_DB_HIGH 						0.1 //lift deadband high limit
//lift speed is always positive so there is no lift deadband low limit

#endif
