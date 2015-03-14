#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#define PRACTICE							0
#define COMPETITION							1
#define BUILD_VER							COMPETITION

#define PI									3.141592653L
#define ZERO_FL								0.0 //float zero
#define SEC_IN_NANOSEC						0.000000001 //1e-9

//dio channels
#define CHAN_ENCODER_LEFT_A 				9
#define CHAN_ENCODER_LEFT_B					8
#define CHAN_ENCODER_RIGHT_A				7
#define CHAN_ENCODER_RIGHT_B				6
#define CHAN_LIFT_ENCODER_A					5
#define CHAN_LIFT_ENCODER_B					4
#define CHAN_LIFT_LOW_LS 					3
#define CHAN_LIFT_HIGH_LS 					2
#define CHAN_FORK_MIN_LS 					0 //rename to forkInner
#define CHAN_FORK_MAX_LS 					1 //rename to forkMax

//pwm channels
#define CHAN_LEFT_DRIVE						8
#define CHAN_RIGHT_DRIVE					9

//can ids
#define CHAN_LIFT_MOTOR_BACK 				12 //can
#define CHAN_LIFT_MOTOR_FRONT				14 //can
#define CHAN_FORK_MOTOR 					13 //can
#define CHAN_L_INTAKE_MOTOR					10 //can
#define CHAN_R_INTAKE_MOTOR					11 //can

//usb channels
#define CHAN_DRIVE_JS						0
#define CHAN_STEERING_WHEEL					1
#define CHAN_LIFT_SYS_JS					2

//buttons
#define BUT_FORKS_IN						5
#define BUT_FORKS_OUT						4
#define BUT_FORKS_STOP						1
#define INTAKES_ON_BUTTON					2
#define INTAKES_OFF_BUTTON					3

//positional PID parameters
#define POS_ERR_TOL							0.05 //this is a percentage
#define POS_TOL_COMP						0.0025 //this is a tuned value
#define POS_PROPORTIONAL_TERM          		0.8f
#define POS_INTEGRAL_TERM               	0.05f
#define POS_DIFFERENTIAL_TERM          		0.0f

//lift PID parameters
#define LIFT_ENCODER_RESOLUTION 			1024
#define LIFT_ENCODER_DIST_PER_PULSE 		(1.0/LIFT_ENCODER_RESOLUTION)
#define LIFT_PROPORTIONAL_TERM           	2.0
#define LIFT_INTEGRAL_TERM               	0.5
#define LIFT_DIFFERENTIAL_TERM           	0.001
#define LIFT_PID_OUT_MIN					-0.1 //minimum controller output
#define LIFT_PID_OUT_MAX					0.7 //maximum controller output

//drive control
#define ENCODER_RESOLUTION					1024.0
#define ENCODER_DIST_PER_PULSE				(1.0/ENCODER_RESOLUTION)
#define WHEEL_DIAMETER						4.0L
#define WHEEL_CIRCUMFERENCE         		(PI*WHEEL_DIAMETER)
#define DRIVE_ENCODER_CPR          		 	360
#if 0
#define PROPORTIONAL_TERM          			0.7f
#define INTEGRAL_TERM               		1.3f
#define DIFFERENTIAL_TERM          			0.1f
#endif
#define PROPORTIONAL_TERM           		0.005f
#define INTEGRAL_TERM               		0.1f
#define DIFFERENTIAL_TERM           	0.001f

#define MAX_RPS								8
#define DRIVE_DB_LOW 						-0.05 //drive deadband low limit
#define DRIVE_DB_HIGH 						0.05 //drive deadband high limit
#define PID_OFF 							false
#define PID_ON								true
#define PID_CONFIG							PID_ON

//autonomous
#define AUTONOMOUS_MAX_FORWARD_SPEED 		0.6 //signed
#define AUTONOMOUS_MAX_REVERSE_SPEED		-0.6 //signed
#define AUTONMOUS_MOVE_DIST					-5.73*1.15 //tire revolutions; for 4 in wheels ~6 feet; - for backwards, + for forwards

//current monitoring
#define FORK_CURRENT_LIMIT					10.0
#define INTAKE_CURRENT_LIMIT				0.35

//non-drive motor configuration
#define MOTOR_REV							-1
#define MOTOR_NOT_REV						1
#define FORK_MOTOR_REV_STATE				MOTOR_REV
#define LIFT_MOTOR_REV_STATE				MOTOR_NOT_REV //both the back and forward lift motors are in the same direction
#define LEFT_INTAKE_MOTOR_REV_STATE			MOTOR_REV
#define RIGHT_INTAKE_MOTOR_REV_STATE		MOTOR_REV

//for intakes + in inwards; intakes only move inwards
//for forks + is outwards and - is inwards
//for lifts + is up and - is down (although can not actively command down as described below)
////for joystick control of lift + is backwards and - is forwards
//
//set all sppeds to unsigned values and set the sign when using the value, do not the sign in the #define
#define MOTOR_STOP							ZERO_FL
#define INTAKE_MOTOR_SPEED					0.3 //unsigned, set sign when used although per the discussion above should only be used positive
#define FORK_MOTOR_SPEED_OUT				0.7 //unsigned, set sign when used
#define FORK_MOTOR_SPEED_IN					0.7 //unsigned, set sign when used
// original #define LIFT_MOTOR_SPEED_UP					0.4 //unsigned, set sign when used
#define LIFT_MOTOR_SPEED_UP					0.6 //unsigned, set sign when used
#define LIFT_MOTOR_SPEED_DOWN				0.2 //unsigned, set sign when used
#define LIFT_DB_LOW 						-0.1 //lift deadband low limit
#define LIFT_DB_HIGH 						0.1 //lift deadband high limit
#define AT_TOP_LIFT_DUR						2.0 //duartion in seconds that lift up motion will be prevented if the top limit switch is hit

#endif
