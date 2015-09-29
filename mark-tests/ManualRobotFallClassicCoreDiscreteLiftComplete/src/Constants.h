#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#define PRACTICE							0
#define COMPETITION							1
#define BUILD_VER							COMPETITION

#define PI									3.141592653L
#define ZERO_FL								0.0 //float zero

//dio channels
#define CHAN_ENCODER_LEFT_A 				9
#define CHAN_ENCODER_LEFT_B					8
#define CHAN_ENCODER_RIGHT_A				7
#define CHAN_ENCODER_RIGHT_B				6
#define CHAN_LIFT_ENCODER_A					5
#define CHAN_LIFT_ENCODER_B					4
#define CHAN_LIFT_LOW_LS 					3
#define CHAN_LIFT_HIGH_LS 					2

//pwm channels
#define CHAN_LEFT_DRIVE						8
#define CHAN_RIGHT_DRIVE					9

//can ids
#define CHAN_LIFT_MOTOR_BACK 				12
#define CHAN_LIFT_MOTOR_FRONT				14
#define CHAN_L_INTAKE_MOTOR					10
#define CHAN_R_INTAKE_MOTOR					13

//usb channels
#define CHAN_DRIVE_JS						0
#define CHAN_STEERING_WHEEL					1
#define CHAN_LIFT_SYS_JS					2

//drive joystick buttons
#define DRIVE_PID_OFF_BUTTON				11
#define DRIVE_PID_ON_BUTTON					10
#define DRIVE_NUDGE_LEFT_BUTTON				4
#define DRIVE_NUDGE_RIGHT_BUTTON			5

//lift joystick buttons
#define INTAKES_IN_BUTTON					2
#define INTAKES_OUT_BUTTON					3
#define INTAKES_OFF_BUTTON					1
#define LIFT_LOW_POS_BUTTON					7
#define LIFT_STEP_POS_BUTTON				10
#define LIFT_HIGH_POS_BUTTON				6
#define LIFT_PICKUP_BUTTON					8

//positional PID parameters for nudging
#if BUILD_VER == COMPETITION
#define POS_NUDGE_PROPORTIONAL_TERM          1.6f
#define POS_NUDGE_INTEGRAL_TERM              0.4f
#define POS_NUDGE_DIFFERENTIAL_TERM          0.0f
#elif BUILD_VER == PRACTICE
#define POS_NUDGE_PROPORTIONAL_TERM          0.8f
#define POS_NUDGE_INTEGRAL_TERM              0.05f
#define POS_NUDGE_DIFFERENTIAL_TERM          0.0f
#endif

//positional PID parameters for autonomous
#define POS_ERR_TOL							0.08 //this is a percentage
#define POS_TOL_COMP						0.0025 //this is a tuned value
#define POS_PROPORTIONAL_TERM          		0.8f
#define POS_INTEGRAL_TERM               	0.05f
#define POS_DIFFERENTIAL_TERM          		0.0f
#define ROT_PROPORTIONAL_TERM          		4.4f
#define ROT_INTEGRAL_TERM               	0.09f
#define ROT_DIFFERENTIAL_TERM          		0.0f

//lift PID parameters
#define LIFT_ENCODER_RESOLUTION 			1024
#define LIFT_ENCODER_DIST_PER_PULSE 		(1.0/LIFT_ENCODER_RESOLUTION)
#define LIFT_PROPORTIONAL_TERM           	2.0
#define LIFT_INTEGRAL_TERM               	0.5
#define LIFT_DIFFERENTIAL_TERM           	0.001
#define LIFT_PID_OUT_MIN					-0.7 //minimum controller output
#define LIFT_PID_OUT_MAX					0.7 //maximum controller output
#define QUARTER_INCH_OFFSET					0.056 //this offset to the lift encoder is a 1/2 in of vertical lift distance; used to prevent the lift from repeatedly hitting the top
#define LIFT_LOW_POS_OFFSET					QUARTER_INCH_OFFSET //lift encoder low position offset from the lift reference
#define LIFT_STEP_POS_OFFSET				8*(4*QUARTER_INCH_OFFSET) //lift encoder step position offset from the lift reference
#define LIFT_HIGH_POS_OFFSET				15*(4*QUARTER_INCH_OFFSET)  //lift encoder high position offset from the lift reference
#define LIFT_PID_ERR_TOL					0.5*QUARTER_INCH_OFFSET //allowable error tolerance for lift pid

//drive control
#define DRIVE_ENCODER_RESOLUTION			1024.0
#define DRIVE_ENCODER_DIST_PER_PULSE		(1.0/DRIVE_ENCODER_RESOLUTION)
#define WHEEL_DIAMETER						4.0L

//drive PID
#if BUILD_VER == COMPETITION
#define DRIVE_PROPORTIONAL_TERM           	0.005f
#define DRIVE_INTEGRAL_TERM               	0.1f
#define DRIVE_DIFFERENTIAL_TERM           	0.001f
#elif BUILD_VER == PRACTICE
#define DRIVE_PROPORTIONAL_TERM           	0.005f
#define DRIVE_INTEGRAL_TERM               	0.1f
#define DRIVE_DIFFERENTIAL_TERM           	0.001f
#endif

#define MAX_RPS								8
#define DRIVE_JS_DB_LOW 					-0.05 //drive joystick deadband low limit (not inclusive, i.e. not part of the deadband)
#define DRIVE_JS_DB_HIGH 					0.05 //drive joystick deadband high limit (not inclusive, i.e. not part of the deadband)
#define STEERING_DB_LOW 					-0.10 //steering wheel deadband low limit (not inclusive, i.e. not part of the deadband)
#define STEERING_DB_HIGH 					0.10 //steering wheel deadband high limit (not inclusive, i.e. not part of the deadband)

//configure the drive PID to be on or off; this overrides the joystick control of drive PID: if this is off the joystick control has no impact on drive PID, if this is on the joystick can turn drive PID on and off
#define GLOBAL_DRIVE_PID_OFF 				false
#define GLOBAL_DRIVE_PID_ON					true
#define GLOBAL_DRIVE_PID_CONFIG				GLOBAL_DRIVE_PID_ON

// autonomous
#define AUTONOMOUS_ON						true
#define AUTONOMOUS_MAX_FORWARD_SPEED 		0.65 //signed
#define AUTONOMOUS_MAX_REVERSE_SPEED		-0.65 //signed
#define AUTONOMOUS_MOVE_1INCH				-0.08//tire revolutions
#define AUTONOMOUS_MOVE_0_DIST				33*AUTONOMOUS_MOVE_1INCH
#define AUTONOMOUS_MOVE_1_DIST				14*AUTONOMOUS_MOVE_1INCH
#define AUTONOMOUS_MOVE_2_DIST				72*AUTONOMOUS_MOVE_1INCH
#define AUTONOMOUS_MOVE_3_DIST				72*AUTONOMOUS_MOVE_1INCH
#define AUTONOMOUS_MOVE_4_DIST				92*AUTONOMOUS_MOVE_1INCH
#define AUTONOMOUS_MOVE_5_DIST				28*AUTONOMOUS_MOVE_1INCH //do not invert sign; invert when used

//nudge
#define NUDGE_MAX_FORWARD_SPEED				0.4 //signed
#define NUDGE_MAX_REVERSE_SPEED				-0.4 //signed
#define NUDGE_MOVE_DIST						0.25 //tire revolutions right is pos, left is neg
#define NUDGE_PID_ERR_TOL					0.08 //allowable error tolerance for nudge pid

//current monitoring
#define INTAKE_CURRENT_LIMIT				15 //amps (not inclusive) - This is currently a conservative guess based on what we used for the fork current limit during competition season; this needs to be tuned

//non-drive motor configuration
#define MOTOR_REV							-1
#define MOTOR_NOT_REV						1
#define MOTOR_STOP							ZERO_FL
#define LIFT_MOTOR_REV_STATE				MOTOR_NOT_REV //both the back and forward lift motors are physically installed in the same direction and should be wired the same (i.e. some polarity) so they should rotate in the same direction and only 1 parameter is thus needed to set the reverse state for both
#define LEFT_INTAKE_MOTOR_REV_STATE			MOTOR_NOT_REV
#define RIGHT_INTAKE_MOTOR_REV_STATE		MOTOR_REV
#define INTAKE_IN_MOTOR_SPEED				0.5 //signed
#define INTAKE_OUT_MOTOR_SPEED				-0.3 //signed
#define LIFT_MOTOR_SPEED_DOWN				-0.3 //signed
#define LIFT_DB_LOW 						-0.2 //lift deadband low limit (not inclusive)
#define LIFT_DB_HIGH 						0.2 //lift deadband high limit (not inclusive)

#define DRIVE_VELOCITY_SCALE  				0.5 //fraction to scale the drive velocity; 0 = no motion; 0.5 = half speed; 1 = full speed

#endif /* _CONSTANTS_H_ */
