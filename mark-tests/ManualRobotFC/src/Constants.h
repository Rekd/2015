#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#define PRACTICE							0
#define COMPETITION							1
#define PARADE								2
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
#define CHAN_FORK_MIN_LS 					0 //aka forkInner
#define CHAN_FORK_MAX_LS 					1 //aka forkOuter

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
#define CHAN_DRIVE_JS						0 //For parade build version, this is the only joystick used
#define CHAN_STEERING_WHEEL					1
#define CHAN_LIFT_SYS_JS					2

//drive joystick buttons
#if BUILD_VER == COMPETITION || BUILD_VER == PRACTICE
#define DRIVE_PID_OFF_BUTTON				11
#define DRIVE_PID_ON_BUTTON					10
#define DRIVE_NUDGE_LEFT_BUTTON				4
#define DRIVE_NUDGE_RIGHT_BUTTON			5
#else //parade: these don't apply to parade
#define DRIVE_PID_OFF_BUTTON				12 //there is no joystick button 12
#define DRIVE_PID_ON_BUTTON					12
#define DRIVE_NUDGE_LEFT_BUTTON				12
#define DRIVE_NUDGE_RIGHT_BUTTON			12
#endif

//lift joystick buttons
#if BUILD_VER == COMPETITION || BUILD_VER == PRACTICE
#define BUT_FORKS_IN						5
#define BUT_FORKS_OUT						4
#define BUT_FORKS_STOP						1
#define INTAKES_ON_BUTTON					2
#define INTAKES_OFF_BUTTON					3
#else //parade
#define BUT_FORKS_IN						4
#define BUT_FORKS_OUT						5
#define BUT_FORKS_STOP						1
#define INTAKES_ON_BUTTON					12 //Intakes don't apply to parade; there is no joystick button 12
#define INTAKES_OFF_BUTTON					12
#define BUT_LIFT_UP							3
#define BUT_LIFT_DOWN						2
#endif

//drive wheel buttons
#if BUILD_VER == COMPETITION || BUILD_VER == PRACTICE
#define DRIVE_NUDGE_WHEEL_LEFT_BUTTON		5
#define DRIVE_NUDGE_WHEEL_RIGHT_BUTTON		6
#else //there is no drive wheel for parade
#define DRIVE_NUDGE_WHEEL_LEFT_BUTTON		12 //there is no joystick button 12
#define DRIVE_NUDGE_WHEEL_RIGHT_BUTTON		12
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

//positional PID parameters for nudging
#if BUILD_VER == PRACTICE
#define POS_NUDGE_PROPORTIONAL_TERM          0.8f
#define POS_NUDGE_INTEGRAL_TERM              0.05f
#define POS_NUDGE_DIFFERENTIAL_TERM          0.0f
#elif BUILD_VER == COMPETITION || BUILD_VER == PARADE //these are defined by not used for parade
#define POS_NUDGE_PROPORTIONAL_TERM          1.6f
#define POS_NUDGE_INTEGRAL_TERM              0.4f
#define POS_NUDGE_DIFFERENTIAL_TERM          0.0f
#endif

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
#define HALF_INCH_OFFSET					0.056 //this number of revolutions is a 1/2 in of vertical lift distance
#define WHEEL_DIAMETER						4.0L
#define WHEEL_CIRCUMFERENCE         		(PI*WHEEL_DIAMETER)
#define DRIVE_ENCODER_CPR          		 	360

//drive PID
#if BUILD_VER == PRACTICE
#define PROPORTIONAL_TERM           		0.005f
#define INTEGRAL_TERM               		0.1f
#define DIFFERENTIAL_TERM           	0.001f
#elif BUILD_VER == COMPETITION || BUILD_VER == PARADE
#define PROPORTIONAL_TERM           		0.005f
#define INTEGRAL_TERM               		0.1f
#define DIFFERENTIAL_TERM           	0.001f
#endif

#define MAX_RPS								8
#define DRIVE_DB_LOW 						-0.05 //drive deadband low limit
#define DRIVE_DB_HIGH 						0.05 //drive deadband high limit
#define STEERING_DB_LOW 						-0.10 //drive deadband low limit
#define STEERING_DB_HIGH 						0.10 //drive deadband high limit

#define PID_OFF 							false
#define PID_ON								true
#define PID_CONFIG							PID_ON

//autonomous
#if BUILD_VER == COMPETITION || BUILD_VER == PRACTICE
//#define AUTONOMOUS_MAX_FORWARD_SPEED 		0.6 //signed
//#define AUTONOMOUS_MAX_REVERSE_SPEED		-0.6 //signed
#define AUTONOMOUS_MAX_FORWARD_SPEED 		0.2 //signed
#define AUTONOMOUS_MAX_REVERSE_SPEED		-0.2 //signed
#define AUTONMOUS_MOVE_DIST					-0.73*1.2 //tire revolutions; for 4 in wheels ~6 feet; - for backwards, + for forwards
#else //parade
#define AUTONOMOUS_MAX_FORWARD_SPEED 		0.0 //signed
#define AUTONOMOUS_MAX_REVERSE_SPEED		0.0 //signed
#define AUTONMOUS_MOVE_DIST					0.0 //tire revolutions; for 4 in wheels 0 feet; - for backwards, + for forwards
#endif

//nudge
#define NUDGE_MAX_FORWARD_SPEED				0.4 //signed
#define NUDGE_MAX_REVERSE_SPEED				-0.4 //signed
#define NUDGE_MOVE_DIST						0.25 //tire revolutions right is pos, left is neg

//current monitoring
#if BUILD_VER == COMPETITION || BUILD_VER == PARADE
#define FORK_CURRENT_LIMIT					32.0
#else //practice
#define FORK_CURRENT_LIMIT					25.0
#endif

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
#if BUILD_VER == COMPETITION || BUILD_VER == PARADE
#define LIFT_MOTOR_SPEED_UP					0.6 //unsigned, set sign when used
#else //practice
#define LIFT_MOTOR_SPEED_UP					0.25 //unsigned, set sign when used
#endif
#define LIFT_MOTOR_SPEED_DOWN				0.2 //unsigned, set sign when used
#define LIFT_DB_LOW 						-0.2 //lift deadband low limit
#define LIFT_DB_HIGH 						0.2 //lift deadband high limit
#define AT_TOP_LIFT_DUR						0.5 //duartion in seconds that lift up motion will be prevented if the top limit switch is hit

#if BUILD_VER == COMPETITION || BUILD_VER == PRACTICE
#define VELOCITY_SCALE  					0.5
#else //parade
#define VELOCITY_SCALE  					0.4 //slower for parade
#endif

#endif /* _CONSTANTS_H_ */