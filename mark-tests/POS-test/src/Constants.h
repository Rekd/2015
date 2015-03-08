#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#define PI							3.141592653L

#define CHAN_ENCODER_LEFT_A 		9
#define CHAN_ENCODER_LEFT_B			8
#define CHAN_ENCODER_RIGHT_A		7
#define CHAN_ENCODER_RIGHT_B		6
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

#endif
