/*
 * Robot.h
 *
 *  Created on: Feb 13, 2015
 *      Author: mll
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

enum States { open_wide, open_narrow, closed_p, closed_pt, closed_ptt, closed_s, released,
	pickup_failed, lift_error, gripper_error };

bool PidOverride;  // true means no PID, false means use PID



#endif /* SRC_ROBOT_H_ */
