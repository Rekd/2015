/*
 * DSBox.h
 *
 *  Created on: Feb 13, 2015
 *      Author: mll
 */

#ifndef SRC_DSBOX_H_
#define SRC_DSBOX_H_
#include "LiftSystem.h"

class DSBox {
public:
	DSBox();
	virtual ~DSBox();

private:
	DriverStation *dsIO;
	LiftSystem *liftSystem;
	bool openWideLED;
	bool openNarrowLED;
	bool intakeLED;
	bool closeLED;
	bool carryPLED;
	bool carryPTLED;
	bool carryPTTLED;
	bool carrySLED;
	bool releaseLED;
	bool pidOverrideLED;
};

#endif /* SRC_DSBOX_H_ */
