/*
 * DSBox.cpp
 *
 *  Created on: Feb 13, 2015
 *      Author: mll
 */

#include <DSBox.h>

DSBox::DSBox(DriverStation *io, LiftSystem *ls) {
	dsIO = io;
	liftSystem = ls;
	openWideLED = true;
	openNarrowLED = false;
	intakeLED = false;
	closeLED = false;
	carryPLED = false;
	carryPTLED = false;
	carryPTTLED = false;
	carrySLED = false;
	releaseLED = false;
	pidOverrideLED = false;
}



DSBox::~DSBox() {
	delete dsIO;
}

/*
 * Update(): called on a regular basis to read and update the Drive Station operator box
 */
void DSBox::Update()
{
	SetPIDStateLEDOn(GetPIDEnabled());  // check state of PID Override toggle
}

/*
 * Publicly accessable functions
 */

bool DSBox::GetPIDEnabled() {
	return !dsIO->GetDigitalIn(BOX_PID_SWITCH);
}

bool DSBox::GetOpenWideButton() {
	return !dsIO->GetDigitalIn(BOX_OPEN_WIDE_BUTTON);
}

////// Add the rest here
