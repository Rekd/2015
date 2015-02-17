/*
 * gyro-i2c.c
 *
 *  Created on: Jan 27, 2015
 *      Author: mll
 */
#include "WPILib.h"
#include "Robot.h"
#include "ITG3200.h"
#include "gyro-i2c.h"
#include "I2C.h"


I2C  *gyroI2C;

void sbi (char port_unused, char address_unused)
{
	gyroI2C = new I2C(I2C::kOnboard, ITG_ADDR);  // create the gyroI2C object
	return;
}


void twiInit(unsigned long scl_freq_unused)
{
	return;  // null function
}


char twiTransmit(char sla, char reg_addr, char value)
{
	bool  ret; //Transfer Aborted... false for success, true for aborted.

	ret = gyroI2C->Write(reg_addr, value);
	if (ret == TRUE)
		return(0);
	else
		return(1);
}


char twiReceive(char sla, char reg_addr, char * value)
{
	bool  ret; //Transfer Aborted... false for success, true for aborted.

	ret = gyroI2C->Read((unsigned char)reg_addr, 1, (unsigned char*)value);
	if (ret == TRUE)
		return(0);
	else
		return(1);

}


void twiReset(void)
{
	return;  // null function
}
