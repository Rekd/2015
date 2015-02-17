/*
 * gyri-i2c.h
 *
 *  Created on: Jan 27, 2015
 *      Author: mll
 *
 *  This is a shim file to allow ITG3200.c/h to work without changes.
 *  It simulates the functionality of SparkFun's TWI Interface Library
 */

#ifndef SRC_GYRO_I2C_H_
#define SRC_GYRO_I2C_H_

#include "WPILib.h"


void twiInit(unsigned long scl_freq_unused);
char twiTransmit(char sla, char reg_addr, char value);
char twiReceive(char sla, char reg_addr, char * value);
void twiReset(void);
void sbi (char port_unused, char address_unused);

#define GYRO_INT_PIN  6  // set this appropriately
#define PORTD	GYRO_INT_PIN  // needed by ITG3200.cpp
#define GYRO_ADO	5  // needed by ITG3200.cpp

#endif /* SRC_GYRO_I2C_H_ */
