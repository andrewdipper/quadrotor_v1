#pragma once

#include "LPC17xx.h"
#include "uart.h"

//controls the I2C busses .. only implemented for I2C0

//initialize the bus to default settings
void i2c_init(I2C_TypeDef *I2C);

//send start condition to the bus
inline int i2c_start(I2C_TypeDef *I2C);

//transmit byte on bus
int i2c_tx(I2C_TypeDef *I2C, unsigned char data);

//set register "reg" to "data" on device with address "addr" /
//useful for sensors
void i2c_set(I2C_TypeDef *I2C, unsigned char addr, unsigned char reg, unsigned char data);

//receive a byte on the bus
int i2c_rx(I2C_TypeDef *I2C, unsigned char *data);

//send stop condition to the bus
inline void i2c_stop(I2C_TypeDef *I2C);


