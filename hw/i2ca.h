/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#pragma once
#include "mk20dx128.h"
#include "pins.h"
#include "uart.h"
#include "i2c.h"

#define I2C_READ 	1
#define I2C_WRITE 	2
#define I2C_WAITING	1
#define I2C_RUNNING	2
#define I2C_FINISHED	3

struct i2c_transfer{
	int type;
	char address;
	int buflen;
	char *buffer;
	int bufind;	
	int status;
};

void i2ca0_init(void);
void i2c0_isr(void);
int i2ca0_qtrans(struct i2c_transfer *i2ctrans);
void i2ca0_purge(void);

void i2ca0_recover(void);


void message(char *msg);
void pdat(char *msg, char data);

