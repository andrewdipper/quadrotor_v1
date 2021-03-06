/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#pragma once
#include "mk20dx128.h"
#include "pins.h"

void i2c0_init(void);
void i2c0_start(void);
void i2c0_restart(void);
void i2c0_stop(void);
int i2c0_tx(char c);
int i2c0_rx(char *data);
int i2c0_set(char addr, char reg, char data);
void i2c0_nack_next(void);

void i2c0_write(char addr, char *data, int len);
void i2c0_writebyte(char addr, char data);
void i2c0_read(char addr, char *data, int len);
