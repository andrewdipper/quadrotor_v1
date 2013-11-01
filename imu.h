/*
Copyright (C) 2013  Andrew Pratt
View the README
*/
#pragma once

#include "i2c.h"
#include "structures.h"

//defines for indexing sensor readings
#ifndef X
#define X 0
#define Y 1
#define Z 2
#endif

#ifndef ACC
#define ACC 0
#define GYR 1
#define MAG 2
#endif


void accel_init(void);  	//configure ADXL345
void gyro_init(void);		//configure ITG3200
void mag_init(void);		//configure HMC5883L

void accel_read(short *acc);	//read X, Y, and Z values from accelerometer
void gyro_read(short *gyr);	//read X, Y, and Z values from gyroscope
void mag_read(short *mag);	//read X, Y, and Z values from magnetometer

void calc_gbias(float *gbias);	//calculate gyroscope zero rotation error

void read_to_val(short *a, float *b);	//convert integer readings to floats
