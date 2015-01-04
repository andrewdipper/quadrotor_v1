/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#pragma once
#include "hw/hw.h"
#include "quadcore.h"




void accel_init(void);
void gyro_init(void);
void mag_init(void);

void accel_parse(char *read, int *data);
void gyro_parse(char *read, int *data);
void mag_parse(char *read, int *data);

void gyro_calc_bias(void);


void sensor_init(void);
void sensor_queue(void);
int sensor_status(void);
void sensor_print(void);
int sensor_parse(float data[3][3]);

void gmagvect_compute(struct quad_state *curr, struct quad_static *stat);





void print_sensor_status(void);
