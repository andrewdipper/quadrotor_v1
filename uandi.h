/*
Copyright (C) 2013  Andrew Pratt
View the README
*/

#pragma once
#include "structures.h"
#include "imu.h"
#include "filters.h"
#include "LPC17xx.h"
#include "timer.h"
#include "quat.h"
#include "uart.h" //for debugging
#include "correct.h"


//The quadrotor should not be moved when this function is executed

//initializes current and past state structures along with other variables
//initializes hardware and sensors
//calculates attitude using accelerometers only
//reads magnetometer values to calculate gmagvect
//recalculates state multiple times to ensure the attitude estimation is correct before flight
void initialization(struct state *curr, struct state *past, float *gyrobias, struct vector *magvect);

//updates the quadrotor's state by reading sensor values and calculating the delta time for use in future calculations
//runs all the sensor values through a lowpass filter to remove noise
void update(struct state *curr, struct state *past, float *gyrobias);

//turns all motors off and increases ESC update period to 20 ms
//prints out watchdog to show that the quadrotor is shutting down due to a lack of control signals
void run_watchdog(void);

//send the motor values to the pwm hardware to send updated signals to the ESCs to update the motor speed
int set_motor(struct motorval *motor);

//read the pulses from the RC receiver and convert them into values stored in a controller structure
int pulse_read(struct controller *control);

//reduce the motor speed to zero
//1 ms pulse to ESC means no power to the motor
int clear_motor(struct motorval *motor);

//for debugging
void print_motor(struct motorval *motor);
void print_control(struct controller *control, struct pid *pid);
