#pragma once
#include "quat.h"
#include "structures.h"
#include "imu.h"
#include "uart.h"

//clears correction information
void clear_correction(struct state *s);

//calculates correction for the bgquat quaternion based on accelerometer readings
void correct_acc(struct state *curr, struct state *past);

//calculates correction for the bgquat quaternion based on magnetometer readings
void correct_mag(struct state *curr, struct state *past, struct vector *gmagvect);
