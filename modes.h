/*
Copyright (C) 2013  Andrew Pratt
View the README
*/

#pragma once
#include "imu.h"
#include "structures.h"
#include "uart.h"
#include "defines.h"
#include "quat.h"

//uses a PID controller to apply the calculated corrections to the motors
int apply_error(struct state *curr, struct state *past, struct motorval *motor, struct ipid *pid, struct controller *control);

// generates a quaternion representing a rotation from the global reference frame to the desired local reference frame
int control_quat(struct state *curr, struct state *past, struct controller *control);

// calculates simultaneous rotations on each axis that must be applied in order to rotate into the desired local reference frame
int qmode(struct state *curr);
