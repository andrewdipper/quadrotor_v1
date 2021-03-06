/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#pragma once
#include "quadcore.h"

void clear_correction(struct quad_state *curr);

void accel_correction(struct quad_state *curr, struct quad_state *past, struct quad_static *stat);
void mag_correction(struct quad_state *curr, struct quad_state *past, struct quad_static *stat);
