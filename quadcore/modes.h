/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#pragma once
#include "quadcore.h"


int control_quat(struct quad_state *curr, struct quad_state *past, struct quad_static *stat);
int qmode(struct quad_state *curr);

int apply_correction(struct quad_state *curr, struct quad_state *past, struct quad_static *stat);

