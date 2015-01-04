/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#pragma once
#include "quadcore.h"
#include "../hw/hw.h"


void init_busses(void);
float get_timeint(void);

void init_orientation(struct quad_state *curr, struct quad_state *past, struct quad_static *stat);

void set_motor(struct quad_static *stat);

void read_control(struct quad_static *stat);

void kill(void);

void init_wdogs(void);

void clear_motor(struct quad_static *stat);

void print_motor(struct quad_static *stat);

void filter_data(struct quad_state *curr, struct quad_state *past);

void increase_motor_refresh(void);


