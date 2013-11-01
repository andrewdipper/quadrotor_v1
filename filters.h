/*
Copyright (C) 2013  Andrew Pratt
View the README
*/

#pragma once
#include "structures.h"

//lowpass filter for the sensor readings
float lowpass(float past, float input, float dt, float k);




