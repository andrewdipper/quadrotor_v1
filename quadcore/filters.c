/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#include "filters.h"

float lowpass(float past, float input, float dt, float k){
 				
	float tk = dt/(dt + k); 					
	return past + tk*(input - past);
}

