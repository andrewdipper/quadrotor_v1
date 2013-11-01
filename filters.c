/*
Copyright (C) 2013  Andrew Pratt
View the README
*/

#include "filters.h"
#include "structures.h"


//gain approaches 1 as dt increases -> trust measurement increasingly more as the time increases

float lowpass(float past, float input, float dt, float k){  	//k is the filter gain
 				
	float tk = dt/(dt + k); 					
	return past + tk*(input - past);
}



