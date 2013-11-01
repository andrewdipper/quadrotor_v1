#include "correct.h"



/*  	Takes the vertical unit vector in global coordinates and rotates the coordinate system to body coordinates. The cross product
*	between the vector and the accelerometer readings gives an anglular velocity vector to rotate the vectors together (correction for the *	quaternion) in one unit of time with the following assumption:
*	when x is small sin(x) = x
*	
*/							

void correct_acc(struct state *curr, struct state *past){  


		

	struct vector bz, temp, corr;
	temp.x = 0;
	temp.y = 0;
	temp.z = 1;
	
	vmult_quaternion(past->bgquat, &temp , &bz);  

	cross_product((struct vector *)curr->val[ACC],  &bz, &corr); 

	scale_vector(curr->corrweight->acccorr, &corr);			//applies the gain for the correction

	add_vector(&corr, curr->corrv, curr->corrv);
}

/*	Same as the accelerometer correction except the the magnetometer reading is treated as an arbitrary vector
*	instead of one aligning with a coordinate axis.  gmagvect is the arbitrary vector calculated in the initialization
*	procedure in global coordinates.  
*	Treating the magnetometer as measuring an arbitrary vector avoids problems caused by magnetic inclination and declination
*/

void correct_mag(struct state *curr, struct state *past, struct vector *gmagvect){

	
	
	struct vector bmagvect, corr;

	vmult_quaternion(past->bgquat, gmagvect, &bmagvect); 

	cross_product((struct vector *)curr->val[MAG],  &bmagvect,  &corr);	
	
	scale_vector(curr->corrweight->magcorr, &corr);			//applies the gain for the correction

	add_vector(&corr, curr->corrv, curr->corrv);
}


void clear_correction(struct state *s){	//used to clear old correction before correct_acc and correct_mag calls
	s->corrv->x = 0;
	s->corrv->y = 0;
	s->corrv->z = 0;
}
