/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#include "correct.h"
#include "../hw/uart.h"

#define ACC_SCALE .9			//how much of the difference the quaternion is rotated by each second
#define MAG_SCALE .9

void clear_correction(struct quad_state *curr){
	curr->correct_vect.x = 0;
	curr->correct_vect.y = 0;
	curr->correct_vect.z = 0;
}

void accel_correction(struct quad_state *curr, struct quad_state *past, struct quad_static *stat){

	struct vector tmp = {0, 0, 1};
	struct vector gz, correct;

	//print_quaternion(&past->bgquat);

	vmult_quaternion(&past->bgquat, &tmp, &gz);

	cross_product((struct vector *)curr->sdata[0], &gz,  &correct);

	//print_vect((struct vector *)curr->sdata[0]);
	//print_vect(&gz);
	//print_vect(&correct);

	scale_vector(stat->acc_scale, &correct);

	add_vector(&curr->correct_vect, &correct, &curr->correct_vect);
/*
*	By changing from global to body coordinates the tmp vector is implicitly rotated by A (the rotation from the global coordinate system to the body).
*	To correct for the implicit rotation the vector is rotated backwards by -gbquat (gbquat is the estimate of the true rotation A).
*	The result is stored in gz
*	The cross product of (gz, ACC) gives the rotation R such that  R*(-gbquat) = A
*	So to correct gbquat:   -(gbquat*(-R)) = A
* 	Hence the opposite cross product is applied (Acc, gz) to get negative R
*	The correction R is applied to gbquat when the update quaternion is calculated
*	
*	The scale value is used to determine how much the system is adjusted by the accelerometer input
*/

}






void mag_correction(struct quad_state *curr, struct quad_state *past, struct quad_static *stat){
 
	struct vector gmag, correct;

	vmult_quaternion(&past->bgquat, &stat->gmagvect, &gmag);

	cross_product((struct vector *)curr->sdata[2], &gmag, &correct);

	scale_vector(stat->mag_scale, &correct);

	//print_vect(&correct);
	//out0("mag");
	//print_vect(&correct);
	add_vector(&curr->correct_vect, &correct, &curr->correct_vect);


/*
* 	Same theory as the accelerometer correction except the initial magnetic field vector is calculated at startup.
*
*/

}


