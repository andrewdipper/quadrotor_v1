/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#include "modes.h"

//assuming 11059 is the center value and the 100% range is 8110-14008
//pitch has +- 30*

//generartes target quaternion from control inputs
int control_quat(struct quad_state *curr, struct quad_state *past, struct quad_static *stat){

	//convert control inputs to pitch roll and yaw angles
	if(stat->controlflag == 1){
		stat->controlflag = 0;
		float pitch = (stat->control[1] - 11059) * ( 3.14159265358979 / (2949.0 * 5.0));
		float roll = (stat->control[0] - 11059) * ( 3.14159265358979 / (2949.0 * 5.0));
		//float yaw = (stat->control[3] - 11059) * ( 3.14159265358979 / (2949.0 * 6.0));
		stat->throttle = (stat->control[2] - 8110) / 5.898;	

		/*
		outi0(pitch / 3.1415926535 * 180);
		out0("  ");
		outi0(roll / 3.1415926535 * 180);
		out0("  ");
		outi0(yaw / 3.1415926535 * 180);
		out0("\n");	
		*/

	
		//generate roll/pitch quaternion given control inputs
		struct quaternion tmpquat;
		float angle;
		if(pitch != 0 || roll != 0){
			angle = sqrt(pitch*pitch + roll*roll);
			pitch /= angle;
			roll /= angle;

			float sinangle = sin(angle/2.0);
			tmpquat.a = cos(angle/2.0);
			tmpquat.b = pitch * sinangle;
			tmpquat.c = roll * sinangle;
			tmpquat.d = 0;

		}else{
			tmpquat.a = 1;
			tmpquat.b = 0;
			tmpquat.c = 0;
			tmpquat.d = 0;
			
		}

		//make quaternion consisting of only the yaw component of bgquat
		struct quaternion yawquat;
		clone_quaternion(&curr->gbquat, &yawquat);
		yawquat.b = 0;
		yawquat.c = 0;
		normalize_quaternion(&yawquat);		

		//calculate target quaternion from yaw position and desired attitudes
		mult_quaternion(&yawquat, &tmpquat, &curr->gtquat); 	


	}else{
		//if no new control input use the past input
		clone_quaternion(&past->gtquat, &curr->gtquat);
	}

	

	return 0;
}


int rate_quat(struct quad_state *curr, struct quad_state *past, struct quad_static *stat){

	struct vector rpy;

	//get roll/pitch/yaw angular velocities from control input
	//these are taken as simultaneous orthagonal rotation angles so order is irrelevant
	rpy.x = (stat->control[1] - 11059) * ( 3.14159265358979 / (2949.0 * 5.0));
	rpy.y = (stat->control[0] - 11059) * ( 3.14159265358979 / (2949.0 * 5.0));
	rpy.z = (stat->control[3] - 11059) * ( 3.14159265358979 / (2949.0 * 6.0));

	stat->throttle = (stat->control[2] - 8110) / 5.898;	

	struct quaternion control_quat;
	gyroquat(&rpy, 1, &control_quat, NULL);
	
	mult_quaternion(&curr->gbquat, &control_quat, &curr->gtquat);

	return 0;
}



//attitude corrections are generated based on a target quaternion
int qmode(struct quad_state *curr){
	struct quaternion btquat;
	
	//make btquat the rotation from the body to the target
	mult_quaternion(&curr->bgquat, &curr->gtquat, &btquat); 


	//inverse sora doesn't handle (-a) very well because of arccos' range
	if(btquat.a < 0){	
		btquat.a *= -1;	
		btquat.b *= -1;	
		btquat.c *= -1;	
		btquat.d *= -1;	
	}
	
	inv_sora(&btquat, &curr->att_correct);		
	scale_vector(180/3.1415926535, &curr->att_correct); 	//converted into degrees of correction

	return 0;
}


//apply the attitude correction to the motors
int apply_correction(struct quad_state *curr, struct quad_state *past, struct quad_static *stat){

#ifdef BQUAD	
	//BQUAD settings
	stat->pidval.px = 2.8;
	stat->pidval.py = 2.8;
	stat->pidval.pz = 3.9;

	stat->pidval.dx = 1.8;
	stat->pidval.dy = 1.8;
	stat->pidval.dz = 2.7;

	stat->pidval.ix = .5;
	stat->pidval.iy = .5;
	stat->pidval.iz = .5;
#endif

	//TODO need to increase d term to deal with bad blade tracking old val 1.1
#ifdef SQUAD2S
	//SQUAD2S settings
	stat->pidval.px = 2.8;
	stat->pidval.py = 2.8;
	stat->pidval.pz = 2.5;

	stat->pidval.dx = 1.7;
	stat->pidval.dy = 1.7;
	stat->pidval.dz = 1.8;

	stat->pidval.ix = .7;
	stat->pidval.iy = .7;
	stat->pidval.iz = .7;
#endif

#ifdef SQUAD3S
	//SQUAD3S settings
	stat->pidval.px = 1.5;
	stat->pidval.py = 1.5;
	stat->pidval.pz = 2;

	stat->pidval.dx = 1.1;
	stat->pidval.dy = 1.1;
	stat->pidval.dz = 1.8;

	stat->pidval.ix = .5;
	stat->pidval.iy = .5;
	stat->pidval.iz = .5;
#endif


	struct ivector p, i, d;

	
	p.x = curr->att_correct.x * stat->pidval.px;
	p.y = curr->att_correct.y * stat->pidval.py;
	//p.z = curr->att_correct.z * stat->pidval.pz;
	float yaw = (stat->control[3] - 11059) * ( 180.0 / (2949.0 * 6.0));
	p.z = yaw * stat->pidval.pz;
	
	d.x = curr->sdata[1][0] * stat->pidval.dx * (180 / 3.1415926535897); 
	d.y = curr->sdata[1][1] * stat->pidval.dy * (180 / 3.1415926535897); 
	d.z = curr->sdata[1][2] * stat->pidval.dz * (180 / 3.1415926535897); 

	//TODO FIX
	if(0 && stat->throttle > 200){
		curr->int_correct.x = past->int_correct.x + (curr->dt * curr->att_correct.x * stat->pidval.ix);
		curr->int_correct.y = past->int_correct.y + (curr->dt * curr->att_correct.y * stat->pidval.iy);
		curr->int_correct.z = past->int_correct.z + (curr->dt * curr->att_correct.z * stat->pidval.iz);
	}else{
		clone_vector(&past->int_correct, &curr->int_correct);
	}

	clear_motor(stat); 

/*
* assumed motor configuration and coordinate axis
*	  1			y		
*	4-+-2		|		z axis is positive out of the screen
*	  3			|---x
*				  
*/

	//good	
	stat->motor[0] += p.x;
	stat->motor[1] -= p.y;
	stat->motor[2] -= p.x;
	stat->motor[3] += p.y;
	
	//good
	stat->motor[0] += p.z;
	stat->motor[1] -= p.z;
	stat->motor[2] += p.z;
	stat->motor[3] -= p.z;

/*	//???	
	stat->motor[0] += curr->int_correct.x;
	stat->motor[1] += curr->int_correct.y;
	stat->motor[2] -= curr->int_correct.x;
	stat->motor[3] -= curr->int_correct.y;
	
	//???
	stat->motor[0] += curr->int_correct.z;
	stat->motor[1] -= curr->int_correct.z;
	stat->motor[2] += curr->int_correct.z;
	stat->motor[3] -= curr->int_correct.z;
*/

	//good
	stat->motor[0] -= d.x;
	stat->motor[1] += d.y;
	stat->motor[2] += d.x;
	stat->motor[3] -= d.y;

	//good
	stat->motor[0] -= d.z;
	stat->motor[1] += d.z;
	stat->motor[2] -= d.z;
	stat->motor[3] += d.z;



	stat->motor[0] += stat->throttle;
	stat->motor[1] += stat->throttle;
	stat->motor[2] += stat->throttle;
	stat->motor[3] += stat->throttle;

	int j;
	for(j = 0; j < 4; ++j){

		if(stat->motor[j] < 0) stat->motor[j] = 0;
		if(stat->motor[j] > 990) stat->motor[j] = 990;
	}

#if defined(SQUAD2S) || defined(SQUAD3S)
//SQUAD motors are oriented in the opposite direction
	int tmp[4];
	for(j = 0; j < 4; ++j){
		tmp[j] = stat->motor[j];
	}
	for(j = 0; j < 4; ++j){
		stat->motor[j] = tmp[3-j];
	}

#endif


	return 0;
}
