/*
Copyright (C) 2013  Andrew Pratt
View the README
*/

#include "modes.h"

/*
*	Generates a quaternion (gdquat) from the rc controller inputs that represents the rotation from the global
*	reference frame to the desired local reference frame for the quadrotor.  The yaw input is applied first and then
*	the roll and pitch inputs are applied to make the yaw input more intuitive.
*	The yaw input defines the rate of rotation on Z and the roll and pitch define the desired inclination angle on the 
*	X and Y axis.
*/

int control_quat(struct state *curr, struct state *past, struct controller *control){
	struct quaternion yawquat, tmpquat;

	if(control->controlflag){						//if new control signals have been received
	
		float pitch = (control->pitch - 1500) * (PI / (500.0 * -6.0));
		float roll = (control->roll - 1500) * (PI / (500.0 * 6.0));
		float yaw = (control->yaw - 1500) * (PI / (500.0 * -12.0));
		if(control->throttle > 1100){
			curr->accum_yaw = yaw * curr->dt + past->accum_yaw;	//only accumulate yaw if in flight to avoid integrator windup
		}else{
			curr->accum_yaw = past->accum_yaw;
		}
		curr->yaw = yaw;

		
		

		if(curr->accum_yaw > 6.3) curr->accum_yaw -= 2*PI;		//put bounds on the size of the angle
		else if(curr->accum_yaw < -6.3) curr->accum_yaw += 2*PI;


		//Simultaneous Orthagonal Rotations Angle (description under angle axis wiki page)
		float angle = curr->accum_yaw/2.0;

		yawquat.a = cos(angle);
		yawquat.b = 0;
		yawquat.c = 0;
		yawquat.d = sin(angle);

		if(pitch == 0 && roll == 0) pitch = .00001;	 		//eliminate div by 0;

		angle = sqrt(pitch*pitch + roll*roll);
		pitch /= angle;
		roll /= angle;

		float sinangle = sin(angle/2.0);
		tmpquat.a = cos(angle/2.0);
		tmpquat.b = pitch * sinangle;
		tmpquat.c = roll * sinangle;
		tmpquat.d = 0;
		
		mult_quaternion(&yawquat, &tmpquat, curr->gdquat);			//rotations in local reference frame

		
	}else{
		
		if(control->throttle > 1050){	//if there are no new control signals use the ones from the last time
			curr->accum_yaw = past->accum_yaw + past->yaw * curr->dt;	//check throttle to avoid integrator wind up
		}else{
			curr->accum_yaw = past->accum_yaw;
		}
		curr->yaw = past->yaw;

		clone_quaternion(past->gdquat, curr->gdquat);

	}

	return 0;
}

/*
*	Given a quaternion representing a desired rotation the function calculates the angles each of the axis
*	must simultaneously rotate through in order to achieve the desired rotation.
*
*/

int qmode(struct state *curr){
	struct quaternion q;

	mult_quaternion(curr->bgquat, curr->gdquat, &q);		//rotations in local frame of reference

	
	if(q.a < 0){	//inv_sora doesn't handle -a very well because of arccos' range 
		q.a *= -1;	
		q.b *= -1;	
		q.c *= -1;	
		q.d *= -1;	
	}
	
	inv_sora(&q, curr->correct);					//calculate corrections for each axis and store inn curr->correct

	scale_vector(180/3.1415926535, curr->correct);			//radians to degrees

	return 0;
}



/*
*	Applies the error correction to the motors by using a PID controller.
*
*/

int apply_error(struct state *curr, struct state *past, struct motorval *motor, struct ipid *pid, struct controller *control){
	//all weights will be div by 100 ... for integer arithmetic
	pid->dx = (control->aux1 + 1) * 20;
	pid->dy = (control->aux1 + 1) * 20;
	pid->px = (control->aux2 + 1) * 50;   
	pid->py = (control->aux2 + 1) * 50;

	pid->dz = 260;
	pid->pz = 380;
	pid->ix = 10;
	pid->iy = 10;
	pid->iz = 10;

	float throttle = control->throttle - 1000;	


	struct ivector gyrovect;
	
	gyrovect.x = pid->dx * (int)(curr->val[GYR][X] * (180 / 3.1415926535897)) / 100;
	gyrovect.y = pid->dy * (int)(curr->val[GYR][Y] * (180 / 3.1415926535897)) / 100;
	gyrovect.z = pid->dz * (int)(curr->val[GYR][Z] * (180 / 3.1415926535897)) / 100;


	

	if(throttle > 100){ 	//stop integrator from winding up
	
		curr->intcorrect->x = past->intcorrect->x + (curr->correct->x * curr->dt);
		curr->intcorrect->y = past->intcorrect->y + (curr->correct->y * curr->dt);
		curr->intcorrect->z = past->intcorrect->z + (curr->correct->z * curr->dt);
	}else{
		curr->intcorrect->x = past->intcorrect->x;
		curr->intcorrect->y = past->intcorrect->y;
		curr->intcorrect->z = past->intcorrect->z;
	}
	
	
	curr->correct->x = curr->correct->x * pid->px / 100.0;
	curr->correct->y = curr->correct->y * pid->py / 100.0;
	curr->correct->z = curr->correct->z * pid->pz / 100.0;


	//motors labelled counter-clockwise (from top)
	//		A
	//	      B   D
	//		C

	//apply corrections using a PID controller
	
	//x and y axis derivative correction(taken directly from gyro instead of calculated to reduce noise)
	motor->a -= gyrovect.x; 		//good
	motor->b -= gyrovect.y;
	motor->c += gyrovect.x;
	motor->d += gyrovect.y;
	
	//z axis derivative correction (taken directly from gyro instead of calculated to reduce noise)
	motor->a -= gyrovect.z;  		//good
	motor->b += gyrovect.z;
	motor->c -= gyrovect.z;
	motor->d += gyrovect.z;

	//x and y axis proportional correction
	motor->a += (int)curr->correct->x;  	//good
	motor->b += (int)curr->correct->y;
	motor->c -= (int)curr->correct->x;
	motor->d -= (int)curr->correct->y;	


	//z axis proportional correction
	motor->a += curr->correct->z; 		//good
	motor->b -= curr->correct->z;
	motor->c += curr->correct->z;
	motor->d -= curr->correct->z;



	//x and y axis integral correction ... currently not in use
/*	motor->a += curr->intcorrect->x * pid->iy / 100;  
	motor->b += curr->intcorrect->y * pid->ix / 100;
	motor->c -= curr->intcorrect->x * pid->iy / 100;
	motor->d -= curr->intcorrect->y * pid->ix / 100; */


	//add throttle
	motor->a += throttle;
	motor->b += throttle;
	motor->c += throttle;
	motor->d += throttle;	

	return 0;
} 



