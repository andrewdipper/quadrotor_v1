/*
Copyright (C) 2013  Andrew Pratt
View the README
*/

#include "uandi.h"


//The quadrotor should not be moved when this function is executed

//initializes current and past state structures along with other variables
//initializes hardware and sensors
//calculates attitude using accelerometers only
//reads magnetometer values to calculate gmagvect
//recalculates state multiple times to ensure the attitude estimation is correct before flight
void initialization(struct state *curr, struct state *past, float *gyrobias, struct vector *magvect){
	int i;

	curr->yaw = 0;
	past->yaw = 0;
	curr->accum_yaw = 0;
	past->accum_yaw = 0;

	curr->corrweight->acccorr = .8;
	curr->corrweight->magcorr = .8;

	past->corrweight->acccorr = .8;
	past->corrweight->magcorr = .8;


	curr->correct->x = 0;
	curr->correct->y = 0;
	curr->correct->z = 0;

	past->correct->x = 0;
	past->correct->y = 0;
	past->correct->z = 0;

	curr->intcorrect->x = 0;
	curr->intcorrect->y = 0;
	curr->intcorrect->z = 0;

	past->intcorrect->x = 0;
	past->intcorrect->y = 0;
	past->intcorrect->z = 0;

	curr->gdquat->a = 1;
	curr->gdquat->b = 0;
	curr->gdquat->c = 0;
	curr->gdquat->d = 0;

	past->gdquat->a = 1;
	past->gdquat->b = 0;
	past->gdquat->c = 0;
	past->gdquat->d = 0;
	


	curr->gbquat->a = 1;
	curr->gbquat->b = 0;
	curr->gbquat->c = 0;
	curr->gbquat->d = 0;

	past->gbquat->a = 1;
	past->gbquat->b = 0;
	past->gbquat->c = 0;
	past->gbquat->d = 0;
	
	i2c_init(I2C1);
	accel_init();
	gyro_init();
	mag_init();
	timer_init(TIM0);
	
	calc_gbias(gyrobias);

	reset_timer(TIM0);
	accel_read(past->read[ACC]);
	gyro_read(past->read[GYR]);
	mag_read(past->read[MAG]);
	
	read_to_val(past->read[ACC], past->val[ACC]);
	read_to_val(past->read[MAG], past->val[MAG]);	

	normalize_vector((struct vector *)past->val[ACC]);	//normalize to 1
	for(i = 0; i < 3; ++i) past->val[GYR][i] = (float)past->read[GYR][i] * (3.14159265358979 / (14.375 * 180.0)) ; //scale to deg/sec
	normalize_vector((struct vector *)past->val[MAG]);	//normalize to 1

	struct state *exchange;

	
	conjugate_quaternion(past->gbquat, past->bgquat);

	for(i = 0; i < 500; ++i){
		update(curr, past, gyrobias);
		
		clear_correction(curr);
		correct_acc(curr, past);
		gyroquat((struct vector *)curr->val[GYR], curr->dt, curr->uquat, curr->corrv);
		mult_quaternion(past->gbquat, curr->uquat, curr->gbquat);
		normalize_quaternion(curr->gbquat);
		conjugate_quaternion(curr->gbquat, curr->bgquat);
		
		exchange = curr;
		curr = past;
		past = exchange;

		

	}

	vmult_quaternion(past->gbquat, (struct vector *)past->val[MAG], magvect);  //changed to proper rotation but should adversely affect the magcorrect

	normalize_vector(magvect);
	for(i = 0; i < 500; ++i){
		update(curr, past, gyrobias);
		
		clear_correction(curr);
		correct_acc(curr, past);
		correct_mag(curr, past, magvect);
		gyroquat((struct vector *)curr->val[GYR], curr->dt, curr->uquat, curr->corrv);
		mult_quaternion(past->gbquat, curr->uquat, curr->gbquat);
		normalize_quaternion(curr->gbquat);
		conjugate_quaternion(curr->gbquat, curr->bgquat);
		
		exchange = curr;
		curr = past;
		past = exchange;

	}

	curr->corrweight->acccorr = 0.75;
	curr->corrweight->magcorr = 0.75;

	past->corrweight->acccorr = 0.75;
	past->corrweight->magcorr = 0.75;

}


//updates the quadrotor's state by reading sensor values and calculating the delta time for use in future calculations
//runs all the sensor values through a lowpass filter to remove noise
void update(struct state *curr, struct state *past, float *gyrobias){ 
	int i, j;
	accel_read(curr->read[ACC]);
	gyro_read(curr->read[GYR]);
	mag_read(curr->read[MAG]);

	curr->dt = TIM0->TC / 100000000.0;  //CCLK frequency
	reset_timer(TIM0);

	read_to_val(curr->read[ACC], curr->val[ACC]);
	read_to_val(curr->read[MAG], curr->val[MAG]);	



	normalize_vector((struct vector *)curr->val[ACC]);	//normalize to 1
	for(i = 0; i < 3; ++i) curr->val[GYR][i] = ((float)curr->read[GYR][i] - gyrobias[i]) * (3.14159265358979 / (14.375 * 180.0)) ; //scale to deg/sec
	normalize_vector((struct vector *)curr->val[MAG]);	//normalize to 1
	
	for(i = 0; i < 3; ++i){
		for(j = 0; j < 3; ++j){
		 	curr->val[i][j] = lowpass(past->val[i][j], curr->val[i][j], curr->dt, 0.01);  //old value .005 //make up for small size of quad
		}
	}




}


//turns all motors off and increases ESC update period to 20 ms
//prints out watchdog to show that the quadrotor is shutting down due to a lack of control signals
void run_watchdog(void){
	int j;
	for(j = 0; j < 1000; ++j){
		PWM1->MR0 = 20000;
		PWM1->MR3 = 1000;
		PWM1->MR4 = 1000;			
		PWM1->MR5 = 1000;			
		PWM1->MR6 = 1000;
	
		PWM1->LER |= (1 << 0) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6);  

		out(UART0, "#watchdog\n");
	}
}

//send the motor values to the pwm hardware to send updated signals to the ESCs to update the motor speed
int set_motor(struct motorval *motor){

	if(motor->a < 1000) motor->a = 1000;
	if(motor->b < 1000) motor->b = 1000;
	if(motor->c < 1000) motor->c = 1000;
	if(motor->d < 1000) motor->d = 1000;
	if(motor->a > 1800) motor->a = 1800;
	if(motor->b > 1800) motor->b = 1800;
	if(motor->c > 1800) motor->c = 1800;
	if(motor->d > 1800) motor->d = 1800;

	PWM1->MR3 = motor->a;
	PWM1->MR4 = motor->b;			
	PWM1->MR5 = motor->c;			
	PWM1->MR6 = motor->d;
	PWM1->LER |= (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6);  

	return 0;
}

//reduce the motor speed to zero
//1 ms pulse to ESC means no power to the motor
int clear_motor(struct motorval *motor){
	motor->a = 1000;
	motor->b = 1000;
	motor->c = 1000;
	motor->d = 1000;
	
	return 0;

}


//read the pulses from the RC receiver and convert them into values stored in a controller structure
int pulse_read(struct controller *control){
	
	if(!get_flag()){ control->controlflag = 0; return 1;}
	clear_flag();
	int i;
	int trxinval[6];
	for(i = 0; i < 6; ++i){
		trxinval[i] = (get_pulse(i) + 50)/ 100;
		if(trxinval[i] > 2000 && trxinval[i] < 2200) trxinval[i] = 2000;
		else if(trxinval[i] < 1000 && trxinval[i] > 800) trxinval[i] = 1000;
		else if(trxinval[i] > 2200 || trxinval[i] < 800) {
			trxinval[i] = 1500; 
			if(i == 2) trxinval[2] = 1000;  
			else if(i == 4) trxinval[4] = 1000; 
			else if(i == 5) trxinval[5] = 1000; 
		}
	}

	

	

	control->roll = trxinval[0];
	control->pitch = trxinval[1];
	control->throttle = trxinval[2];
	control->yaw = trxinval[3];
	control->aux1 = (trxinval[4] - 1000)/100.0;  	//from 0-10
	control->aux2 = (trxinval[5] - 1000)/100.0;	//from 0-10
	
	control->controlflag = 1;

	return 0;
}


//for debugging
void print_motor(struct motorval *motor){

	out(UART0, "#");

	outi(UART0, motor->a); out(UART0, "   ");
	outi(UART0, motor->b); out(UART0, "   ");
	outi(UART0, motor->c); out(UART0, "   ");
	outi(UART0, motor->d); out(UART0, "   ");

	out(UART0, "\n");
}

//for debugging
void print_control(struct controller *control, struct pid *pid){
	outi(UART0, (int)(control->roll)); out(UART0, "   ");
	outi(UART0, (int)(control->pitch)); out(UART0, "   ");
	outi(UART0, (int)(control->throttle)); out(UART0, "   ");
	outi(UART0, (int)(control->yaw)); out(UART0, "   ");
	outi(UART0, (int)(pid->dx * 1000)); out(UART0, "   ");
	outi(UART0, (int)(pid->dz * 1000)); out(UART0, "   ");


	out(UART0, "\n");

}


