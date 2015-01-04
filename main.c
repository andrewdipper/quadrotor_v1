/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#include "hw/hw.h"
#include "quadcore/quadcore.h"

//minimum read time for the three sensors is ~780 us

int main(void){
	
	//initialize busses necessary for operation
	init_busses();

	out0("STARTING");

	delay_us(6000000);

	struct quad_state a = {{{0,0,0}, {0,0,0}, {0,0,0}}, 
				{1, 0, 0, 0}, {1, 0, 0, 0}, 
				{1, 0, 0, 0}, {1, 0, 0, 0}, 
				{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 1};

	struct quad_state b = {{{0,0,0}, {0,0,0}, {0,0,0}},
				{1, 0, 0, 0}, {1, 0, 0, 0}, 
				{1, 0, 0, 0}, {1, 0, 0, 0}, 
				{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 1};

	struct quad_static qstatic = {{0,1,0}, 3.3, 3.3, {0,0,0,0}, 
				      {0,0,0,0,0,0,0,0,0}, 1, 0, {0,0,0,0,0,0,0,0,0}, 0}; //gains set in the init orientation function
	
	
	//strucures for maintaining quadrotor state
	struct quad_state *curr, *past, *swap;
	curr = &a;
	past = &b;

	
	//initialize timers and sensors
	ftm0_init();
	sensor_init();

	//calculate attitude of quad...MUST BE STATIC FOR THIS OPERATION
	init_orientation(curr, past, &qstatic);
	

	//make sure motors are  OFF
	clear_motor(&qstatic);
	set_motor(&qstatic);


	delay_us(100000);

	//submit sensor read request
	sensor_queue();	

	//increase update rate for tighter control
	increase_motor_refresh();

	//set timer to compute time between calculations
	pit0_set(48000000);

	//initialize safety timers
	init_wdogs();


	while(1){
		
		//read control input if available
		read_control(&qstatic);
		
		//wait for the sensor values to be updated
		while(sensor_status() != 0 );		

		//reset the sensor watchdog
		pit1_set_int(48000);			

		//Handler for bad sensor data
		if(sensor_parse(curr->sdata) != 0){		
			clone_vector((struct vector *)&past->sdata[0], (struct vector *)&curr->sdata[0]);
			clone_vector((struct vector *)&past->sdata[1], (struct vector *)&curr->sdata[1]);
			clone_vector((struct vector *)&past->sdata[2], (struct vector *)&curr->sdata[2]);
			out0("ERROR: FAILED SENSOR READ");		
		}
		
		//submit a sensor read request for next iteration
		sensor_queue();

		//get passed time from last iteration
		curr->dt = get_timeint();

		//filter sensor data using lowpass to smooth noise
		filter_data(curr, past);
				
		
		//get attitude corrections from accelerometer and magnetometer
		clear_correction(curr);
		accel_correction(curr, past, &qstatic);
		mag_correction(curr, past, &qstatic);

		//translate gyroscope output to quaternion representing rotation over curr->dt time
		gyroquat2((struct vector *)curr->sdata[1], curr->dt, &curr->uquat, &curr->correct_vect);

		//apply update rotation to gb rotation and compute bg rotation
		mult_quaternion(&past->gbquat, &curr->uquat, &curr->gbquat);
		normalize_quaternion(&curr->gbquat);
		conjugate_quaternion(&curr->gbquat, &curr->bgquat);
	
		//get quaterion representing target position from control inputs
		control_quat(curr, past, &qstatic);
		
		//compute attitude correction
		qmode(curr);
		
		//update desired motor values from attitude correction
		apply_correction(curr, past, &qstatic);


		//Safety Feature: Motors don't spin up unless throttle above threshold
		if(qstatic.throttle < 150){			
			clear_motor(&qstatic);
		}

		set_motor(&qstatic);

		//DEBUG
		//print_quaternion(&curr->uquat);
		//print_quaternion(&curr->gbquat);
		//out0("   ");
		//print_motor(&qstatic);
		//print_vect((struct vector*) &curr->sdata[0]);
		//print_vect(&curr->att_correct);
		//out0("\n");
		
		
		//Swap past and current structures to setup for next iteration
		swap = curr;
		curr = past;
		past = swap;
	
	}
	

	
	
	

	
/////////////////////////////////////

	return 0;
}

