#include "LPC17xx.h"
#include "structures.h"
#include "i2c.h"
#include "uart.h"
#include "pwm.h"
#include "imu.h"
#include "timer.h"
#include "filters.h"
#include "uandi.h"
#include "modes.h"
#include "quat.h"
#include "correct.h"
#include "math.h"


int main (void){ 

	SystemInit();

	//initialize important hardware devices
	i2c_init(I2C1);
	uart_init(UART0, 57600);
	
	out(UART0, "\n\nHELLO3\n");
	
	

////////////////////////// Declare and link structures	
	struct vector v1, v2, v3, v4, v5, v6;
	struct quaternion q1, q2, q3, q4, q5, q6, q7, q8;	

	float gyrobias[3];
	struct state a1, a2;
	struct state *curr = &a1;
	struct state *past = &a2;
	struct state *exchange;

	curr->gbquat = &q1;
	curr->uquat = &q2;
	curr->corrv = &v1;
	past->gbquat = &q3;
	past->uquat = &q4;
	past->corrv = &v2;

	curr->bgquat = &q8;
	past->bgquat = &q7;

	curr->gdquat = &q5;
	past->gdquat = &q6;

	curr->correct = &v3;
	past->correct = &v4;

	curr->intcorrect = &v5;
	past->intcorrect = &v6;


	struct correct_weight cw1, cw2;
	curr->corrweight = &cw1;
	past->corrweight = &cw2;


	struct vector magvect;
	


	pwm_init();						//initialize pwm so motors ESCs have time to go through startup procedure

	initialization(curr, past, gyrobias, &magvect);		//run initialization routine....calculates initial state

/////////////////////////////////////

		

	//3 timers used to read pulses from receiver
	timer_init(TIM1);
	timer_init(TIM2);
	timer_init(TIM3);
	//enable interrupts for all three timers
	NVIC_EnableIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(TIMER2_IRQn);
	NVIC_EnableIRQ(TIMER3_IRQn);


	
	struct ipid ipid;
	struct motorval motor;

	struct controller control;
	control.throttle = 0;
	control.pitch = 0;
	control.roll = 0;
	control.yaw = 0;
	control.aux1 = 0;
	control.aux2 = 0;
	control.controlflag = 1;


	PWM1->MR0 = 4000;		//set esc update rate to 250hz for less latency
	PWM1->LER |= (1 << 0);

	watchdog_init(10000000);	//configure soft watchdog to enable after no control input for 1 second



//	int loops = 0;
//	int timer = 0;
	int correctcnt = 0;


	
while(1){
	
	if(watchdog_read()) break;  		//check if watchdog is enabled // shutdown if true
	
	
	
	update(curr, past, gyrobias);		//read sensors and send them through a lowpass filter

	//used to time number of loop execution per second : typically ~1000
	//timer += curr->dt * 100000000;
	//++loops;
	//if(timer > 100000000){ outi(UART0, loops); out(UART0, "\n"); loops = 0; timer = 0;}

	++correctcnt;
	if(correctcnt > 10){	//correct every tenth loop to reduce floating point operations
		correctcnt = 0;
		clear_correction(curr);
		correct_acc(curr, past);
		correct_mag(curr, past, &magvect);
	}else{
		clone_vector(past->corrv, curr->corrv);
	}

	gyroquat2((struct vector *)curr->val[GYR], curr->dt, curr->uquat, curr->corrv);		//estimate change in attitude based on the gyros
	mult_quaternion(past->gbquat, curr->uquat, curr->gbquat);				//update gbquat with the update quaternion

	if(correctcnt > 9){normalize_quaternion(curr->gbquat);}					//make sure the quaternion is a unit quaternion

	conjugate_quaternion(curr->gbquat, curr->bgquat);


	pulse_read(&control);						//read the pulses from the receiver

	
	
	
	control_quat(curr, past, &control);				//calculate the desired attitued
	qmode(curr);							//calculate correction on X,Y,Z axis
	clear_motor(&motor);						
	apply_error(curr, past, &motor, &ipid, &control);		//calculate speed for each motor based on necessary correction

	if(control.throttle < 1050) clear_motor(&motor); 		//kill motors if throttle below 5% (safety)

	set_motor(&motor);						//send motor values to the pwm hardware


	//swap current and past state structures			
	exchange = curr;
	curr = past;
	past = exchange;

	}


	run_watchdog();						//watchdog has been enabled...shutdown by turning off the motors

	return 0;  
}




	
