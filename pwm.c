/*
Copyright (C) 2013  Andrew Pratt
View the README
*/

#include "pwm.h"

//initializes pwm hardware
//used to control the ESCs
void pwm_init(void){
	SC->PCONP |= (1 << 6);  		// power up PWM (powered by default)
						//PCLK = CCLk/4 (25Mhz);

	PINCON->PINSEL4 |= (1 << 4) | (1 << 6) | (1 << 8) | (1 << 10);	//pin 1.18 for PWM1.1 and pin 1.20 for PWM1.2

	PINCON->PINMODE4 |= (1 << 5) | (1 << 7) | (1 << 9) | (1 << 11); //no pullup or pulldown on 1.18 or 1.20	


	PWM1->PR = 25;				//prescale counter = 25 = 1000000 counts per second for TC

	PWM1->MR0 = 20000;			//20 ms period
	
	PWM1->MCR |= (1 << 1);			//Reset timer on match 0

	PWM1->MR3 = 1000;
	PWM1->MR4 = 1000;			//2 ms pulse in 20 ms period for PWM 1.1
	PWM1->MR5 = 1000;			//1 ms pulse in 20 ms period for PWM 1.2
	PWM1->MR6 = 1000;

	PWM1->TCR |= (1 << 0) | (1 << 3);  	//enable timer/counter and pwm

	PWM1->PCR |= (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14);	// PWm output enable

	PWM1->LER |= (1 << 0) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6);  
}
