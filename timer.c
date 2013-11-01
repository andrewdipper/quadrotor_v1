#include "timer.h"

//variables used by timer interrupts for timing the RC recevier's pulses
static int rxinstart[6];
static int rxinstop[6];
static int rxinval[6];
static int newflag = 0;


//initializes timers
//TIM 1,2,3 are for reading receiver's control signals (need offload this work or find a ppm signal to fix this)
void timer_init(TIM_TypeDef *TIM){
	if(TIM == TIM0){
		SC->PCONP |= (1 << 1);		//Timer 0 powered by default
		SC->PCLKSEL0 |= (1 << 2);  //PCLK = CCLk
		TIM0->PR = 0;
		TIM1->TCR = 0x03;
		TIM1->TCR = 0x01;
	}else if(TIM == TIM1){
		PINCON->PINMODE3 |= (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);
		SC->PCONP |= (1 << 2);  	//Power timer 1
		SC->PCLKSEL0 |= (1 << 4);
		PINCON->PINSEL3 |= (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7); //(0x0f << 4)
		TIM1->CCR |= (0x3f);
		TIM1->PR = 0;	
		TIM1->TCR = 0x03;
		TIM1->TCR = 0x01;					//enable timer	
	}else if(TIM == TIM2){
		SC->PCONP |= (1 << 22);
		SC->PCLKSEL1 |= (1 << 12);
		PINCON->PINSEL0 |= (1 << 8) | (1 << 9) | (1 << 10) | (1 << 11); //(0x0f << 8)
		TIM2->CCR |= (0x3f);
		TIM2->PR = 0;						//no prescaler
		TIM2->TCR = 0x03;
		TIM2->TCR = 0x01;					//enable timer	
	}else if(TIM == TIM3){
		SC->PCONP |= (1 << 23);
		SC->PCLKSEL1 |= (1 << 14);
		PINCON->PINSEL1 |= (1 << 14) | (1 << 15) | (1 << 16) | (1 << 17);
		TIM3->CCR |= (0x3f);
		TIM3->PR = 0;						//no prescaler
		TIM3->TCR = 0x03;
		TIM3->TCR = 0x01;						//enable timer	

	}	
}

//initialize soft watchdog that ensures that the quadrotor is still receiving control signals
void watchdog_init(unsigned int time){
	SC->PCLKSEL0 |= (1 << 0);
	WDT->WDTC = time;
	WDT->WDCLKSEL = (1 << 0);	//use pclk
	WDT->WDMOD = (1 << 0);
	WDT->WDFEED = 0xAA;
	WDT->WDFEED = 0x55;
}

//reset the watchdog 
void watchdog_reset(void){
	WDT->WDFEED = 0xAA;
	WDT->WDFEED = 0x55;
}

//check to see if watchdog was set off (means no control signals are being received)
inline int watchdog_read(void){
	return ((WDT->WDMOD & (1 << 2)) == 0) ? 0 : 1; 
}


inline void reset_timer(TIM_TypeDef *TIM){
	TIM->TCR = 0x03;			
	TIM->TCR = 0x01;
}

//returns flag signalling if new control data was received
inline int get_flag(void){  
	return newflag;
}

//clears flag signalling that new control data was received
inline void clear_flag(void){ 
	newflag = 0;
}

//retrieve pulse length for channel "i"
int get_pulse(int i){
	return rxinval[i];
}



//interrupt handlers for measuring pulse lengths from the receiver
//


void TIMER1_IRQHandler(void){
	int val = (TIM1->IR & (0x03 << 4));
	if(val & (1 << 4)){
		if(GPIO1->FIOPIN &  (1 << 18)){rxinstart[0] = TIM1->CR0;   }
		else {rxinstop[0] = TIM1->CR0;  rxinval[0] = rxinstop[0] - rxinstart[0];  watchdog_reset();} 
		TIM1->IR |= (1 << 4);
	}
	if(val & (1 << 5)){
		if(GPIO1->FIOPIN &  (1 << 19))rxinstart[1] = TIM1->CR1;  
		else{ rxinstop[1] = TIM1->CR1; rxinval[1] = rxinstop[1] - rxinstart[1];} 
		TIM1->IR |= (1 << 5);
	}
}

void TIMER2_IRQHandler(void){
	int val = (TIM2->IR & (0x03 << 4));
	if(val & (1 << 4)){
		if(GPIO0->FIOPIN &  (1 << 4))rxinstart[2] = TIM2->CR0;   
		else {rxinstop[2] = TIM2->CR0;  rxinval[2] = rxinstop[2] - rxinstart[2];}
		GPIO1->FIOPIN ^= 1 << 29;	
		TIM2->IR |= (1 << 4);
	}
	if(val & (1 << 5)){
		if(GPIO0->FIOPIN &  (1 << 5))rxinstart[3] = TIM2->CR1;  
		else {rxinstop[3] = TIM2->CR1; rxinval[3] = rxinstop[3] - rxinstart[3];} 
		TIM2->IR |= (1 << 5);
	}
}

void TIMER3_IRQHandler(void){
	int val = (TIM3->IR & (0x03 << 4));
	if(val & (1 << 4)){
		if(GPIO0->FIOPIN &  (1 << 23))rxinstart[4] = TIM3->CR0;  
		else {rxinstop[4] = TIM3->CR0;  rxinval[4] = rxinstop[4] - rxinstart[4];}
		TIM3->IR |= (1 << 4);
	}
	if(val & (1 << 5)){
		if(GPIO0->FIOPIN &  (1 << 24))rxinstart[5] = TIM3->CR1;  
		else {rxinstop[5] = TIM3->CR1;  rxinval[5] = rxinstop[5] - rxinstart[5]; newflag = 1;}
		TIM3->IR |= (1 << 5);
	}
}




