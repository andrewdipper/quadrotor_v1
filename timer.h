#pragma once
#include "LPC17xx.h"

//initializes timers
//TIM 1,2,3 are for reading receiver's control signals
void timer_init(TIM_TypeDef *TIM);


inline void reset_timer(TIM_TypeDef *TIM);

//initialize soft watchdog that ensures that the quadrotor is still receiving control signals
void watchdog_init(unsigned int time);

//reset the watchdog 
void watchdog_reset(void);

//check to see if watchdog was set off (means no control signals are being received)
inline int watchdog_read(void);

//retrieve pulse length for channel "i"
int get_pulse(int i);

//clears flag signalling that new control data was received
inline void clear_flag(void);

//returns flag signalling if new control data was received
inline int get_flag(void);

//interrupt handlers for measuring pulse lengths from the receiver
void TIMER1_IRQHandler(void);
void TIMER2_IRQHandler(void);
void TIMER3_IRQHandler(void);


