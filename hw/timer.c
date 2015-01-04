/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#include "timer.h"

#include "../quadcore/imu.h"
#include "../quadcore/aux.h"

//ALL PITS USE THE BUS CLOCK (48Mhz typ)
inline void pit_module_enable(void){
	SIM_SCGC6 |= SIM_SCGC6_PIT;	//provide the PIT with clock (Not doing this causes instant reset)
	PIT_MCR = 0;			//make sure the clocks are not disabled	
}



//PIT0________________________________________

inline void pit0_set_int(uint32_t timerval){		//disables and reenables to so LDVAL is updated
	PIT_TCTRL0 = 0x00;
	PIT_LDVAL0 = timerval;
	PIT_TFLG0 = 0x01;				//clear interrupts
	PIT_TCTRL0 = (1 << 1) | (1 << 0);		//enable timer and interrupts
}

inline void pit0_set(uint32_t timerval){		//disables and reenables to so LDVAL is updated
	PIT_TCTRL0 = 0x00;
	PIT_LDVAL0 = timerval;
	PIT_TCTRL0 = (1 << 0);				//enable timer ... will not interrupt
}

inline int pit0_read(void){
	return PIT_CVAL0;
}
	

void pit0_isr(void){
	PIT_TFLG0 = 0x01;				//clear the interrupt flag
	uart0_tx('i');
}







//PIT1________________________________________


inline void pit1_set_int(uint32_t timerval){		//disables and reenables to so LDVAL is updated
	PIT_TCTRL1 = 0x00;
	PIT_LDVAL1 = timerval;
	PIT_TFLG1 = 0x01;				//clear interrupts
	PIT_TCTRL1 = (1 << 1) | (1 << 0);		//enable timer and interrupts
}

inline void pit1_set(uint32_t timerval){		//disables and reenables to so LDVAL is updated
	PIT_TCTRL1 = 0x00;
	PIT_LDVAL1 = timerval;
	PIT_TCTRL1 = (1 << 0);				//enable timer ... will not interrupt
}

inline int pit1_read(void){
	return PIT_CVAL1;
}
	

void pit1_isr(void){
	print_sensor_status();
	i2ca0_purge();
	i2ca0_recover();
	sensor_queue();
	out0("SENSOR ERROR -------- BASICALLY A CRASH");
	PIT_TFLG1 = 0x01;				//clear the interrupt flag
	pit1_set_int(48000);											//reset the sensor watchdog

}



//PIT2________________________________________



inline void pit2_set_int(uint32_t timerval){		//disables and reenables to so LDVAL is updated
	PIT_TCTRL2 = 0x00;
	PIT_LDVAL2 = timerval;
	PIT_TFLG2 = 0x01;				//clear interrupts
	PIT_TCTRL2 = (1 << 1) | (1 << 0);		//enable timer and interrupts
}

inline void pit2_set(uint32_t timerval){		//disables and reenables to so LDVAL is updated
	PIT_TCTRL2 = 0x00;
	PIT_LDVAL2 = timerval;
	PIT_TCTRL2 = (1 << 0);				//enable timer ... will not interrupt
}

inline int pit2_read(void){
	return PIT_CVAL2;
}
	

void pit2_isr(void){
	out0("LOST RADIO");
	kill();
	PIT_TFLG2 = 0x01;				//clear the interrupt flag
}



