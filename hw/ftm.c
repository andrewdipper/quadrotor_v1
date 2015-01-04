/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#include "ftm.h"

//I think this uses the bus clock F_BUS (48mhz)

// 20	D5	FTM0_CH5	
// 21	D6	FTM0_CH6
// 22	C1	FTM0_CH0	
// 23	C2	FTM0_CH1

//TODO adjust to get higher resolution
void ftm0_init(){
	SIM_SCGC6 |= SIM_SCGC6_FTM0;			//send ftm0 a clock

	FTM0_SC = (1 << 3) | (4 << 0);			//use the system clock F_BUS with a prescaler of 16
	FTM0_CNTIN = 0x0000;				//start counting at 0
	FTM0_MOD = 60000;				//20 ms period (servo standard) 50 hz (3000 per ms)

	CORE_PIN20_CONFIG = PORT_PCR_MUX(4);		//CH5
	CORE_PIN21_CONFIG = PORT_PCR_MUX(4);		//CH6
	CORE_PIN22_CONFIG = PORT_PCR_MUX(4);		//CH0
	CORE_PIN23_CONFIG = PORT_PCR_MUX(4);		//CH1

	FTM0_C5SC = 0x28;
	FTM0_C6SC = 0x28;
	FTM0_C0SC = 0x28;
	FTM0_C1SC = 0x28;

	FTM0_C5V = 3000;					//start all at a 1 ms pulse
	FTM0_C6V = 3000;
	FTM0_C0V = 3000;
	FTM0_C1V = 3000;
}

uint16_t ftm0_count(void){
	return FTM0_CNT;	
}  



