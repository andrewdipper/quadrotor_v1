/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#include "i2c.h"

//master only so address is unnecessary
//I2C baud rate = bus speed (Hz)/(mul × SCL divider)
//Clock source is F_BUS


//TODO make this asynchronous (this bus is too slow to wait for);

void i2c0_init(void){
	SIM_SCGC4 |= SIM_SCGC4_I2C0;

	
	I2C0_A1 = 0x16;
	I2C0_RA = 0;

	CORE_PIN18_CONFIG =  PORT_PCR_ODE | PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(2);	//configure scl TEMP enable pullups
	CORE_PIN19_CONFIG =  PORT_PCR_ODE | PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(2);	//configure sda	TEMP enable pullups


	I2C0_F = 0x85;		//how???			//400  khz somehow

	I2C0_C2 = (1 << 5);					//enable high drive

	I2C0_C1 = (1 << 7);					//12c enable / master mode/ start in transmit mode

}

void i2c0_start(void){
	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	I2C0_C1 = (1 << 7) | (1 << 5) | (1 << 4);		//send start signal (enable module)
	delay_us(10);						//make sure written before returning
	while(!(I2C0_S & (1 << 5))); 				//wait until transfer has been completed
}

void i2c0_restart(void){
	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	I2C0_C1 = (1 << 7) | (1 << 5) | (1 << 4) | (1 << 2);	//send start signal (enable module)
	delay_us(4);						//make sure written before returning
	while(!(I2C0_S & (1 << 5))); 				//wait until transfer has been completed
}

void i2c0_stop(void){
	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	I2C0_C1 = (1 << 7);					//send start signal (enable module)
	delay_us(10);						//make sure written before returning
	while((I2C0_S & (1 << 5))); 			//wait until transfer has been completed
}


int i2c0_tx(char c){
	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	I2C0_C1 |= (1 << 4);					//put module in transmit mode
	I2C0_D = c;						//write to data register
	//delay_us(10);						//make sure written before returning
	while((I2C0_S & (1 << 1)) == 0); 			//wait until transfer has been completed
	return I2C0_S;
}

void i2c0_nack_next(void){
	I2C0_C1 |= (1 << 3);
}

int i2c0_rx(char *data){	
	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	I2C0_C1 &= ~(1 << 4);					//put module in receive mode
	*data = I2C0_D;						//write to data register
								//do i need a dummy read
	delay_us(10);						//make sure written before returning
	while((I2C0_S & (1 << 7)) == 0); 				//wait until transfer has been completed
	return I2C0_S;
}

int i2c0_set(char addr, char reg, char data){
	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	I2C0_C1 |= (1 << 4);					//put module in transmit mode
	I2C0_D = addr;						//write to data register
	delay_us(10);						//make sure written before returning
	while((I2C0_S & (1 << 7)) == 0); 			//wait until transfer has been completed

	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	I2C0_D = reg;						//write to data register
	delay_us(10);						//make sure written before returning
	while((I2C0_S & (1 << 7)) == 0); 			//wait until transfer has been completed

	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	I2C0_D = data;						//write to data register
	delay_us(10);						//make sure written before returning
	while((I2C0_S & (1 << 7)) == 0); 			//wait until transfer has been completed
	return I2C0_S;

}


void i2c0_writebyte(char addr, char data){
	
//send a start
	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	I2C0_C1 = (1 << 7) | (1 << 5) | (1 << 4);		//send start signal (enable module)
	while(!(I2C0_S & (1 << 5))); 				//wait until transfer has been completed

//send addr
	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	I2C0_C1 |= (1 << 4);					//put module in transmit mode
	I2C0_D = addr;						//write to data register
	while((I2C0_S & (1 << 1)) == 0); 			//wait until transfer has been completed


	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	I2C0_D = data;						//write to data register
	while((I2C0_S & (1 << 1)) == 0); 			//wait until transfer has been completed

	

//stop bus
	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	I2C0_C1 = (1 << 7);					//send start signal (enable module)
	while((I2C0_S & (1 << 5))); 			//wait until transfer has been completed


}


void i2c0_write(char addr, char *data, int len){
	
//send a start
	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	I2C0_C1 = (1 << 7) | (1 << 5) | (1 << 4);		//send start signal (enable module)
	while(!(I2C0_S & (1 << 5))); 				//wait until transfer has been completed

//send addr
	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	I2C0_C1 |= (1 << 4);					//put module in transmit mode
	I2C0_D = addr;						//write to data register
	while((I2C0_S & (1 << 1)) == 0); 			//wait until transfer has been completed

	int i;
	for(i = 0; i < len; ++i){
		I2C0_S = (1 << 4) | (1 << 1);				//clear flags
		I2C0_D = data[i];					//write to data register
		while((I2C0_S & (1 << 1)) == 0); 			//wait until transfer has been completed
	}
	

//stop bus
	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	I2C0_C1 = (1 << 7);					//send start signal (enable module)
	while((I2C0_S & (1 << 5))); 			//wait until transfer has been completed


}

void i2c0_read(char addr, char *data, int len){
	
//send a start
	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	I2C0_C1 = (1 << 7) | (1 << 5) | (1 << 4);		//send start signal (enable module)
	while(!(I2C0_S & (1 << 5))); 				//wait until transfer has been completed

//send addr
	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	I2C0_C1 |= (1 << 4);					//put module in transmit mode
	I2C0_D = addr;						//write to data register
	while((I2C0_S & (1 << 1)) == 0); 			//wait until transfer has been completed

//read data
	I2C0_C1 &= ~(1 << 4);					//put module in receive mode
	
	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	data[0] = I2C0_D;					//dummy read will return address
	while((I2C0_S & (1 << 1)) == 0); 				//wait until transfer has been completed

	int i;
	for(i = 0; i < len-1; ++i){
		I2C0_S = (1 << 4) | (1 << 1);				//clear flags
		data[i] = I2C0_D;						//write to data register
		while((I2C0_S & (1 << 1)) == 0); 				//wait until transfer has been completed

	}

	I2C0_C1 |= (1 << 3);

	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	data[i] = I2C0_D;						//write to data register
	while((I2C0_S & (1 << 1)) == 0); 				//wait until transfer has been completed

//stop bus
	I2C0_S = (1 << 4) | (1 << 1);				//clear flags
	I2C0_C1 = (1 << 7);					//send start signal (enable module)
	while((I2C0_S & (1 << 5))); 			//wait until transfer has been completed



}



