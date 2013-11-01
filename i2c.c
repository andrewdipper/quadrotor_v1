#include "i2c.h"
//
// functions for driving the I2C bus
//


void i2c_init(I2C_TypeDef *I2C){
	if(I2C == I2C0){
	}else if(I2C == I2C1) {
		SC->PCONP |= (1<<19);  		 		//power up i2c1 (default is powered)
		SC->PCLKSEL1 |= (1 << 6);			//use cclk for i2c1 clock
		PINCON->PINSEL0 |= 0x0F;			//selects pins for SDA1 and SCL1(0.0 and 0.1)
		PINCON->PINMODE0 |= (1 << 1) | (1 << 3);    	//no pull-up or pull-down of SDA1 and SCL1
		PINCON->PINMODE_OD0 |= (1 << 0) | (1 << 1);	//open drain for SDA1 and SCL1
		I2C->I2SCLH = 125;				//set 50% duty in standard mode
		I2C->I2SCLL = 125;				//(400khz)
		I2C->I2CONCLR = (1 << 3) | (1 << 5);		//CLear start and Si bits
		I2C->I2CONSET = (1 << 6);			//enable I2C
	}else if(I2C == I2C2){
	}
}


////////////////////////////////////////////////////////////////////////////////////////////////////////


inline int i2c_start(I2C_TypeDef *I2C){				//send start on the bus
	I2C->I2CONSET = (1 << 5);
	I2C->I2CONCLR = (1 << 3);
	while(!(I2C1->I2CONSET & (1 << 3)));
	I2C->I2CONCLR = (1 << 5);		
	return (I2C->I2STAT & 0xf8);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////





//////////////////////////////////////////////////////////////////////////////////////////////////////////


int i2c_tx(I2C_TypeDef *I2C, unsigned char data){
	I2C->I2DAT = data & 0xff;			//load data
	I2C->I2CONCLR = (1 << 3);			//clear SI
	while(!(I2C->I2CONSET & (1 << 3)));		//wait for SI to be set
	return (I2C->I2STAT & 0xf8);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////


void i2c_set(I2C_TypeDef *I2C, unsigned char addr, unsigned char reg, unsigned char data){
	I2C->I2DAT = addr & 0xff;			//address
	I2C->I2CONCLR = (1 << 3);			//clear SI
	while(!(I2C1->I2CONSET & (1 << 3)));		//wait for SI to be set

	I2C->I2DAT = reg & 0xff;			//register
	I2C->I2CONCLR = (1 << 3);			//clear SI
	while(!(I2C1->I2CONSET & (1 << 3)));		//wait for SI to be set

	I2C->I2DAT = data & 0xff;			//data
	I2C->I2CONCLR = (1 << 3);			//clear SI
	while(!(I2C1->I2CONSET & (1 << 3)));		//wait for SI to be set
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////

	
int i2c_rx(I2C_TypeDef *I2C, unsigned char *data){
	I2C->I2CONSET = (1 << 2);
	I2C->I2CONCLR = (1 << 3);
	while(!(I2C->I2CONSET & (1 << 3)));		//wait for SI to be set
	*data = I2C->I2DAT;
	return (I2C->I2STAT & 0xf8);
}





	



//////////////////////////////////////////////////////////////////////////////////////////////////////////

inline void i2c_stop(I2C_TypeDef *I2C){			//send stop on the bus
	
	I2C->I2CONSET = (1 << 4);
	//I2C->I2CONCLR = (1 << 3);
}
	

