/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#include "i2ca.h"

struct i2c_transfer *curr;

//TODO make this queue more efficient
#define I2CLISTLEN 	20
static struct i2c_transfer *i2clist[I2CLISTLEN];
static int i2clistind = 0;

void i2ca0_init(void){
	SIM_SCGC4 |= SIM_SCGC4_I2C0;

	I2C0_A1 = 0;
	I2C0_RA = 0;

	CORE_PIN18_CONFIG =  PORT_PCR_ODE | PORT_PCR_DSE | PORT_PCR_MUX(2);	//configure scl TEMP enable pullups
	CORE_PIN19_CONFIG =  PORT_PCR_ODE | PORT_PCR_DSE | PORT_PCR_MUX(2);	//configure sda	TEMP enable pullups


	I2C0_F = 0x85;		//how???			//400  khz somehow
	I2C0_C2 = (1 << 5);					//enable high drive
	I2C0_C1 = (1 << 7);					//12c enable / master mode/ start in transmit mode

	i2clistind = 0;

}

void i2ca0_purge(void){
	i2clistind = 0;
}

void i2ca0_recover(void){
	
	I2C0_C1 = 0;
	I2C0_S = 0xff;

	CORE_PIN19_CONFIG =  PORT_PCR_ODE | PORT_PCR_DSE | PORT_PCR_MUX(1);	//configure sda	TEMP enable pullups
	pmode(18, INPUT);
	
	int i = 0;
	while(i < 20){
		pset(19, 1);
		delay_us(1);
		pset(19, 0);
		delay_us(1);
		if(pread(18) == 1) ++i;
		else i = 0;
	}
		


	i2ca0_init();

}


int i2ca0_qtrans(struct i2c_transfer *i2ctrans){
	//message(" in qtrans");	
	i2ctrans->status = I2C_WAITING;
	__disable_irq();
	if(i2clistind >= I2CLISTLEN-1){ __enable_irq();	return -1;}
	i2clist[i2clistind++] = i2ctrans;
	__enable_irq();
	
	if(i2clistind == 1){
		curr = i2clist[0];

		curr->bufind = 0;
		curr->status = I2C_RUNNING;

		I2C0_S = (1 << 4) | (1 << 1);					//clear flags (necessary???)
		if(I2C0_S & (1 << 5) && I2C0_C1 & (1 << 5)){						//send restart since already master
			I2C0_C1 |= (1 << 4) | (1 << 2);
			//message("fork 1");
		}else{								//send start since not master				
			I2C0_C1 = (1 << 7) | (1 << 5) | (1 << 4) ;		//send start signal (enable module)  enable master transmit
		}
		
		

		I2C0_D = curr->address;

		I2C0_C1 |= (1 << 6);
	}

	return 0;
}





void i2c0_isr(void){
	__disable_irq();
	char status = I2C0_S;
	if(status & ((1 << 6) | (1 << 4) | (1 << 3))){
		//pdat("ERROR---:", status);
		I2C0_S |= (1 << 4);
	}
	if(curr->bufind < curr->buflen){					//still operations to complete
		if(curr->type == I2C_WRITE){
			I2C0_D = curr->buffer[curr->bufind++];
			//message("transmitted");
		}else if(curr->type == I2C_READ){
			if(I2C0_C1 & (1 << 4)){
				I2C0_C1 &= ~(1 << 4);
				char dum = I2C0_D;
				//message("read dum");
			}else{
				if(curr->buflen - curr->bufind == 1) I2C0_C1 |= (1 << 3);
				curr->buffer[curr->bufind++] = I2C0_D;
				//pdat("read byte", curr->buffer[curr->bufind - 1]);
			}
		}
	}else{										//nothing left to complete
		I2C0_C1 &= ~(1 << 3); 							///clear txak
		curr->status = I2C_FINISHED;
		int i;
		for(i = 1; i < i2clistind; ++i){
			i2clist[i-1] = i2clist[i];
		}
		--i2clistind;
		//pdat("listlen", i2clistind);
		if(i2clistind > 0){
			curr = i2clist[0];

			curr->bufind = 0;
			curr->status = I2C_RUNNING;

			I2C0_C1 |= (1 << 2) | (1 << 4);		//send start re signal 
			I2C0_D = curr->address;


		}else{	
			I2C0_C1 = (1 << 7) | (1 << 4) | (1 << 5);			//nothing left to but idle the bus
		}
	}


	I2C0_S |= (1 << 1);
	__enable_irq();
	
}


void message(char *msg){
	out0("STAT = ");
	outih0(I2C0_S);
	out0("  ");
	out0(msg);
	out0("\n");
}

void pdat(char *msg, char data){
	out0("Dat = ");
	outih0(data);
	out0("  ");
	out0(msg);
	out0("\n");
}

