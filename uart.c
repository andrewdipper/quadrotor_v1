/*
Copyright (C) 2013  Andrew Pratt
View the README
*/

#include "uart.h"

//testing variables
static int start3;
static int end3;	
static unsigned char uart3buf[800];




//initialize uart to default values
//only use uart0
void uart_init(UART_TypeDef *UART,  int baud){	// expects UART0  UART2 or UART3
	int dlm = (100000000)/(16*baud);
	int dll = dlm % 256;
	dlm /= 256;
	if(UART == UART0){	
		SC->PCONP |= (1 << 3); 			//power up UART0
		SC->PCLKSEL0 |= (1 << 6);		//choose clock for UART0 (cclk)
		UART0->LCR |= (1 << 7);			//sets DLAB
		UART0->DLM = dlm;			//clock divider msb				02
		UART0->DLL = dll;			//clock divider lsb( result in 9600 baud)	8b
		UART0->LCR |= 0x03;                  	//(1 << 0) | (1 << 1) // select 8bit mode
		UART0->FCR |= (1 << 0);			//enable fifo
		PINCON->PINSEL0 |= (1 << 4) | (1 << 6);	//select pins for uart

	}else if(UART == UART2){
		SC->PCONP |= (1 << 24); 		//power up UART2 //not tested
		SC->PCLKSEL1 |= (1 << 16);		//choose clock for UART2 (cclk)
		UART2->LCR |= (1 << 7);			//sets DLAB
		UART2->DLM = dlm;			//clock divider msb				02
		UART2->DLL = dll;			//clock divider lsb( result in 9600 baud)	8b
		UART2->LCR |= 0x03;                  	//(1 << 0) | (1 << 1) // select 8bit mode
		UART2->FCR |= (1 << 0);			//enable fifo
		PINCON->PINSEL0 |= (1 << 20) | (1 << 22);	//select pins for uart

	}else if(UART == UART3){
		SC->PCONP |= (1 << 25); 		//power up UART3 //not tested
		SC->PCLKSEL1 |= (1 << 18);		//choose clock for UART3 (cclk)
		UART3->LCR |= (1 << 7);			//sets DLAB
		UART3->DLM = dlm;			//clock divider msb				02
		UART3->DLL = dll;			//clock divider lsb( result in 9600 baud)	8b
		UART3->LCR &= ~(1 << 7);		//clear DLAB
		UART3->LCR |= 0x03;                  	//(1 << 0) | (1 << 1) // select 8bit mode
		UART3->FCR |= (1 << 6) | (1 << 7);	//interrupt when 14 characters available
		UART3->FCR |= (1 << 0);			//enable fifo
		PINCON->PINSEL9 |= (1 << 24) | (1 << 25) | (1 << 26) | (1 << 27);	//select pins for uart
		start3 = 0;
		end3 = 0;
		
		NVIC_EnableIRQ(UART3_IRQn);
		NVIC_SetPriority(UART3_IRQn, 1);
		
		UART3->IER |= (1 << 0);		//enable Receive Data Available interrupt
		
		
	
	}

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////////

//send byte
void uart_tx(UART_TypeDef *UART, unsigned char data){ 

		UART->LCR &= ~(1 << 7);          	//clears DLAB providing access to THR
		while(!(UART->LSR & (1 << 5)));	//waits until THR is empty
		UART->THR = (data & 0xff); //place data in THR  (try without ANDING 0xff)****	
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////

//receive byte (blocking)
void uart_rx(UART_TypeDef *UART,  char *data){ //Does not work with UART 1
		
		while(!(UART->LSR & 0x01));		//wait for data 
		UART->LCR &= ~(1 << 7);          	//clears DLAB providing access to THR
		*data = (UART->RBR & 0xff);		//writes character into data
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////

//receive byte (nonblocking)
int uart_rxnb(UART_TypeDef *UART,  char *data){ //Does not work with UART 1
		
		if(UART->LSR & 0x01){		//wait for data 
			UART->LCR &= ~(1 << 7);          	//clears DLAB providing access to THR
			*data = (UART->RBR & 0xff);		//writes character into data
			return 1;
		}else return 0;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////

//print out string in "data"
void out(UART_TypeDef *UART, char *data){		//Does not work with UART 1
	UART->LCR &= ~(1 << 7); 
	int i = 0;
	while(data[i] != 0){
		while(!(UART->LSR & (1 << 5)));		//waits until THR is empty
		UART->THR = (data[i++] & 0x00ff);
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////

//print out integer in "n"	
void outi(UART_TypeDef *UART, int n){
	if(n < 0){
		n *= -1;
		while(!(UART->LSR & (1 << 5)));		//waits until THR is empty
		UART->THR = ('-' & 0x00ff);
	}
	int i = 0;
	unsigned short t[13]; 
	for(i = 0;n/10 > 0; i++){
		t[i] = (n%10) + 48;

		n /= 10;
	}
	 t[i] = (n % 10) + 48;

	UART->LCR &= ~(1 << 7);  
	for(;i >= 0; i--){
		while(!(UART->LSR & (1 << 5)));		//waits until THR is empty
		UART->THR = (t[i] & 0x00ff);
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////

//print out "n" in hex
void outih(UART_TypeDef *UART, int n){
	if(n < 0){
		n *= -1;
		while(!(UART->LSR & (1 << 5)));		//waits until THR is empty
		UART->THR = ('-' & 0x00ff);
	}
	int i = 0;
	unsigned short t[13]; 
	for(i = 0;n/16 > 0; i++){
		if(n % 16 > 9) t[i] = ((n % 16) - 10 + 'a');
	 	else t[i] = (n % 16) + 48;
		n /= 16;
	}
	if(n % 16 > 9) t[i] = ((n % 16) - 10 + 'a');
	 else t[i] = (n % 16) + 48;

	UART->LCR &= ~(1 << 7);  
	for(;i >= 0; i--){
		while(!(UART->LSR & (1 << 5)));		//waits until THR is empty
		UART->THR = (t[i] & 0x00ff);
	}
}










//////
//////
/////The rest of this is just testing stuff for UART3 (make it interrupt based)


unsigned char *get3buf(void){
	return uart3buf;
}

int get3end(void){
	return end3;
}

int get3start(void){
	return start3;
}

void set3start(int n){
	start3 = n;
}

void set3end(int n){
	end3 = n;
}


void UART3_IRQHandler(void){
	int type = UART3->IIR & 0x0e;
	if(type != 0x04) return;
	while((UART3->LSR & 0x01)){ uart3buf[end3++] = UART3->RBR; if(end3 >= 400) end3 = 0; if(start3 == end3) ++start3;}
	
}


