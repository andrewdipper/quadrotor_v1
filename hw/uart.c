/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#include "uart.h"



#ifndef F_BUS
#define F_BUS 48000000
#endif

#ifndef IRQ_PRIORITY
#define IRQ_PRIORITY 64
#endif
//0 highest 255 lowest


#define UART0_BUF_SIZE 2048

static uint8_t tx0_buf[UART0_BUF_SIZE];
static uint8_t rx0_buf[UART0_BUF_SIZE];
static uint16_t tx0_head;		//index where next character will be queued
static uint16_t tx0_tail;		//index of next character that will be dequeued and sent
static uint16_t rx0_head;		//index where next character received will be queued
static uint16_t rx0_tail;		//index of next character that will be dequeued and returned in a read

	

//uart0 uses system core clock
//make sure there is a good close ground or bytes will be lost/duplicated
void uart0_init(uint32_t baud){
	uint32_t divisor = ((F_CPU) * 2 + baud/2) / baud;
	SIM_SCGC4 |= SIM_SCGC4_UART0;								//enable clock

	CORE_PIN0_CONFIG = PORT_PCR_MUX(3);		
	CORE_PIN1_CONFIG = PORT_PCR_MUX(3);	

	UART0_BDH = (divisor >> 13) & 0x1f;
	UART0_BDL = (divisor >> 5) & 0xff;							//bdl and bdh multiplied by 16
	UART0_C4 = divisor & 0x1F; 								//fractional part of divider
	UART0_C1 = UART_C1_ILT;									//read idle after stop bits
	UART0_TWFIFO = 1;
	UART0_RWFIFO = 1;
	UART0_PFIFO = 0xaa;									//8byte rx and tx fifo
	UART0_CFIFO |= 0xC0; 									//flush buffers
	

	UART0_C2 = 0x2c;									//enable RX and TX and rx interrupt
	
	tx0_head = 0;
	tx0_tail = 0;
	rx0_head = 0;
	rx0_tail = 0;

	return;
}


void uart0_status_isr(void){
	char status = UART0_S1;
	if(status & (1 << 5)){														//read the data from the fifo
		while(UART0_RCFIFO > 0){
			status = UART0_S1;									//necessary to clear flag
			rx0_buf[rx0_head] = UART0_D;
			if(++rx0_head >= UART0_BUF_SIZE) rx0_head = 0;
			if(rx0_head == rx0_tail){
				if(++rx0_tail >= UART0_BUF_SIZE) rx0_tail = 0;					//overwrite data if the buffer is too full
			}
		}
	}else if(status & (1 << 7)){
		if(tx0_head == tx0_tail){												//if the user queue is empty then disable TIE so it doesn't reassert
			UART0_C2 &= ~(1 << 7);												
		}else{
			while(tx0_head != tx0_tail && UART0_TCFIFO < 8){					//otherwise copy from user queue to the tx fifo
				status = UART0_S1;
				UART0_D = tx0_buf[tx0_tail];
				if(++tx0_tail >= UART0_BUF_SIZE) tx0_tail = 0;
			}
		}
	}
}


void uart0_tx(char c){				//transmitter should be enabled in init	(always remains on)
	__disable_irq();	
	char status;
	tx0_buf[tx0_head] = c;									//place the next character in the buffer
	if(++tx0_head >= UART0_BUF_SIZE) tx0_head = 0;
	if(tx0_head == tx0_tail){
		if(++tx0_tail >= UART0_BUF_SIZE) tx0_tail = 0;							//overwrite data if the buffer is too full
	}	
						
	while(tx0_head != tx0_tail && UART0_TCFIFO < 8){
		status = UART0_S1;					//read status so TDRE flag clears	
		UART0_D = tx0_buf[tx0_tail];
		if(++tx0_tail >= UART0_BUF_SIZE) tx0_tail = 0;
	}
	if(UART0_TCFIFO > UART0_TWFIFO) UART0_C2 |= (1 << 7); 		//enable tie interrupt because more than watermark data is in the fifo
	__enable_irq();
}




int uart0_rx(char *c){	
	__disable_irq();
	while(UART0_RCFIFO > 0){
		rx0_buf[rx0_head] = UART0_D;
		if(++rx0_head >= UART0_BUF_SIZE) rx0_head = 0;
		if(rx0_head == rx0_tail){
			if(++rx0_tail >= UART0_BUF_SIZE) rx0_tail = 0;							//overwrite data if the buffer is too full
		}
	}		
	if(rx0_head == rx0_tail){ __enable_irq(); return -1;}
	*c = rx0_buf[rx0_tail];
	if(++rx0_tail >= UART0_BUF_SIZE) rx0_tail = 0;	
	__enable_irq();
	return 0;
	
}

void uart0_rxb(char *c){
	while(uart0_rx(c) == -1);
}

int uart0_rxlen(void){
	__disable_irq();
	if(rx0_head < rx0_tail){ __enable_irq(); return rx0_head + UART0_BUF_SIZE - rx0_tail;}
	int tmp =  rx0_head - rx0_tail;
	__enable_irq();
	return tmp;
}

int uart0_txlen(void){
	__disable_irq();
	if(tx0_head < tx0_tail){ __enable_irq(); return tx0_head + UART0_BUF_SIZE - tx0_tail;}
	int tmp =  tx0_head - tx0_tail;
	__enable_irq();
	return tmp;
}

void out0(char *c){					//transmitter should be enabled in init (always remains on)
	__disable_irq();
	int i;
	char status;
	for(i=0; c[i] != 0; ++i){													//copy from input string into user queue
		tx0_buf[tx0_head] = c[i];
		if(++tx0_head >= UART0_BUF_SIZE) tx0_head = 0;
		if(tx0_head == tx0_tail){
			if(++tx0_tail >= UART0_BUF_SIZE) tx0_tail = 0;						//overwrite data if the buffer is too full
		}
	}
					
	while(tx0_head != tx0_tail && UART0_TCFIFO < 8){
		status = UART0_S1;														//read status so TDRE flag clears					
		UART0_D = tx0_buf[tx0_tail];
		if(++tx0_tail >= UART0_BUF_SIZE) tx0_tail = 0;
	}
	if(UART0_TCFIFO > UART0_TWFIFO) UART0_C2 |= (1 << 7); 						//enable tie interrupt because more than watermark data is in the fifo
	__enable_irq();
}



void outi0(int n){ 
	if(n < 0){
		n *= -1;
		uart0_tx('-');
	}
	int i = 0;
	unsigned short t[16]; 
	for(i = 0;n/10 > 0; i++){
		t[i] = (n%10) + 48;

		n /= 10;
	}
	 t[i] = (n % 10) + 48;

	for(;i >= 0; i--){
		uart0_tx(t[i]);
	}
}

void outih0(int n){
	if(n < 0){
		n *= -1;
		uart0_tx('-');
	}
	int i = 0;
	unsigned short t[16]; 
	for(i = 0;n/16 > 0; i++){
		if(n % 16 > 9) t[i] = ((n % 16) - 10 + 'a');
	 	else t[i] = (n % 16) + 48;
		n /= 16;
	}
	if(n % 16 > 9) t[i] = ((n % 16) - 10 + 'a');
	 else t[i] = (n % 16) + 48;

	for(;i >= 0; i--){
		uart0_tx(t[i]);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//UART1_____________________________________________________________________________________________________________
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//TODO CURRENTLY UNTESTED

#define UART1_BUF_SIZE 2048

static uint8_t tx1_buf[UART1_BUF_SIZE];
static uint8_t rx1_buf[UART1_BUF_SIZE];
static uint16_t tx1_head;		//index where next character will be queued
static uint16_t tx1_tail;		//index of next character that will be dequeued and sent
static uint16_t rx1_head;		//index where next character received will be queued
static uint16_t rx1_tail;		//index of next character that will be dequeued and returned in a read

	

//uart1 uses system core clock
//make sure write speed does not exceed the transfer speed
void uart1_init(uint32_t baud){
	uint32_t divisor = ((F_CPU) * 2 + baud/2) / baud;							
	SIM_SCGC4 |= SIM_SCGC4_UART1;								//enable clock

	CORE_PIN9_CONFIG = PORT_PCR_MUX(3);							//Configure rx pin
	CORE_PIN10_CONFIG = PORT_PCR_MUX(3);							//Configure tx pin
	

	UART1_BDH = (divisor >> 13) & 0x1f;
	UART1_BDL = (divisor >> 5) & 0xff;							//bdl and bdh multiplied by 16
	UART1_C4 = divisor & 0x1F; 								//fractional part of divider
	UART1_C1 = UART_C1_ILT;									//read idle after stop bits
	UART1_TWFIFO = 1;									//TODO change these to desired value
	UART1_RWFIFO = 1;									//TODO change these to desired value
	UART1_PFIFO = 0xaa;									//8byte rx and tx fifo
	UART1_CFIFO |= 0xC0; 									//flush buffers
	

	UART1_C2 = 0x2c;									//enable RX and TX and rx interrupt
	
	tx1_head = 0;
	tx1_tail = 0;
	rx1_head = 0;
	rx1_tail = 0;

}


void uart1_status_isr(void){
	char status = UART1_S1;
	if(status & (1 << 5)){														//read the data from the fifo
		while(UART1_RCFIFO > 0){
			status = UART1_S1;
			rx1_buf[rx1_head] = UART1_D;
			if(++rx1_head >= UART1_BUF_SIZE) rx1_head = 0;
			if(rx1_head == rx1_tail){
				if(++rx1_tail >= UART1_BUF_SIZE) rx1_tail = 0;					//overwrite data if the buffer is too full
			}
		}
	}else if(status & (1 << 7)){
		if(tx1_head == tx1_tail){									//if the user queue is empty then disable TIE so it doesn't reassert
			UART1_C2 &= ~(1 << 7);												
		}else{
			while(tx1_head != tx1_tail && UART1_TCFIFO < 8){					//otherwise copy from user queue to the tx fifo
				status = UART1_S1;
				UART1_D = tx1_buf[tx1_tail];
				if(++tx1_tail >= UART1_BUF_SIZE) tx1_tail = 0;
			}
		}
	}
}


void uart1_tx(char c){												//transmitter should be enabled in init	(always remains on)
	__disable_irq();	
	char status;
	tx1_buf[tx1_head] = c;											//place the next character in the buffer
	if(++tx1_head >= UART1_BUF_SIZE) tx1_head = 0;
	if(tx1_head == tx1_tail){
		if(++tx1_tail >= UART1_BUF_SIZE) tx1_tail = 0;							//overwrite data if the buffer is too full
	}	
						
	while(tx1_head != tx1_tail && UART1_TCFIFO < 8){
		status = UART1_S1;										//read status so TDRE flag clears	
		UART1_D = tx1_buf[tx1_tail];
		if(++tx1_tail >= UART1_BUF_SIZE) tx1_tail = 0;
	}
	if(UART1_TCFIFO > UART1_TWFIFO) UART1_C2 |= (1 << 7); 							//enable tie interrupt because more than watermark data is in the fifo
	__enable_irq();
}




int uart1_rx(char *c){	
	__disable_irq();
	while(UART1_RCFIFO > 0){
		rx1_buf[rx1_head] = UART1_D;
		if(++rx1_head >= UART1_BUF_SIZE) rx1_head = 0;
		if(rx1_head == rx1_tail){
			if(++rx1_tail >= UART1_BUF_SIZE) rx1_tail = 0;							//overwrite data if the buffer is too full
		}
		
	}		
	if(rx1_head == rx1_tail){ __enable_irq(); return -1;}
	*c = rx1_buf[rx1_tail];
	if(++rx1_tail >= UART1_BUF_SIZE) rx1_tail = 0;	
	__enable_irq();
	return 0;
	
}

void uart1_rxb(char *c){
	while(uart1_rx(c) == -1);
}


int uart1_rxlen(void){
	__disable_irq();
	if(rx1_head < rx1_tail){ __enable_irq(); return rx1_head + UART1_BUF_SIZE - rx1_tail;}
	int tmp =  rx1_head - rx1_tail;
	__enable_irq();
	return tmp;
}


void out1(char *c){					//transmitter should be enabled in init (always remains on)
	__disable_irq();
	int i;
	char status;
	for(i=0; c[i] != 0; ++i){													//copy from input string into user queue
		tx1_buf[tx1_head] = c[i];
		if(++tx1_head >= UART1_BUF_SIZE) tx1_head = 0;
		if(tx1_head == tx1_tail){
			if(++tx1_tail >= UART1_BUF_SIZE) tx1_tail = 0;						//overwrite data if the buffer is too full
		}
	}
					
	while(tx1_head != tx1_tail && UART1_TCFIFO < 8){
		status = UART1_S1;														//read status so TDRE flag clears					
		UART1_D = tx1_buf[tx1_tail];
		if(++tx1_tail >= UART1_BUF_SIZE) tx1_tail = 0;
	}
	if(UART1_TCFIFO > UART1_TWFIFO) UART1_C2 |= (1 << 7); 						//enable tie interrupt because more than watermark data is in the fifo
	__enable_irq();
}



void outi1(int n){ 
	if(n < 0){
		n *= -1;
		uart1_tx('-');
	}
	int i = 0;
	unsigned short t[16]; 
	for(i = 0;n/10 > 0; i++){
		t[i] = (n%10) + 48;

		n /= 10;
	}
	 t[i] = (n % 10) + 48;

	for(;i >= 0; i--){
		uart1_tx(t[i]);
	}
}

void outih1(int n){
	if(n < 0){
		n *= -1;
		uart1_tx('-');
	}
	int i = 0;
	unsigned short t[16]; 
	for(i = 0;n/16 > 0; i++){
		if(n % 16 > 9) t[i] = ((n % 16) - 10 + 'a');
	 	else t[i] = (n % 16) + 48;
		n /= 16;
	}
	if(n % 16 > 9) t[i] = ((n % 16) - 10 + 'a');
	 else t[i] = (n % 16) + 48;

	for(;i >= 0; i--){
		uart1_tx(t[i]);
	}
}














//uart2 uses bus clock
void uart2_init(uint32_t baud){
									//enable TX and RX
}


