/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#include "spi.h"

//uses bus clock F_BUS (usually 48mhz)

#define SPI0_BUF_SIZE 512			//size of a block
#define SPI0_NUM_BUFS 2

#define SPI0_STOPPED 	1			//no data to transfer and not transmitting
#define SPI0_RUNNING 	2			//data to transfer and transmitting
#define SPI0_FINISHING 	3			//transmitting but no more data to transfer


/*
static char tx0_buf[SPI0_NUM_BUFS][SPI0_BUF_SIZE];
static uint16_t tx0_buf_ind[SPI0_NUM_BUFS];
static uint16_t tx0_head;		//index where next character will be queued
static uint8_t tx0_bufind;
static uint8_t tx0_stat;
static uint32_t tx0_sdblock;


*/

/*
int spi_atx(char c){
	tx0_buf[tx0_bufind][tx0_head] = c;									//place the next character in the buffer
	if(++tx0_head >= SPI0_BUF_SIZE){
		tx0_head = 0; 
		init_block_awrite(tx0_sdblock, tx0_bufind);
		++tx0_sdblock;
		if(++tx0_bufind >= SPI0_NUM_BUFS) tx0_bufind = 0;
	}
	return 0;
}

int spi_aout(char *c, int num){
	int i;
	for(i = 0; i < num; ++i){
		tx0_buf[tx0_bufind][tx0_head] = c[i];						//place the next character in the buffer
		if(++tx0_head >= SPI0_BUF_SIZE){
			tx0_head = 0; 
			init_block_awrite(tx0_sdblock, tx0_bufind);
			++tx0_sdblock;
			if(++tx0_bufind >= SPI0_NUM_BUFS) tx0_bufind = 0;
		}
	}
	return 0;
}


int init_block_awrite(int blocknum, int bufnum){
	if(tx0_stat != SPI0_STOPPED) return -1;				//only one transfer at a time
	SPI0_MCR |= (1 << 11) | (1 << 10);				//clear the fifos (they should be empty because not transmitting)
	SPI0_MCR |= (1 << 31) | (1 << 16);				//master mode and make cs active low
	SPI0_SR = (1 << 31) | (1 << 28);				//clear flags	
	SPI0_RSER |= (1 << 25);						//enable fifo fill interrupt
	
	tx0_buf_ind[bufnum] = 0;

	int ret;
	while(spi_rx() != 0xff);
	ret = spi_sendpack(0x40 + 24, blocknum);
	while(ret != 0x00)ret = spi_rx();
	spi_tx(0xff);					//send it some clocks
	spi_tx(0xfe);	
	
	tx0_stat = SPI0_RUNNING;
	out0("before transin init");
	for(tx0_buf_ind[bufnum] = 0; (((SPI0_SR >> 12) & 0xf) < 4) & tx0_buf_ind[bufnum] < 512; ++tx0_buf_ind[bufnum]){
		SPI0_PUSHR = (tx0_buf[bufnum][tx0_buf_ind[bufnum]] & 0xff) | (1 << 16) | (1 << 31);
		out0("\nind = ");
		outi0(tx0_buf_ind[bufnum]);
	}

	tx0_stat = SPI0_STOPPED;
	return 0;
}
*/





void spi0_init(void){
	SIM_SCGC6 |= SIM_SCGC6_SPI0;				//enable clock

	CORE_PIN10_CONFIG = PORT_PCR_MUX(2);			//CS pin
	CORE_PIN11_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);			//DOUT pin
	CORE_PIN12_CONFIG = PORT_PCR_PS | PORT_PCR_PE | PORT_PCR_MUX(2);			//DIN pin
	CORE_PIN13_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);			//SCK pin

	SPI0_CTAR0 = (7 << 27) | (5 << 12) | (6 << 0); 		//8bit frame with baud of 93750*4 khz

				
	SPI0_MCR = (1 << 31) | (1 << 24) | (1 << 16) | (1 << 12) | (1 << 12);	//master mode and cs[0] is active low enable halt	

	//SCK baud rate = (fSYS/PBR) x [(1+DBR)/BR]
	
	
	
	
}


inline void spi_incr_cspeed(void){
	SPI0_CTAR0 = (1 << 31) | (7 << 27) | (0 << 12) | (0 << 0);			//increase to 24mhz


}

char spi_tx(char c){
	SPI0_MCR |= (1 << 31) | (1 << 16) | (1 << 10) | (1 << 11);
	SPI0_SR = (1 << 31) | (1 << 28);
	SPI0_PUSHR = (c & 0xff) | (1 << 16) | (1 << 31);			//write data and cs line
	while(!(SPI0_SR & (1 << 31)));				//wait for transfer to complete
	return SPI0_POPR;
}


char spi_rx(void) {
 	SPI0_MCR |= (1 << 31) | (1 << 16) | (1 << 10) | (1 << 11);
	SPI0_SR = (1 << 31) | (1 << 28);
	SPI0_PUSHR = (0xff) | (1 << 16) | (1 << 31);			//write data and cs line
	while(!(SPI0_SR & (1 << 31)));				//wait for transfer to complete
	return SPI0_POPR;
}


////////////////////////////////////////////////////////////////////////////////////
// SD CARD STUFF
////////////////////////////////////////////////////////////////////////////////////

char spi_sendpack(unsigned char a, unsigned int data){
	spi_tx(a);				//command byte
	spi_tx((data >> 24) & 0xff);		//send 4 data bytes
	spi_tx((data >> 16) & 0xff);
	spi_tx((data >> 8) & 0xff);
	spi_tx(data & 0xff);
	char ret = spi_tx(0x00);	//checksum is used only for CMD0
	while(ret == 0xff) ret = spi_tx(0xff);
	return ret;
}

char spi_sendpackcrc(unsigned char a, unsigned int data, unsigned char crc){
	spi_tx(a);				//command byte
	spi_tx((data >> 24) & 0xff);		//send 4 data bytes
	spi_tx((data >> 16) & 0xff);
	spi_tx((data >> 8) & 0xff);
	spi_tx(data & 0xff);
	char ret = spi_tx(crc);	//checksum is used only for CMD0
	while(ret == 0xff) ret = spi_tx(0xff);
	return ret;
}

int read_block(char *data, unsigned long address){
	int ret;
	while(spi_rx() != 0xff);
	ret = spi_sendpack(0x40 + 17, address);
	while(ret != 0x00)ret = spi_rx();
	while(spi_rx() != 0xfe);	
	int i;
	for(i=0; i < 512; ++i){
		data[i] = spi_rx();
	}
	int checksum = spi_rx() << 8;
	checksum |= spi_rx();
	return 0;
}


int write_block(char *data, unsigned long address){
	int ret;
	while(spi_rx() != 0xff);
	ret = spi_sendpack(0x40 + 24, address);
	while(ret != 0x00)ret = spi_rx();
	spi_tx(0xff);					//send it some clocks
	spi_tx(0xfe);					//data token start
	
	int i;
	for(i = 0; i < 512; ++i){
		spi_tx(data[i]);
	}
	spi_tx(0x00);					//send false checksum
	return spi_tx(0x00);				//return status
}

int init_sdcard(void){
	int i, ret;
	for(i = 0; i < 200; ++i){					//give card time to boot
		spi_tx(0xff);
	}
	
	spi_sendpackcrc(0x40 + 0, 0x00000000, 0x95);

	spi_sendpackcrc(0x40 + 8, 0x000001aa, 0x87);

	spi_sendpack(0x40 + 55, 0x00000000);				//tell card will be sending acmd	
	ret = spi_sendpack(0x40 + 41, 0x40000000);			//send acmd 41
	while(ret != 0x00){spi_sendpack(0x40 + 55, 0x00000000); ret = spi_sendpack(0x40 + 41, 0x40000000); }

	spi_incr_cspeed();

	//TODO add error checking
	return 0;
}





