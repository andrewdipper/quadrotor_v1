/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#pragma once
#include "mk20dx128.h"
#include "pins.h"
#include "uart.h"


void spi0_init(void);
char spi_tx(char c);
char spi_rx(void);

inline void spi_incr_cspeed(void);


//sdcard stuff

char spi_sendpackcrc(unsigned char a, unsigned int data, unsigned char crc);
char spi_sendpack(unsigned char a, unsigned int data);

int read_block(char *data, unsigned long address);
int write_block(char *data, unsigned long address);

int init_sdcard(void);
