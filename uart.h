/*
Copyright (C) 2013  Andrew Pratt
View the README
*/

#pragma once
#include "LPC17xx.h"



//receive byte (blocking)
void uart_rx(UART_TypeDef *UART, char *data);
//receive byte (nonblocking)
int uart_rxnb(UART_TypeDef *UART,  char *data);

//initialize uart to default values
//only use uart0
void uart_init(UART_TypeDef *UART, int baud);

//transmit byte
void uart_tx(UART_TypeDef *UART, unsigned char data);

//print string in "data"
void out(UART_TypeDef *UART, char *data);
//print integer in "n"
void outi(UART_TypeDef *UART, int n);
//print integer in "n" in hex
void outih(UART_TypeDef *UART, int n);



//Testing stuff.......................................
unsigned char *get3buf(void);
int get3end(void);
int get3start(void);
void set3start(int n);
void set3end(int n);

void UART3_IRQHandler(void);
