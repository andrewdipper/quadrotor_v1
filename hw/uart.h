/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#pragma once
#include "mk20dx128.h"
#include "pins.h"

void uart0_init(uint32_t baud);
void uart0_tx(char c);
int uart0_rx(char *c);
void out0(char *c);
void outi0(int n);
void outih0(int n);
void uart0_rxb(char *c);
int uart0_rxlen(void);
int uart0_txlen(void);

void uart1_init(uint32_t baud);
void uart1_tx(char c);
int uart1_rx(char *c);
void out1(char *c);
void outi1(int n);
void outih1(int n);
void uart1_rxb(char *c);
int uart1_rxlen(void);



void uart2_init(uint32_t baud);


