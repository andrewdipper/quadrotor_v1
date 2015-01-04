/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#pragma once
#include "mk20dx128.h"
#include "uart.h"

inline void pit_module_enable(void);

inline void pit0_set_int(uint32_t timerval);
inline void pit0_set(uint32_t timerval);
void pit0_isr(void);
inline int pit0_read(void);


inline void pit1_set_int(uint32_t timerval);
inline void pit1_set(uint32_t timerval);
void pit1_isr(void);
inline int pit1_read(void);


inline void pit2_set_int(uint32_t timerval);
inline void pit2_set(uint32_t timerval);
void pit2_isr(void);
inline int pit2_read(void);

