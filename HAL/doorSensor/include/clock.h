#ifndef CLOCK_H
#define CLOCK_H
#include "stm32f091xc.h"    
#include "system_stm32f0xx.h" 
#include "core_cm0.h"        


void internal_clock();
void setup_lse_clock(); // config ext crystal for rtc
void init_systick(void); // up 1 ms
void SysTick_Handler(void);
void delay_ms(int ms); // delay light blinking or whatevr

#endif
