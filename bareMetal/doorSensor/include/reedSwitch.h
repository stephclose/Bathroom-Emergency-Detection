#ifndef REEDSWITCH_H
#define REEDSWITCH_H

#include <stdint.h>


void reedSwitch_init(void);
uint8_t getState(void);
//void reedSwitchEnableInt(void);
//void EXTI0_1_IRQHandler(void); // int handler for state change

#endif 
