#ifndef REEDSWITCH_H
#define REEDSWITCH_H

#include <stdint.h>

#define REEDPIN 0 // PA0 pin used for Reed Switch
extern volatile uint8_t doorState; // door state variable

void reedSwitch_init(void);
uint8_t getState(void);
void reedSwitchEnableInt(void);
void EXTI0_1_IRQHandler(void); // int handler for state change

#endif // REEDSWITCH_H
