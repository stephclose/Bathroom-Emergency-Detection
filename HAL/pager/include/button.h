#ifndef BUTTON_H
#define BUTTON_H

#include "stm32f0xx_hal.h"

void Button_Init(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif // BUTTON_H
