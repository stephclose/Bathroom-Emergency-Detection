#include "stm32f0xx_hal.h"
#include "button.h"
#include "buzzer.h"
#include "clock.h"
#include "error_handler.h"
#include "i2c_lcd.h"
#include "rfm9x.h"
#include "uart.h"
#include <stdio.h>


extern I2C_HandleTypeDef hi2c1;

int main(void) {
    HAL_Init(); 

    while (1) {
    }
}
