#include "stm32f0xx.h"
#include "vibrator.h"
#include "support.h"
#include "clock.h"
#include "uart.h"
#include "debug.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void vibrator_motor_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~(0x3 << (0 * 2));
    GPIOC->MODER |=  (0x1 << (0 * 2));  // Output mode
    GPIOC->OTYPER &= ~(1 << 0);         // Push-pull
}

void vibrator_motor_on(void) {
    GPIOC->ODR |= (1 << 0);
}

void vibrator_motor_off(void) {
    GPIOC->ODR &= ~(1 << 0);
}

void test_vibrator_motor(void) {
    uart_send_string("Activating vibrator motor...\r\n");
    vibrator_motor_on();
    delay_ms(500);
    vibrator_motor_off();
    uart_send_string("Motor off.\r\n");
}
