#include "stm32f0xx.h"
#include "vibrator.h"
#include "support.h"
#include "clock.h"

void vibrator_motor_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~(0x3 << (1 * 2));  //clear bits
    GPIOC->MODER |=  (0x1 << (1 * 2));  //set as output
    GPIOC->OTYPER &= ~(1 << 1); //push-pull mode
    GPIOC->ODR &= ~(1 << 1); //start LOW (off)
}

void vibrator_motor_on(void) {
    GPIOC->ODR |= (1 << 1);  // Set PC1 HIGH
}

void vibrator_motor_off(void) {
    GPIOC->ODR &= ~(1 << 1); // Set PC1 LOW
}