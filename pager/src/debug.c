#include "stm32f0xx.h"
#include "support.h"
//===========================================================================
// LED DEBUG
//===========================================================================
void pc8_led_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; 
    GPIOC->MODER &= ~GPIO_MODER_MODER8; 
    GPIOC->MODER |= GPIO_MODER_MODER8_0; 
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT_8; 
    GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8; 
    GPIOC->ODR &= ~GPIO_ODR_8;
}

void toggle_pc8_debug(int times, int delay_ms) {
    for (int i = 0; i < times; i++) {
        GPIOC->ODR ^= GPIO_ODR_8;
        nano_wait(delay_ms * 1000000);
        GPIOC->ODR ^= GPIO_ODR_8;
        nano_wait(delay_ms * 1000000);
    }
}

void pc7_led_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~GPIO_MODER_MODER7;
    GPIOC->MODER |= GPIO_MODER_MODER7_0;
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT_7;
    GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7;
    GPIOC->ODR &= ~GPIO_ODR_7;
}

void toggle_pc7_debug(int times, int delay_ms) {
    for (int i = 0; i < times; i++) {
        GPIOC->BSRR = GPIO_BSRR_BS_7; //on
        nano_wait(delay_ms * 1000000);
        GPIOC->BSRR = GPIO_BSRR_BR_7; //off
        nano_wait(delay_ms * 1000000);
    }
}