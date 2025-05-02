#include "stm32f0xx.h"
#include "buzzer.h"
#include "clock.h"
#include "support.h"

void buzzer_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~(0x3 << (0 * 2));
    GPIOC->MODER |=  (0x1 << (0 * 2));
    GPIOC->ODR &= ~(1 << 0);  // Start OFF
}

void buzzer_on(void) {
    GPIOC->ODR |= (1 << 0);
}

void buzzer_off(void) {
    GPIOC->ODR &= ~(1 << 0);
}

void buzzer_pulse(uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
        buzzer_on();
        delay_ms(100);  // ON duration
        buzzer_off();
        delay_ms(100);  // OFF duration
    }
}