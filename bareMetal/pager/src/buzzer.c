#include "stm32f0xx.h"
#include "buzzer.h"
#include "clock.h"
#include "support.h"

void setup_tim2_pwm_pa15(void) {
    // Enable clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Set PA15 to alternate function AF0
    GPIOA->MODER &= ~(0x3 << (15 * 2));
    GPIOA->MODER |=  (0x2 << (15 * 2));  // AF mode

    GPIOA->AFR[1] &= ~(0xF << 28);
    GPIOA->AFR[1] |=  (0x0 << 28);       // AF0 = TIM2_CH1

    // Configure TIM2
    TIM2->PSC = 47;       // 1 MHz
    TIM2->ARR = 499;      // 2 kHz
    TIM2->CCR1 = 250;     // 50% duty cycle

    TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM2->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos); // PWM Mode 1
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;

    TIM2->CCER |= TIM_CCER_CC1E;
    TIM2->CR1  |= TIM_CR1_ARPE;
    TIM2->EGR  |= TIM_EGR_UG;  // Force update
    TIM2->CR1  |= TIM_CR1_CEN; // Start timer
}


void buzzer_test_gpio_toggle(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~(0x3 << (15 * 2));
    GPIOA->MODER |=  (0x1 << (15 * 2));

    while (1) {
        GPIOA->ODR ^= (1 << 15); 
        nano_wait(250000);
    }
}

//here!!!

static int buzzer_on_flag = 0;

void buzzer_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~(0x3 << (15 * 2));
    GPIOA->MODER |=  (0x1 << (15 * 2)); // Output mode
    GPIOA->ODR &= ~(1 << 15); // start low
}

void buzzer_on(void) {
    buzzer_on_flag = 1;
}

void buzzer_off(void) {
    buzzer_on_flag = 0;
    GPIOA->ODR &= ~(1 << 15); // force low
}

void buzzer_poll(void) {
    static int count = 0;

    if (buzzer_on_flag) {
        count++;
        if (count >= 250) {   // ~500 Hz with 1ms polling
            GPIOA->ODR ^= (1 << 15);
            count = 0;
        }
    }
}
