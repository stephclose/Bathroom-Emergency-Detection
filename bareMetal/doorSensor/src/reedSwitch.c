#include "stm32f091xc.h"
#include "reedSwitch.h"
#include "clock.h"
#include "rf.h"
#include <stdint.h>
#include <stdio.h>

#define REEDPIN 0 //pa0
#define DEBOUNCE 50 // 50 ms debounce delay

extern volatile uint32_t system_time; // systick counter
extern volatile uint8_t doorState;
extern volatile uint8_t doorEvent;
//static uint32_t lastISRChangeTime = 0; // for debounce timing 

void reedSwitch_init()
{
    // clockfor GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // set to input pa0
    GPIOA->MODER &= ~(3U << (REEDPIN * 2));

    // internal pull up resistor so pin is high when open
    GPIOA->PUPDR &= ~(3U << (REEDPIN * 2));
    GPIOA->PUPDR |=  (1U << (REEDPIN * 2)); // 01 = pull-up
}

uint8_t getState()
{
    // 1 open 0 close
    return (GPIOA->IDR & (1<<0)) ? 1 : 0;
}

/*
// interrupt logic to wake up the stm from deep sleep mode when a door state change happens
void EXTI0_1_IRQHandler(void) 
{
    if (EXTI->PR & (1 << REEDPIN)) // if interrupt from pa0
    {
        uint32_t currentTime = system_time; 

        if ((currentTime - lastISRChangeTime) >= DEBOUNCE) // check if 50 ms passed
        {
            doorState = getState();
            doorEvent = 1;
            lastISRChangeTime = currentTime;
            printf("ISR: Door state changed to %d\r\n", doorState);

        }
        EXTI->PR |= (1 << REEDPIN); // clear int flag
    }
}

void reedSwitchEnableInt()
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // system config controller

    SYSCFG->EXTICR[0] &= ~(0xF << (0 * 4)); // PA0 for the exti0

    EXTI->IMR |= (1 << 0); // unmask interrupt
    EXTI->RTSR |= (1 << 0); // rising edge trigger
    EXTI->FTSR |= (1 << 0); // falling edge trigger

    NVIC_EnableIRQ(EXTI0_1_IRQn); // enable int in NVIC
}
*/



