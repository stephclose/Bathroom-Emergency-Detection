#include "stm32f091xc.h"
#include "reedSwitch.h"
#include <stdint.h>
#include <stdio.h>

#define REEDPIN 0 //pa0
#define DEBOUNCE 50 // 50 ms debounce delay

extern volatile uint32_t system_time; // systick counter

void reedSwitch_init()
{
    // clockfor GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // set to input pa0
    GPIOA->MODER &= ~(0x3 << (REEDPIN * 2));

    // internal pull up resistor so pin is high when open
    GPIOA->PUPDR |= (0x1 << (REEDPIN * 2));
}

uint8_t getState()
{
    // 1 open 0 close
    return (GPIOA->IDR & (1 << REEDPIN)) ? 1 : 0; 
}


// interrupt logic to wake up the stm from deep sleep mode when a door state change happens
/*
void EXTI0_1_IRQHandler(void) 
{
    if (EXTI->PR & (1 << REEDPIN)) // if interrupt from pa0
    {
        uint32_t currentTime = system_time; 

        if ((currentTime - lastChangeTime) >= DEBOUNCE) // check if 50 ms passed
        {
            doorState = getState();
            if (doorState) printf("DOOR OPEN\r\n");
            else printf("DOOR CLOSED\r\n");
            
            lastChangeTime = currentTime;
        }
        
        EXTI->PR |= (1 << REEDPIN); // clear int flag
    }
}

void reedSwitchEnableInt()
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // system config controller

    SYSCFG->EXTICR[0] &= ~(0xF << (REEDPIN * 4)); // PA0 for the exti0
    EXTI->IMR |= (1 << REEDPIN); // unmask interrupt
    EXTI->RTSR |= (1 << REEDPIN); // rising edge trigger
    EXTI->FTSR |= (1 << REEDPIN); // falling edge trigger
    NVIC_EnableIRQ(EXTI0_1_IRQn); // enable int in NVIC
}
*/



