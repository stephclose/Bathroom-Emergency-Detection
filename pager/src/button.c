#include "stm32f0xx.h"
#include "display.h"
#include "support.h"
#include "uart.h"
#include "button.h"
#include "clock.h"

//===========================================================================
// Configure Button on PA0 as External Interrupt (EXTI)
//===========================================================================
void button_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA->MODER &= ~GPIO_MODER_MODER0;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0; //map PA0 to EXTI0
    EXTI->IMR |= EXTI_IMR_IM0;  //unmask EXTI0
    EXTI->RTSR |= EXTI_RTSR_TR0; //trigger on rising edge (button release)
    EXTI->FTSR |= EXTI_FTSR_TR0; //trigger on falling edge (button press)

    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_SetPriority(EXTI0_1_IRQn, 2); //set priority level
}


//===========================================================================
// EXTI Interrupt Handler for PA0 Button Press
//===========================================================================
void EXTI0_1_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) {  //check if PA0 triggered interrupt
        if (!(GPIOA->IDR & GPIO_IDR_0)) { //confirm button is pressed
            uart_send_string("Button Pressed!\r\n");
        }
        lcd_clear();
        lcd_set_cursor(0, 0); 
        lcd_send_string("Emergency");
        lcd_set_cursor(1, 0); 
        lcd_send_string("Acknowledged");
        EXTI->PR |= EXTI_PR_PR0; //clear interrupt flag
    }
}