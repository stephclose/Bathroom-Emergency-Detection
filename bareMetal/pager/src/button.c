#include "stm32f0xx.h"
#include "display.h"
#include "support.h"
#include "uart.h"
#include "button.h"
#include "clock.h"
#include "state.h"

//PB2 (EXTI2): Triggers the emergency alert
//PA0 (EXTI0): Acknowledges the emergency

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
    if (EXTI->PR & EXTI_PR_PR0) {
        if (!(GPIOA->IDR & GPIO_IDR_0)) { //confirm button is pressed
            uart_send_string("PA0 Button: Acknowledge\r\n");

            // Only acknowledge if in emergency mode
            if (lcd_state == LCD_STATE_EMERGENCY) {
                lcd_state = LCD_STATE_ACKNOWLEDGED;
            }
        }
        EXTI->PR |= EXTI_PR_PR0; // Clear pending flag
    }
}

//===========================================================================
// FAKE RF ACKNOWLEDGED, JUST SIMULATED THAT AN RF WAS RECIEVED OR SMT
//===========================================================================

void configure_pb2_exti(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable GPIOB clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // Enable SYSCFG

    GPIOB->MODER &= ~GPIO_MODER_MODER2; // PB2 as input
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR2;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR2_1; // Pull-down

    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI2;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PB; // Map EXTI2 to PB2

    EXTI->IMR |= EXTI_IMR_IM2; // Unmask EXTI2
    EXTI->FTSR |= EXTI_FTSR_TR2; // Trigger on falling edge

    NVIC_EnableIRQ(EXTI2_3_IRQn);
    NVIC_SetPriority(EXTI2_3_IRQn, 1);
}

void EXTI2_3_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR2) {
        if (lcd_state == LCD_STATE_STANDBY) {
            lcd_state = LCD_STATE_EMERGENCY;
        } else if (lcd_state == LCD_STATE_EMERGENCY) {
            lcd_state = LCD_STATE_ACKNOWLEDGED;
        }
        uart_send_string("PB2 Button Toggled\r\n");
        EXTI->PR |= EXTI_PR_PR2; // Clear pending flag
    }
}
