#include "stm32f0xx.h"
#include "clock.h"
#include "reedSwitch.h"
#include "uart.h"
#include <stdio.h>

volatile uint8_t doorState = 1; // door starts open
volatile uint32_t lastChangeTime = 0; // system time for debounce 
extern volatile uint32_t system_time;



int main(void) {
    internal_clock(); // configure hsi in case this is probelm of gibberish 
    init_systick(); // init systick timer

    uart_init(); // uart communication 

    reedSwitch_init(); // initalize reed switch gpio


    // PC6 for LED output for gpio since i cant print :|
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER |= (1 << 12); // Set PC6 as output
    GPIOC->MODER &= ~(1 << 13); // Clear bit for output mode

    while (1) 
    {
        uint32_t currentTime = system_time;

        // read the reed switch with 50 ms debounce 
        if ((currentTime - lastChangeTime) >= 50) 
        { 
            uint8_t newState = getState(); 
            if (newState != doorState) {
                doorState = newState;

                // LED: ON if closed with magnet, OFF if open
                if (doorState == 0) {
                    GPIOC->ODR |= (1 << 6); //ON LED
                } else {
                    GPIOC->ODR &= ~(1 << 6); //OFF LED
                }

                lastChangeTime = currentTime; // debounce timer update 
            }
        }










        uart_send_string("Loopback test: Sending data to RX...\r\n"); // not working 
        // for UART testing gibberish 
        if (USART2->ISR & USART_ISR_RXNE) // data recieved 
        { 
            char received = USART2->RDR;   // data was read 
            uart_send_char(received);// echo back data 
        }      
    
    }
}
