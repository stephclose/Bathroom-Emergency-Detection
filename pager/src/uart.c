#include "stm32f0xx.h"
#include "uart.h"
#include "support.h"
#include <stdio.h>

//===========================================================================
// Initialize USART2 (PA2 = TX, PA3 = RX)
//===========================================================================
void uart_init(void) {
    //GPIOC Clock (For Debugging LED on PC6)
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;  
    GPIOC->MODER |= (1 << (6 * 2));  //PC6 as output
    uart_send_string("USART2 Initialized!\r\n");

    //USART2 Clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    //toggle PC6 (Start of UART Init)
    GPIOC->ODR ^= (1 << 6); 
    nano_wait(1000000000); 
    uart_send_string("DEBUG: GPIO Config Start\r\n");

    //PA2 (TX), PA3 (RX) to Alternate Function (AF1)
    GPIOA->MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2))); 
    GPIOA->MODER |= (2 << (2 * 2)) | (2 << (3 * 2));    
    GPIOA->AFR[0] &= ~((0xF << (4 * 2)) | (0xF << (4 * 3))); 
    GPIOA->AFR[0] |= (1 << (4 * 2)) | (1 << (4 * 3));   

    //flash PC6 (GPIO Config Done)
    GPIOC->ODR ^= (1 << 6); 
    nano_wait(1000000000);
    uart_send_string("DEBUG: GPIO Config Done\r\n");

    //set Baud Rate
    USART2->BRR = 48000000 / 115200; 

    //flash PC6 (Baud Rate Set)
    GPIOC->ODR ^= (1 << 6); 
    nano_wait(1000000000); 
    uart_send_string("DEBUG: Baud Rate Set\r\n");

    //USART2 Transmitter & Receiver
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE; 

    //flash PC6 (UART Enabled)
    GPIOC->ODR ^= (1 << 6); 
    nano_wait(1000000000);
    uart_send_string("DEBUG: USART2 Enabled\r\n");
}

//===========================================================================
// Send a Single Character via USART2
//===========================================================================
void uart_send_char(char c) {
    while (!(USART2->ISR & USART_ISR_TXE)); // Wait for TX buffer to be empty
    USART2->TDR = c;  // Send char
}

//===========================================================================
// Send a String via USART2
//===========================================================================
void uart_send_string(const char *str) {
    while (*str) {
        uart_send_char(*str++); // Send character-by-character
        nano_wait(500000);
    }
}

//===========================================================================
// Debug Function to Print the USART Baud Rate
//===========================================================================
void debug_uart(void) {
    char buf[50];
    uint32_t baud_setting = USART2->BRR;
    sprintf(buf, "USART2 BRR: 0x%08lX (%lu)\r\n", (unsigned long)baud_setting, (unsigned long)baud_setting);
    uart_send_string(buf);
}