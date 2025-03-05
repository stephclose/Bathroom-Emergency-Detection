#include "stm32f0xx.h"
#include "usart.h"

//===========================================================================
// Initialize USART2 (PA2 = TX, PA3 = RX)
//===========================================================================
void usart_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; //enable USART2 clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //enable GPIOA clock

    //configure PA2 (TX) as Alternate Function
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1; //AF mode for TX and RX

    GPIOA->AFR[0] |= (1 << (2 * 4)) | (1 << (3 * 4)); //AF1 (USART2_TX, USART2_RX)

    //configure USART2
    USART2->BRR = 48000000 / 115200; //set baud rate 48MHz / 115200
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE; //enable transmit and receive
    USART2->CR1 |= USART_CR1_UE; //enable USART2
}

//===========================================================================
// Send a Single Character via USART2
//===========================================================================
void uart_send_char(char c) {
    while (!(USART2->ISR & USART_ISR_TXE)); //wait for empty buffer
    USART2->TDR = c;  //send char
}

//===========================================================================
// Send a String via USART2
//===========================================================================
void uart_send_string(const char *str) {
    while (*str) {
        uart_send_char(*str++);
    }
}