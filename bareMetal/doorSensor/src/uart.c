
#include "uart.h"

// tx pa2 rx pa3
void uart_init(void) 
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2))); // clear
    GPIOA->MODER |= (2 << (2 * 2)) | (2 << (3 * 2));    // alt func mode
    GPIOA->AFR[0] &= ~((0xF << (4 * 2)) | (0xF << (4 * 3))); // clear AF 
    GPIOA->AFR[0] |= (1 << (4 * 2)) | (1 << (4 * 3));   // set for USART2

    // 115200 baud rate 
    USART2->BRR = 48000000 / 115200; 
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE; // usart tx rx 
}

// send ONE character :/
void uart_send_char(char c) 
{
    while (!(USART2->ISR & USART_ISR_TXE)); // till buff empty 
    USART2->TDR = c;
}

// send string 
void uart_send_string(const char* str) 
{
    while (*str) 
    {
        uart_send_char(*str++);
    }
}

void print_usart_status(void) 
{
    char buffer[64];
    //sprintf(buffer, "USART ISR Status: 0x%08lX\r\n", USART2->ISR);
    uart_send_string(buffer);
}