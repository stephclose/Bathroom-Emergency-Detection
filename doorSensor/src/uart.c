#include "stm32f091xc.h"
#include <stdio.h>

void uart_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // Enable USART1 clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;     // Enable GPIOA clock

    // PA9 = TX, PA10 = RX (AF1)
    GPIOA->MODER |= (0x2 << (9 * 2)) | (0x2 << (10 * 2)); // Alternate function
    GPIOA->AFR[1] |= (0x1 << (1 * 4)) | (0x1 << (2 * 4)); // Set AF1

    USART1->BRR = 48000000 / 115200;  // Baud rate (48 MHz / 115200)
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable TX, RX, USART
}

// Redirect printf() to UART (fixes newline for PuTTY)
int _write(int file, char *ptr, int len)
{
    for (int i = 0; i < len; i++) {
        while (!(USART1->ISR & USART_ISR_TXE));  // Wait for TX buffer
        USART1->TDR = ptr[i];  // Send character
        if (ptr[i] == '\n') {  // Fix newline issue for PuTTY
            while (!(USART1->ISR & USART_ISR_TXE));
            USART1->TDR = '\r';
        }
    }
    return len;
}

