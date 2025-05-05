#include "stm32f0xx.h"
#include "hc12.h"
#include "clock.h"
#include "uart.h"
#include "support.h"

void uart5_init(void) {
    // Enable clocks for GPIOC, GPIOD, and USART5
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;

    GPIOC->MODER |= (1 << (6 * 2));

    GPIOC->ODR ^= (1 << 6);
    nano_wait(1000000);
  

    GPIOC->MODER &= ~(3 << (12 * 2));
    GPIOC->MODER |=  (2 << (12 * 2));  // AF mode
    GPIOC->AFR[1] &= ~(0xF << ((12 - 8) * 4));
    GPIOC->AFR[1] |=  (2 << ((12 - 8) * 4));  // AF2 for USART5_TX

    GPIOD->MODER &= ~(3 << (2 * 2));
    GPIOD->MODER |=  (2 << (2 * 2));  // AF mode
    GPIOD->AFR[0] &= ~(0xF << (2 * 4));
    GPIOD->AFR[0] |=  (2 << (2 * 4));  // AF2 for USART5_RX

    nano_wait(1000000);

    // Set baud rate for 115200 bps assuming 48 MHz clock
    USART5->BRR = 48000000 / 115200;
    nano_wait(1000000);
    USART5->CR1 = USART_CR1_TE | USART_CR1_UE;
    nano_wait(1000000);
}


void uart5_send_char(char c)
{
    while (!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = c;
}

void uart5_send_string(const char *str)
{
    while (*str)
        uart5_send_char(*str++);
}

void usart4_init(void) {
    // 1. Enable GPIOC and USART4 clocks
    RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART4EN;

    // 2. Set PC10 and PC11 to Alternate Function mode (AF1 = USART4)
    GPIOC->MODER &= ~((3 << (10 * 2)) | (3 << (11 * 2)));     // Clear MODER
    GPIOC->MODER |=  (2 << (10 * 2)) | (2 << (11 * 2));       // Set to AF mode

    GPIOC->AFR[1] &= ~((0xF << ((10 - 8) * 4)) | (0xF << ((11 - 8) * 4))); // Clear AFR
    GPIOC->AFR[1] |=  (1 << ((10 - 8) * 4)) | (1 << ((11 - 8) * 4));       // Set AF1

    // 3. Configure USART4 (9600 baud, 8N1)
    USART4->BRR = 48000000 / 9600; // Assuming 48 MHz sysclk
    USART4->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}


void uart4_send_char(char c) {
    while (!(USART4->ISR & USART_ISR_TXE));
    USART4->TDR = c;
}

void uart4_send_string(const char *str) {
    while (*str)
        uart4_send_char(*str++);
}

char usart4_receive_char(void) {
    while (!(USART4->ISR & USART_ISR_RXNE));
    return USART4->RDR;
}

int usart4_try_receive_char(char *c) {
    if (USART4->ISR & USART_ISR_RXNE) {
        *c = USART4->RDR;
        return 1;
    }
    return 0;
}