#ifndef UART_H
#define UART_H
#include "stm32f0xx.h"

void uart_init(void);
void uart_send_char(char c);
void uart_send_string(const char* str);
void print_usart_status(void);

#endif 
