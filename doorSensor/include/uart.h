#ifndef UART_H
#define UART_H

#include "stm32f091xc.h"
#include <stdio.h>

// Initialize UART (USART1 on PA9 and PA10)
void uart_init(void);

// Redirect printf() output to UART
int _write(int file, char *ptr, int len);

#endif // UART_H
