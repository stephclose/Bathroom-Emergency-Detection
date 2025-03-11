#ifndef UART_H
#define UART_H

#include "stm32f0xx_hal.h"

extern UART_HandleTypeDef huart2;

#define RX_BUFFER_SIZE 64
extern uint8_t rx_buffer[RX_BUFFER_SIZE];

void uart_init(void);
void uart_send_char(char c);
void uart_send_string(const char *str);
char uart_receive_char(void);
void USART2_IRQHandler(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_MspInit(UART_HandleTypeDef *huart);

#endif // UART_H
