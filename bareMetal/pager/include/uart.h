#ifndef USART_H
#define USART_H

void uart_init(void);
void uart_send_char(char c);
void uart_send_string(const char *str);

void debug_uart(void);
int _write(int file, char *ptr, int len);

#endif
