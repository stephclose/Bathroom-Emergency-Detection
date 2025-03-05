#ifndef USART_H
#define USART_H

void usart_init(void);
void uart_send_char(char c);
void uart_send_string(const char *str);

#endif
