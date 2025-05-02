#ifndef HC12_H
#define HC12_H

void uart5_init(void);
void uart5_send_char(char c);
void uart5_send_string(const char *str);
void uart4_init(void);
void uart4_send_string(const char *str);
char usart4_receive_char(void);
int usart4_try_receive_char(char *c);


#endif
