#ifndef RF_H
#define RF_H

#include "stm32f0xx.h"

void rf_init(void);
void rf_send(uint8_t *data, uint8_t len);
uint8_t rf_receive(uint8_t *buffer);
void rf_transmit(const char *msg);


#endif
