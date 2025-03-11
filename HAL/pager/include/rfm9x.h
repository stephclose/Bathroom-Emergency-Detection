#ifndef RFM9X_H
#define RFM9X_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h" 

extern SPI_HandleTypeDef hspi1; 

#define RFM9X_NSS_PORT      GPIOA
#define RFM9X_NSS_PIN       GPIO_PIN_4
#define RFM9X_RESET_PORT    GPIOB
#define RFM9X_RESET_PIN     GPIO_PIN_0
#define RFM9X_DIO0_PORT     GPIOA
#define RFM9X_DIO0_PIN      GPIO_PIN_8
#define RFM9X_FREQUENCY_433MHZ  433000000
#define RFM9X_FREQUENCY_868MHZ  868000000
#define RFM9X_FREQUENCY_915MHZ  915000000

void RFM9x_Init(void);
void RFM9x_Reset(void);
void RFM9x_SetFrequency(uint32_t frequency);
void RFM9x_SetTxPower(int8_t power);
void RFM9x_SetMode(uint8_t mode);
void RFM9x_SendPacket(uint8_t *data, uint8_t length);
uint8_t RFM9x_ReceivePacket(uint8_t *buffer, uint8_t maxLength);
uint8_t RFM9x_ReadRegister(uint8_t reg);
void RFM9x_WriteRegister(uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif // RFM9X_H
