#include "stm32f0xx_hal.h"
#include "rfm9x.h"
#include "error_handler.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RFM9X_NSS_PORT     GPIOA
#define RFM9X_NSS_PIN      GPIO_PIN_4  //NSS (CS)
#define RFM9X_SCK_PORT     GPIOA
#define RFM9X_SCK_PIN      GPIO_PIN_5  //SCK
#define RFM9X_MISO_PORT    GPIOA
#define RFM9X_MISO_PIN     GPIO_PIN_6  //MISO
#define RFM9X_MOSI_PORT    GPIOA
#define RFM9X_MOSI_PIN     GPIO_PIN_7  //MOSI
#define RFM9X_RESET_PORT   GPIOB
#define RFM9X_RESET_PIN    GPIO_PIN_0  //RESET

SPI_HandleTypeDef hspi1;

void MX_SPI1_Init(void) {
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        //Error_Handler();
    }
}

void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    //NSS Pin
    GPIO_InitStruct.Pin = RFM9X_NSS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RFM9X_NSS_PORT, &GPIO_InitStruct);

    //RESET Pin
    GPIO_InitStruct.Pin = RFM9X_RESET_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RFM9X_RESET_PORT, &GPIO_InitStruct);

    //NSS High (deselect)
    HAL_GPIO_WritePin(RFM9X_NSS_PORT, RFM9X_NSS_PIN, GPIO_PIN_SET);
}

void RFM9x_Select(void) {
    HAL_GPIO_WritePin(RFM9X_NSS_PORT, RFM9X_NSS_PIN, GPIO_PIN_RESET);
}

void RFM9x_Deselect(void) {
    HAL_GPIO_WritePin(RFM9X_NSS_PORT, RFM9X_NSS_PIN, GPIO_PIN_SET);
}

void RFM9x_Reset(void) {
    HAL_GPIO_WritePin(RFM9X_RESET_PORT, RFM9X_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(RFM9X_RESET_PORT, RFM9X_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
}

uint8_t RFM9x_Transfer(uint8_t data) {
    uint8_t receivedData = 0;
    if (HAL_SPI_TransmitReceive(&hspi1, &data, &receivedData, 1, HAL_MAX_DELAY) != HAL_OK) {
        //Error_Handler();
    }
    return receivedData;
}

void RFM9x_WriteRegister(uint8_t reg, uint8_t value) {
    RFM9x_Select();
    RFM9x_Transfer(reg | 0x80);
    RFM9x_Transfer(value);
    RFM9x_Deselect();
}

uint8_t RFM9x_ReadRegister(uint8_t reg) {
    uint8_t value;
    RFM9x_Select();
    RFM9x_Transfer(reg & 0x7F);
    value = RFM9x_Transfer(0x00);
    RFM9x_Deselect();
    return value;
}

void RFM9x_SetMode(uint8_t mode) {
    RFM9x_WriteRegister(0x01, mode);
}


