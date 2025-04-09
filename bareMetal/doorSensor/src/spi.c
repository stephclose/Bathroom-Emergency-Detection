// spi.c
#include "stm32f0xx.h"
#include "spi.h"
#include <stdio.h>

void spi_init(void) {
    // Enable GPIOA and SPI1 clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Configure PA5 (SCK), PA6 (MISO), PA7 (MOSI) as alternate function
    GPIOA->MODER &= ~((3U << (5 * 2)) | (3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOA->MODER |=  ((2U << (5 * 2)) | (2U << (6 * 2)) | (2U << (7 * 2)));

    // Set AF0 for SPI1
    GPIOA->AFR[0] &= ~((0xF << (5 * 4)) | (0xF << (6 * 4)) | (0xF << (7 * 4)));

    // Reset SPI1 configuration
    SPI1->CR1 = 0;
    SPI1->CR2 = 0;

    // SPI in master mode, software NSS, Mode 0
    SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA); // Mode 0

    // Set baud rate prescaler to fPCLK/32
    SPI1->CR1 |= (SPI_CR1_BR_1 | SPI_CR1_BR_0); // fPCLK/32

    // 8-bit data size
    SPI1->CR2 = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;
    SPI1->CR2 &= ~SPI_CR2_FRF; // Motorola mode

    // Enable SPI
    SPI1->CR1 |= SPI_CR1_SPE;

    printf("[SPI INIT] SPI1 initialized for SX1276\r\n");
}

uint8_t spi_transfer(uint8_t data) 
{
    while (!(SPI1->SR & SPI_SR_TXE)); // Wait until TX is empty
    *((__IO uint8_t*)&SPI1->DR) = data;
    while (!(SPI1->SR & SPI_SR_RXNE)); // Wait until RX is full
    uint8_t received = *((__IO uint8_t*)&SPI1->DR);
    return received;
}


// #include "spi.h"
// #include "uart.h"
// #include <stdio.h>
// #include <string.h>

// void spi_init(void) 
// {
//     // Enable clocks for GPIOA and SPI1
//     RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
//     RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    
//     // Configure PA5, PA6, PA7 as alternate function mode for SPI1
//     GPIOA->MODER &= ~((3U << (5*2)) | (3U << (6*2)) | (3U << (7*2)));
//     GPIOA->MODER |=  ((2U << (5*2)) | (2U << (6*2)) | (2U << (7*2)));
//     GPIOA->AFR[0] &= ~((0xF << (5*4)) | (0xF << (6*4)) | (0xF << (7*4)));
    
//     // Disable SPI1 before configuring
//     SPI1->CR1 = 0;  
//     SPI1->CR1 &= ~SPI_CR1_SPE;
    
//     // Configure SPI1: master mode, software NSS management.
//     SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
//     SPI1->CR1 |= (SPI_CR1_BR_0 | SPI_CR1_BR_1); // 011 => /16

//     // Set baud rate (for example, prescaler = 64, adjust as needed)
//     SPI1->CR2 = (SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0);
//     // Set data size to 8 bits (using SPI_CR2)
//     SPI1->CR2 = (SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0);
    
//     // Enable SPI1.
//     SPI1->CR1 |= SPI_CR1_SPE;
    
//     printf("[SPI INIT] SPI1 initialized.\r\n");
// }

// // Transfers one byte over SPI and returns the received byte.
// uint8_t spi_transfer(uint8_t data) 
// {
//     while (!(SPI1->SR & SPI_SR_TXE)); // Wait until TX buffer is empty.
//     SPI1->DR = data;
//     while (!(SPI1->SR & SPI_SR_RXNE)); // Wait until RX buffer is not empty.
//     uint8_t received = (uint8_t)SPI1->DR;
//     printf("[SPI] Sent: 0x%02X, Received: 0x%02X\r\n", data, received);
//     return received;
// }


// /*
// // Initializes SPI1 for the RF module (PA5: SCK, PA6: MISO, PA7: MOSI)
// void spi_init(void) 
// {
//     // Enable clocks for GPIOA and SPI1
//     RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
//     RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    
//     // Configure PA5, PA6, PA7 as alternate function mode for SPI1
//     GPIOA->MODER &= ~((0x3 << (5 * 2)) | (0x3 << (6 * 2)) | (0x3 << (7 * 2)));
//     GPIOA->MODER |= ((0x2 << (5 * 2)) | (0x2 << (6 * 2)) | (0x2 << (7 * 2)));
    
//     // Set Alternate Function 0 (AF0) for SPI1 on PA5, PA6, PA7
//     GPIOA->AFR[0] &= ~((0xF << (5 * 4)) | (0xF << (6 * 4)) | (0xF << (7 * 4)));
    
//     // Disable SPI1 before configuring
//     SPI1->CR1 &= ~SPI_CR1_SPE;
    
//     // Configure SPI1: master mode, software NSS management
//     SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
//     // Set baud rate (example: prescaler settings, adjust as needed)
//     SPI1->CR1 |= (SPI_CR1_BR_2 | SPI_CR1_BR_1);
//     // Set 8-bit data size
//     SPI1->CR2 = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;
    
//     // Enable SPI1
//     SPI1->CR1 |= SPI_CR1_SPE;
    
//     printf("[SPI INIT] SPI1 initialized.\r\n");
// }

// // Transfers one byte over SPI and returns the received byte
// uint8_t spi_transfer(uint8_t data) 
// {
//     while (!(SPI1->SR & SPI_SR_TXE)); // wait for TX buffer empty
//     SPI1->DR = data;
//     while (!(SPI1->SR & SPI_SR_RXNE)); // wait for RX buffer not empty
//     uint8_t received = (uint8_t)SPI1->DR;
//     printf("[SPI] Sent: 0x%02X, Received: 0x%02X\r\n", data, received);
//     return received;
// }


// void spi_init(void) 
// {
//     RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // gpioa clock
//     RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // spi1 clock

//     // pa5 pa6 PA7 to alt functions 
//     GPIOA->MODER &= ~(0x3 << (5 * 2) | 0x3 << (6 * 2) | 0x3 << (7 * 2)); 
//     GPIOA->MODER |= (0x2 << (5 * 2)) | (0x2 << (6 * 2)) | (0x2 << (7 * 2)); 
//     GPIOA->AFR[0] &= ~((0xF << (5 * 4)) | (0xF << (6 * 4)) | (0xF << (7 * 4))); // alt func set for spi1
    
//     SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA); // Clear CPOL and CPHA bits → Mode 0

//     SPI1->CR1 &= ~SPI_CR1_BR;         // Clear baud rate bits
//     SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1;  // Set prescaler (this is an example; check your MCU datasheet)


//     SPI1->CR1 = SPI_CR1_MSTR | (SPI_CR1_BR_2) | SPI_CR1_SSI | SPI_CR1_SSM; // master, baud rate 8 bit 
//     SPI1->CR1 |= SPI_CR1_SPE; // spi enable
// }

// uint8_t spi_transfer(uint8_t data) // send recv one byte over spi 
// {
//     while (!(SPI1->SR & SPI_SR_TXE)); // tx buffer empty
//     SPI1->DR = data;                  // send
//     while (!(SPI1->SR & SPI_SR_RXNE)); // wait 
//     return (uint8_t)SPI1->DR;                  // recvd data returned
// }
// */