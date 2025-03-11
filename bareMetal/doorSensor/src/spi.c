#include "spi.h"

void spi_init(void) 
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // gpioa clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // spi1 clock

    // pa5 pa6 PA7 to alt functions 
    GPIOA->MODER &= ~(0x3 << (5 * 2) | 0x3 << (6 * 2) | 0x3 << (7 * 2)); 
    GPIOA->MODER |= (0x2 << (5 * 2)) | (0x2 << (6 * 2)) | (0x2 << (7 * 2)); 
    GPIOA->AFR[0] |= (0x0 << (5 * 4)) | (0x0 << (6 * 4)) | (0x0 << (7 * 4)); 

    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_1 | SPI_CR1_SSI | SPI_CR1_SSM; // master, baud rate 8 bit 
    SPI1->CR1 |= SPI_CR1_SPE; // spi enable
}

uint8_t spi_transfer(uint8_t data) 
{
    while (!(SPI1->SR & SPI_SR_TXE)); // tx buffer empty
    SPI1->DR = data;                  // send
    while (!(SPI1->SR & SPI_SR_RXNE)); // wait 
    return SPI1->DR;                  // recvd data 
}
