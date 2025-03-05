#include "stm32f0xx.h"
#include "rfm9x.h"
#include "support.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RFM9X_NSS_PIN      GPIO_ODR_4  // PA4 (CS)
#define RFM9X_SCK_PIN      GPIO_ODR_5  // PA5 (Clock)
#define RFM9X_MISO_PIN     GPIO_ODR_6  // PA6 (MISO)
#define RFM9X_MOSI_PIN     GPIO_ODR_7  // PA7 (MOSI)
#define RFM9X_G0_PIN       GPIO_ODR_8  // PA8 (Interrupt)
#define RFM9X_RESET_PIN    GPIO_ODR_9  // PA0 (RESET)

//===========================================================================
// Configure GPIO and SPI1 for RFM9X LoRa
//===========================================================================
void init_spi1_lora(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //enable SPI1 clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //enable GPIOA clock

    //PA4 (NSS), PA5 (SCK), PA6 (MISO), PA7 (MOSI) as alternate function
    GPIOA->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); //AF mode

    GPIOA->AFR[0] &= ~(0xF << (4 * 4) | 0xF << (5 * 4) | 0xF << (6 * 4) | 0xF << (7 * 4)); //AF0 for SPI1

    //PA0 (RESET) and PA8 (DIO0) as output/input
    GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER8);
    GPIOA->MODER |= GPIO_MODER_MODER0_0; //PA0 as output
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR8;  //PA8 as input (LoRa interrupt)

    //SPI1 in Master mode, software NSS
    SPI1->CR1 &= ~SPI_CR1_SPE; //disable SPI before configuring
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI; //master mode, software NSS
    SPI1->CR1 |= SPI_CR1_BR; //set baud rate
    SPI1->CR2 = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; //8-bit data size
    SPI1->CR1 |= SPI_CR1_SPE; //rnable SPI
}

//===========================================================================
// Reset RFM9X Module
//===========================================================================
void rfm9x_reset(void) {
    GPIOA->ODR &= ~RFM9X_RESET_PIN; //RESET low
    nano_wait(100000);
    GPIOA->ODR |= RFM9X_RESET_PIN; //RESET high
    nano_wait(500000); 
}

//===========================================================================
// Set NSS (Chip Select) Low or High
//===========================================================================
void rfm9x_nss_select(void) {
    GPIOA->ODR &= ~RFM9X_NSS_PIN; //NSS Low. select RFM9X
    nano_wait(100);
}

void rfm9x_nss_deselect(void) {
    nano_wait(100);
    GPIOA->ODR |= RFM9X_NSS_PIN;  //NSS High. deselect RFM9X
}


//===========================================================================
// Send and receive one byte over SPI
//===========================================================================
uint8_t spi1_lora_transfer(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE)); //transmit buffer empty
    SPI1->DR = data;  //send byte
    while (!(SPI1->SR & SPI_SR_RXNE)); //received byte
    return SPI1->DR; //read
}

//===========================================================================
// Write to RFM9X Register
//===========================================================================
void rfm9x_write_register(uint8_t reg, uint8_t value) {
    rfm9x_nss_select();
    spi1_lora_transfer(reg | 0x80);  //set MSB, show write operation
    spi1_lora_transfer(value);
    rfm9x_nss_deselect();
}

//===========================================================================
// Read from RFM9X Register
//===========================================================================
uint8_t rfm9x_read_register(uint8_t reg) {
    rfm9x_nss_select();
    spi1_lora_transfer(reg & 0x7F); //clear MSB for read operation
    uint8_t value = spi1_lora_transfer(0x00); //dummy byte and receive data
    rfm9x_nss_deselect();
    return value;
}

//===========================================================================
// Set Mode (Standby, Sleep, Transmit, Receive)
//===========================================================================
void rfm9x_set_mode(uint8_t mode) {
    rfm9x_write_register(0x01, mode);
}

//===========================================================================
// Set Frequency
//===========================================================================
void rfm9x_set_frequency(uint32_t freq) {
    uint32_t frf = (freq * 1000000) / 61.035; //convert MHz to FRF register value
    rfm9x_write_register(0x06, (frf >> 16) & 0xFF); //MSB
    rfm9x_write_register(0x07, (frf >> 8) & 0xFF); //MID
    rfm9x_write_register(0x08, frf & 0xFF); //LSB
}

//===========================================================================
// Set TX Power
//===========================================================================
void rfm9x_set_tx_power(uint8_t power) {
    if (power > 20) power = 20;  //max power is 20dBm
    rfm9x_write_register(0x09, power);
}

//===========================================================================
// Send Packet (LoRa Mode)
//===========================================================================
void rfm9x_send_packet(uint8_t *data, uint8_t length) {
    rfm9x_set_mode(0x01);

    rfm9x_write_register(0x0D, 0x00);  //set FIFO pointer
    for (uint8_t i = 0; i < length; i++) {
        rfm9x_write_register(0x00, data[i]);  //write data to FIFO
    }

    rfm9x_write_register(0x22, length);  //set payload length
    rfm9x_set_mode(0x83);  //transmit mode

    while (!(rfm9x_read_register(0x12) & 0x08));  //wait for TX done
    rfm9x_write_register(0x12, 0x08);  //clear IRQ flag
}

//===========================================================================
// Receive Packet (LoRa Mode)
//===========================================================================
uint8_t rfm9x_receive_packet(uint8_t *buffer, uint8_t max_length) {
    rfm9x_set_mode(0x05); //RX continuous mode

    while (!(rfm9x_read_register(0x12) & 0x40)); //wait for RX done
    rfm9x_write_register(0x12, 0x40); //clear IRQ flag

    uint8_t length = rfm9x_read_register(0x13); //payload len
    if (length > max_length) length = max_length;

    rfm9x_write_register(0x0D, 0x00); //FIFO pointer
    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = rfm9x_read_register(0x00); // read FIFO
    }

    return length;
}

//===========================================================================
// Initialize RFM9X LoRa Module
//===========================================================================
void rfm9x_init(void) {
    nano_wait(100000);  //allow time for module to start up

    //ex setup (for nown)
    rfm9x_write_register(0x01, 0x81); //set to LoRa mode
    rfm9x_write_register(0x06, 0x6C); //set freq
    rfm9x_write_register(0x09, 0xFF); //max power output
}

