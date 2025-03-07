#include "stm32f0xx.h"
#include "rfm9x.h"
#include "support.h"
#include "uart.h"
#include "debug.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RFM9X_NSS_PIN      (1 << 4)  // PA4 (CS)
#define RFM9X_SCK_PIN      (1 << 5)  // PA5 (Clock)
#define RFM9X_MISO_PIN     (1 << 6)  // PA6 (MISO)
#define RFM9X_MOSI_PIN     (1 << 7)  // PA7 (MOSI)
#define RFM9X_G0_PIN       (1 << 8)  // PA8 (Interrupt)
#define RFM9X_RESET_PIN    (1 << 0)  // PB0 (RESET)

//===========================================================================
// Configure GPIO and SPI1 for RFM9X LoRa
//===========================================================================
void init_spi1_lora(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;  //Enable SPI1 clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;   //Enable GPIOA clock

    // Configure PA4 (NSS), PA5 (SCK), PA6 (MISO), PA7 (MOSI) for SPI1
    GPIOA->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); //AF mode for SCK, MISO, MOSI
    GPIOA->MODER |= GPIO_MODER_MODER4_0;  //NSS as output (manual control)

    GPIOA->AFR[0] &= ~(0xF << (4 * 4) | 0xF << (5 * 4) | 0xF << (6 * 4) | 0xF << (7 * 4)); //AF0 for SPI1

    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5 | GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7);
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR6_0;  // Enable pull-up on MISO

    GPIOA->ODR |= GPIO_ODR_4;  //NSS HIGH (inactive)

    // Configure SPI1
    SPI1->CR1 &= ~SPI_CR1_SPE;  // Disable SPI1 before configuring
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI; // Master mode, software NSS

    SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA); // Ensure CPOL=0, CPHA=0 (SPI Mode 0)
    
    SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1; // Baud rate prescaler (adjust as needed)
    
    SPI1->CR2 = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; // 8-bit data size
    SPI1->CR1 |= SPI_CR1_SPE;  // Enable SPI1

    uart_send_string("DEBUG: SPI1 Configured for Mode 0 (CPOL=0, CPHA=0)\r\n");
}



//===========================================================================
// Reset RFM9X Module
//===========================================================================
void rfm9x_reset(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;  
    GPIOB->MODER |= GPIO_MODER_MODER0_0;  
    GPIOB->ODR &= ~GPIO_ODR_0; //RESET Low
    nano_wait(50000000); 
    GPIOB->ODR |= GPIO_ODR_0; //RESET High
    nano_wait(50000000); 
}

//===========================================================================
// Set NSS (Chip Select) Low or High
//===========================================================================
void rfm9x_nss_select(void) {
    uart_send_string("DEBUG: Forcing NSS LOW\r\n");
    GPIOA->ODR &= ~RFM9X_NSS_PIN; // Set NSS Low (Select RFM9X)
    
    // Verify NSS actually changed
    char buf[50];
    sprintf(buf, "CHECK NSS AFTER LOW: %d\r\n", (GPIOA->ODR & RFM9X_NSS_PIN) ? 1 : 0);
    uart_send_string(buf);

    nano_wait(1000); // Small delay
}

void rfm9x_nss_deselect(void) {
    nano_wait(1000); // Small delay before NSS goes HIGH
    GPIOA->ODR |= RFM9X_NSS_PIN;  // Set NSS High (Deselect RFM9X)
    
    // Verify NSS actually changed
    char buf[50];
    sprintf(buf, "CHECK NSS AFTER HIGH: %d\r\n", (GPIOA->ODR & RFM9X_NSS_PIN) ? 1 : 0);
    uart_send_string(buf);
}


//===========================================================================
// Send and receive one byte over SPI
//===========================================================================
uint8_t spi1_lora_transfer(uint8_t data) {
    char buf[50];

    // Check NSS before SPI transfer
    sprintf(buf, "Before SPI Transfer, NSS: %d\r\n", (GPIOA->ODR & RFM9X_NSS_PIN) ? 1 : 0);
    uart_send_string(buf);

    GPIOC->ODR |= GPIO_ODR_8;  //PC8 ON (Indicating SPI Start)

    //wait until TX buffer is empty
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = data;
    uart_send_string("TXE ready!\r\n");

    // **DEBUG**: Toggle PC8 every time SCK toggles
    for (int i = 0; i < 8; i++) { 
        GPIOC->ODR ^= GPIO_ODR_8;  // Toggle PC8
        nano_wait(5000);  // Small delay to make toggling visible
    }

    //wait until RX buffer has received data
    while (!(SPI1->SR & SPI_SR_RXNE));
    uint8_t received = SPI1->DR;

    GPIOC->ODR &= ~GPIO_ODR_8;  // PC8 OFF

    // Check NSS after SPI transfer
    sprintf(buf, "After SPI Transfer, NSS: %d\r\n", (GPIOA->ODR & RFM9X_NSS_PIN) ? 1 : 0);
    uart_send_string(buf);

    return received;
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
    nano_wait(500);
    spi1_lora_transfer(reg & 0x7F); //clear MSB for read operation
    uint8_t value = spi1_lora_transfer(0x00); //dummy byte and receive data
    nano_wait(500);
    rfm9x_nss_deselect();

    char buf[50];
    sprintf(buf, "Reg 0x%02X = 0x%02X\r\n", reg, value);
    uart_send_string(buf);

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
    nano_wait(100000);

    //ex setup (for nown)
    rfm9x_write_register(0x01, 0x81); //set to LoRa mode
    rfm9x_write_register(0x06, 0x6C); //set freq
    rfm9x_write_register(0x09, 0xFF); //max power output
}


//===========================================================================
// LOOPBACK TESTS
//===========================================================================
void test_rfm9x_registers() {
    uart_send_string("Testing SPI Communication with RFM9X...\r\n");

    for (uint8_t reg = 0x00; reg <= 0x7F; reg++) {  //read all 128 registers
        uint8_t value = rfm9x_read_register(reg);
        
        char buf[50];
        sprintf(buf, "Reg 0x%02X = 0x%02X\r\n", reg, value);
        uart_send_string(buf);

        nano_wait(100000);
    }
}

void test_spi_loopback(void) {
    uart_send_string("Starting Echo SPI Loopback...\r\n");

    uint8_t test_values[] = {0xAA, 0x55, 0xFF, 0x00, 0x42};
    for (int i = 0; i < sizeof(test_values); i++) {
        uint8_t sent = test_values[i];
        uint8_t received = spi1_lora_transfer(sent);

        char buf[50];
        sprintf(buf, "Sent: 0x%02X, Received: 0x%02X\r\n", sent, received);
        uart_send_string(buf);

        nano_wait(5000000);
    }

    uart_send_string("Basic SPI Loopback Test Complete!\r\n");
}

void check_miso_status(void) {
    char buf[50];
    sprintf(buf, "MISO (PA6) Before Transfer: %d\r\n", (GPIOA->IDR & (1 << 6)) ? 1 : 0);
    uart_send_string(buf);
}


void debug_spi_gpio(void) {
    char buf[100];
    
    // Read GPIO modes
    uint32_t moder = GPIOA->MODER;
    uint32_t afr0 = GPIOA->AFR[0];
    uint32_t pupdr = GPIOA->PUPDR;

    sprintf(buf, "GPIOA MODER: 0x%08lX\r\n", moder);
    uart_send_string(buf);

    sprintf(buf, "GPIOA AFR[0]: 0x%08lX\r\n", afr0);
    uart_send_string(buf);

    sprintf(buf, "GPIOA PUPDR: 0x%08lX\r\n", pupdr);
    uart_send_string(buf);
}

void test_sck_pin(void) {
    GPIOA->MODER &= ~GPIO_MODER_MODER5;
    GPIOA->MODER |= GPIO_MODER_MODER5_0;  

    uart_send_string("Toggling SCK (PA5)...\r\n");

    for (int i = 0; i < 10; i++) {
        GPIOA->ODR ^= (1 << 5); 
        nano_wait(500000);
    }

    uart_send_string("SCK (PA5) Toggle Test Done!\r\n");
}

void check_rfm9x_version() {
    uart_send_string("Checking RFM9X Version...\r\n");
    
    rfm9x_nss_select();
    uint8_t version = rfm9x_read_register(0x42);
    rfm9x_nss_deselect();

    char buf[50];
    sprintf(buf, "RFM9X Version Read: 0x%02X\r\n", version);
    uart_send_string(buf);

    if (version == 0x12) {
        uart_send_string("FM9X Detected Successfully!\r\n");
    } else {
        uart_send_string("ERROR: RFM9X Not Detected! Check SPI and Power.\r\n");
        while (1); //Halt if no detection
    }
}

