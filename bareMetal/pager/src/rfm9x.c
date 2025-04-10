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
// Initialize SPI and GPIOS
//===========================================================================
void init_spi1_lora(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //enable SPI1 clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //enable GPIOA clock

    // -- PA4: GPIO output mode for NSS
    GPIOA->MODER &= ~GPIO_MODER_MODER4;
    GPIOA->MODER |= GPIO_MODER_MODER4_0;

    GPIOA->ODR |= (1 << 4);

    // -- PA5–PA7: AF mode for SPI1
    GPIOA->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |=  (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); // AF

    // -- Set AF0 for SPI1 on PA5–PA7 (AFR[0])
    GPIOA->AFR[0] &= ~((0xF << (5 * 4)) | (0xF << (6 * 4)) | (0xF << (7 * 4)));

    //Configure SPI1 (Mode 0, 8-bit, Master)
    SPI1->CR1 &= ~SPI_CR1_SPE;
    
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;  //MM, software NSS
    SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);
    SPI1->CR1 |= SPI_CR1_BR; //lowest baud rate 

    SPI1->CR2 |= SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;
    SPI1->CR2 &= ~SPI_CR2_DS_3; //8-bit transmission
    SPI1->CR1 |= SPI_CR1_SPE;  

    uart_send_string("SPI1 Enabled and Configured in Mode 0\r\n");
}

void test_rfm9x_write_read_frf_registers() {
    uart_send_string("Testing manual FRF write...\r\n");

    rfm9x_set_mode(0x81); //standby
    nano_wait(10000);

    //manually write FRF: 0xE4 0x00 0xC0 = 915 MHz
    rfm9x_write_register(0x06, 0xE4);
    nano_wait(1000);
    rfm9x_write_register(0x07, 0x00);
    nano_wait(1000);
    rfm9x_write_register(0x08, 0xC0);
    nano_wait(1000);

    uint8_t msb = rfm9x_read_register(0x06);
    nano_wait(500);
    uint8_t mid = rfm9x_read_register(0x07);
    nano_wait(500);
    uint8_t lsb = rfm9x_read_register(0x08);

    char buf[64];
    sprintf(buf, "Confirm FRF: 0x%02X 0x%02X 0x%02X\r\n", msb, mid, lsb);
    uart_send_string(buf);
}

void rfm9x_enter_lora_mode(void) {
    uint8_t opmode = rfm9x_read_register(0x01);
    if (!(opmode & 0x80)) {
        rfm9x_write_register(0x01, opmode | 0x80); //set LoRa mode bit
        nano_wait(10000);
    }
}

//===========================================================================
// Reset RFM9X Module
//===========================================================================
void rfm9x_reset(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;  
    GPIOB->MODER &= ~GPIO_MODER_MODER0;  //clear mode
    GPIOB->MODER |= GPIO_MODER_MODER0_0; //set PB0 as output

    GPIOB->ODR &= ~GPIO_ODR_0; //RESET Low
    nano_wait(50000000); 
    GPIOB->ODR |= GPIO_ODR_0; //RESET High
    nano_wait(50000000); 
}

//===========================================================================
// NSS (Chip Select) Control Functions
//===========================================================================
void rfm9x_nss_select(void) {
    GPIOA->ODR &= ~GPIO_ODR_4;  // NSS LOW
}

void rfm9x_nss_deselect(void) {
    GPIOA->ODR |= GPIO_ODR_4;   // NSS HIGH
}

//===========================================================================
// Send and receive one byte over SPI
//===========================================================================
uint8_t spi1_lora_transfer(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE)); //wait until TX buffer is empty
    SPI1->DR = data;

    while (!(SPI1->SR & SPI_SR_RXNE));
    
    uint8_t result = SPI1->DR;

    (void) SPI1->SR;
    (void) SPI1->DR;

    return result;
    
}

//===========================================================================
// Write to RFM9X Register
//===========================================================================
void rfm9x_write_register(uint8_t reg, uint8_t value) {
    rfm9x_nss_select();
    nano_wait(100);
    spi1_lora_transfer(reg | 0x80);  //set MSB, show write operation
    nano_wait(100);
    spi1_lora_transfer(value);
    nano_wait(100);
    rfm9x_nss_deselect();
}

//===========================================================================
// Read from RFM9X Register
//===========================================================================
uint8_t rfm9x_read_register(uint8_t reg) {
    rfm9x_nss_select();
    spi1_lora_transfer((reg) & 0x7F);
    uint8_t value = spi1_lora_transfer(0xff); //dummy byte and receive data
    nano_wait(500);
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
void rfm9x_set_frequency(uint32_t freq_mhz) {

    rfm9x_set_mode(0x81); //standby
    nano_wait(10000);

    uint64_t frf = ((uint64_t)freq_mhz * 1000000ULL << 19) / 32000000ULL;
    uint8_t msb = (frf >> 16) & 0xFF;
    uint8_t mid = (frf >> 8) & 0xFF;
    uint8_t lsb = frf & 0xFF;

    // Write each register with delay
    rfm9x_write_register(0x06, msb);
    nano_wait(1000);

    uint8_t verify_msb = rfm9x_read_register(0x06);
    char bufff[64];
    sprintf(bufff, "Written MSB: 0x%02X, Readback MSB: 0x%02X\r\n", msb, verify_msb);
    uart_send_string(bufff);

    rfm9x_write_register(0x07, mid);
    nano_wait(1000);
    rfm9x_write_register(0x08, lsb);
    nano_wait(1000);

    // Confirm write
    char buf[64];
    sprintf(buf, "FRF Written: %02X %02X %02X\r\n", msb, mid, lsb);
    uart_send_string(buf);

    uint8_t read_msb = rfm9x_read_register(0x06);
    uint8_t read_mid = rfm9x_read_register(0x07);
    uint8_t read_lsb = rfm9x_read_register(0x08);

    sprintf(buf, "FRF Readback: %02X %02X %02X\r\n", read_msb, read_mid, read_lsb);
    uart_send_string(buf);
}


//===========================================================================
// Set TX Power
//===========================================================================
void rfm9x_set_tx_power(uint8_t power) {
    if (power > 20) power = 20;  //max power is 20dBm
    rfm9x_write_register(0x09, power);
    
    if (power > 17) {
        rfm9x_write_register(0x09, 0x8F); //PA_BOOST + High Power
        rfm9x_write_register(0x4D, 0x87); //RegPaDac = High Power Mode
    } else { 
        rfm9x_write_register(0x09, 0x80 | (power - 2)); //PA_BOOST
        rfm9x_write_register(0x4D, 0x84); //RegPaDac = Normal Mode
    }
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
// Setup TX Transmission (LoRa Mode)
//===========================================================================
void rfm9x_transmit_message(const char* message) {
    uint8_t len = strlen(message);
    if (len > 255) len = 255; //max payload limit

    rfm9x_set_mode(0x01); //SLEEP
    nano_wait(10000);

    rfm9x_set_mode(0x82); //standby
    rfm9x_write_register(0x0D, 0x00); //FIFO pointer to base

    for (uint8_t i = 0; i < len; i++) {
        rfm9x_write_register(0x00, message[i]);
    }

    rfm9x_write_register(0x22, len); //payload len
    rfm9x_set_mode(0x83); //TX mode

    uart_send_string("Transmitting...\r\n");

    while (!(rfm9x_read_register(0x12) & 0x08)); 
    rfm9x_write_register(0x12, 0x08); //clear

    uart_send_string("Transmit complete!\r\n");
    rfm9x_set_mode(0x82);
}

//===========================================================================
// Setup RX Transmission (LoRa Mode)
//===========================================================================

uint8_t rfm9x_try_receive_message(char* buffer, uint8_t max_len) {
    uint8_t irq_flags = rfm9x_read_register(0x12);

    if (irq_flags & 0x40) {
        rfm9x_write_register(0x12, 0xFF); //clear all IRQs

        uint8_t len = rfm9x_read_register(0x13);
        if (len > max_len - 1) len = max_len - 1;

        rfm9x_write_register(0x0D, 0x00); //FIFO address to current RX base
        for (uint8_t i = 0; i < len; i++) {
            buffer[i] = rfm9x_read_register(0x00);
        }

        buffer[len] = '\0'; //null-terminate for printing
        return len;
    }

    return 0; //nothing
}


//===========================================================================
// TESTING FUNCTIONS
//===========================================================================

void rfm9x_print_tx_config(uint8_t expected_payload_len) {
    char buf[64];

    uint8_t opmode = rfm9x_read_register(0x01);
    uint8_t paconfig = rfm9x_read_register(0x09);
    uint8_t frf_msb = rfm9x_read_register(0x06);
    uint8_t frf_mid = rfm9x_read_register(0x07);
    uint8_t frf_lsb = rfm9x_read_register(0x08);
    uint8_t fifo_addr = rfm9x_read_register(0x0D);
    uint8_t payload_len = rfm9x_read_register(0x22);
    uint8_t irq_flags = rfm9x_read_register(0x12);
    uint8_t version = rfm9x_read_register(0x42);

    sprintf(buf, "TX CONFIG CHECK:\r\n");
    uart_send_string(buf);
    sprintf(buf, "  RegOpMode         (0x01) = 0x%02X\r\n", opmode);
    uart_send_string(buf);
    sprintf(buf, "  RegPaConfig       (0x09) = 0x%02X\r\n", paconfig);
    uart_send_string(buf);
    sprintf(buf, "  RegFrf:           %02X %02X %02X\r\n", frf_msb, frf_mid, frf_lsb);
    uart_send_string(buf);
    sprintf(buf, "  RegFifoAddrPtr    (0x0D) = 0x%02X\r\n", fifo_addr);
    uart_send_string(buf);
    sprintf(buf, "  RegPayloadLength  (0x22) = 0x%02X (expected %d)\r\n", payload_len, expected_payload_len);
    uart_send_string(buf);
    sprintf(buf, "  RegIrqFlags       (0x12) = 0x%02X\r\n", irq_flags);
    uart_send_string(buf);
    sprintf(buf, "  RegVersion        (0x42) = 0x%02X\r\n", version);
    uart_send_string(buf);
}

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
    char buf[200];
    sprintf(buf, "GPIOA MODER: 0x%08lX\r\n", GPIOA->MODER);
    uart_send_string(buf);
    sprintf(buf, "GPIOA AFR[0]: 0x%08lX\r\n", GPIOA->AFR[0]);
    uart_send_string(buf);
    sprintf(buf, "GPIOA PUPDR: 0x%08lX\r\n", GPIOA->PUPDR);
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

void test_spi_manual_transfer(void) {
    uart_send_string("Manually Sending SPI Data...\r\n");

    rfm9x_nss_select();

    SPI1->DR = 0xAA;  //Send 0xAA

    while (!(SPI1->SR & SPI_SR_TXE));
    uart_send_string("TXE Set\r\n");

    while (!(SPI1->SR & SPI_SR_RXNE));
    uint8_t received = SPI1->DR;

    rfm9x_nss_deselect();

    char buf[50];
    sprintf(buf, "Received: 0x%02X\r\n", received);
    uart_send_string(buf);
}

void test_rfm9x_basic_communication() {
    uart_send_string("Testing RFM9X Communication...\r\n");

    uint8_t version = rfm9x_read_register(0x42); //Version register

    char buf[50];
    sprintf(buf, "Version Register (0x42) = 0x%02X\r\n", version);
    uart_send_string(buf);

    if (version == 0x12) {
        uart_send_string("RFM9X Initialized Successfully!\r\n");
    }
}

void fake_rx_custom_message(const char* message) {
    uint8_t len = strlen(message);
    rfm9x_set_mode(0x01);
    rfm9x_write_register(0x0D, 0x00);

    for (uint8_t i = 0; i < len; i++) {
        rfm9x_write_register(0x00, message[i]);
    }

    rfm9x_write_register(0x13, len);  //RegRxNbBytes
    rfm9x_write_register(0x10, 0x00);  //RegFifoRxCurrentAddr

    //manually simulate interrupt
    rfm9x_write_register(0x12, 0x40);

    uart_send_string("Reading back message from FIFO...\r\n");
    rfm9x_set_mode(0x05);
    uint8_t buf[64];
    rfm9x_write_register(0x0D, 0x00);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = rfm9x_read_register(0x00);
    }
    buf[len] = '\0';

    uart_send_string("What I get back: ");
    uart_send_string((char*)buf);
    uart_send_string("\r\n");
}


void rfm9x_test_suite() {
    uart_send_string("\nRunning RFM9X Test Suite...\r\n");

    uart_send_string("Step 1: Resetting RFM9X...\r\n");
    rfm9x_reset();
    uart_send_string("Reset Done\r\n");

    uart_send_string("Step 2: Initializing SPI1...\r\n");
    init_spi1_lora();
    uart_send_string("SPI1 Init Done\r\n");

    uart_send_string("Step 3: Verifying GPIO Config...\r\n");
    debug_spi_gpio();
    uart_send_string("GPIO Debug Done\r\n");

    uart_send_string("Step 4: Toggling SCK Pin...\r\n");
    test_sck_pin();
    uart_send_string("SCK Pin Test Done\r\n");

    uart_send_string("Step 5: Checking MISO Line...\r\n");
    check_miso_status();
    uart_send_string("MISO Status Check Done\r\n");

    uart_send_string("Step 6: SPI Loopback Test...\r\n");
    test_spi_loopback();
    uart_send_string("SPI Loopback Done\r\n");

    uart_send_string("Step 7: Manual Transfer Test...\r\n");
    test_spi_manual_transfer();
    uart_send_string("Manual Transfer Done\r\n");

    uart_send_string("Step 8: Reading Version Register (0x42)...\r\n");
    uint8_t version = rfm9x_read_register(0x42);

    char buf[50];
    sprintf(buf, "Version Register: 0x%02X\r\n", version);
    uart_send_string(buf);

    if (version == 0x12) {
        uart_send_string("RFM9X Communication OK!\r\n");
    } else {
        uart_send_string("ERROR: Version Read Failed. SPI or Wiring?\r\n");
    }

    uart_send_string("RFM9X Test Suite Complete!\r\n\n");
}


