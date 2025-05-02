#include "stm32f0xx.h"
#include "rfm9x.h"
#include "support.h"
#include "clock.h"
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

#define ENABLE_SPI1_CLOCK()     RCC->APB2ENR |= RCC_APB2ENR_SPI1EN
#define ENABLE_GPIOA_CLOCK()    RCC->AHBENR |= RCC_AHBENR_GPIOAEN
#define ENABLE_GPIOB_CLOCK()    RCC->AHBENR |= RCC_AHBENR_GPIOBEN

//===========================================================================
// Initialize SPI and GPIOS
//===========================================================================
void init_spi1_lora(void) {
    ENABLE_SPI1_CLOCK();
    ENABLE_GPIOA_CLOCK();

    //PA4: NSS
    GPIOA->MODER &= ~GPIO_MODER_MODER4;
    GPIOA->MODER |= GPIO_MODER_MODER4_0;
    GPIOA->ODR |= RFM9X_NSS_PIN;

    //PA5–PA7: AF SPI1
    GPIOA->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |=  (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); // AF
    GPIOA->AFR[0] &= ~((0xF << (5 * 4)) | (0xF << (6 * 4)) | (0xF << (7 * 4)));

    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);
    SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;  //MM, software NSS
    SPI1->CR1 |= SPI_CR1_BR; //lowest br
    SPI1->CR1 |= SPI_CR1_SPE;  

    //SPI1->CR2 |= SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;
    
    //SPI1->CR2 &= ~SPI_CR2_DS_3; //8-bit transmission

    SPI1->CR2 &= ~SPI_CR2_DS; // Clear data size bits
    SPI1->CR2 |= SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; // Set data size to 8 bits


}

//===========================================================================
// Reset RFM9X Module
//===========================================================================
void rfm9x_reset(void) {
    ENABLE_GPIOB_CLOCK();
    GPIOB->MODER &= ~GPIO_MODER_MODER0;
    GPIOB->MODER |= GPIO_MODER_MODER0_0;
    GPIOB->ODR &= ~GPIO_ODR_0; //RESET Low
    nano_wait(50000000); 
    GPIOB->ODR |= GPIO_ODR_0; //RESET High
    nano_wait(50000000); 
}

//===========================================================================
// NSS (Chip Select) Control Functions
//===========================================================================

void rfm9x_nss_select(void)   { GPIOA->ODR &= ~RFM9X_NSS_PIN; }
void rfm9x_nss_deselect(void) { GPIOA->ODR |=  RFM9X_NSS_PIN; }

void rfm9x_enter_lora_mode(void) {
    uint8_t opmode = rfm9x_read_register(0x01);
    if (!(opmode & 0x80)) {
        rfm9x_write_register(0x01, opmode | 0x80);
        nano_wait(10000);
    }
}

//===========================================================================
// Send and receive one byte over SPI
//===========================================================================
uint8_t spi1_lora_transfer(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE)); //wait until TX buffer is empty
    SPI1->DR = data;
    while (!(SPI1->SR & SPI_SR_RXNE));
    return SPI1->DR;
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
    nano_wait(1000); 
}

void rfm9x_write_registers(uint8_t start_reg, const uint8_t* values, uint8_t length) {
    rfm9x_nss_select();
    nano_wait(100);
    spi1_lora_transfer(start_reg | 0x80);  // Set MSB for write operation
    for (uint8_t i = 0; i < length; i++) {
        spi1_lora_transfer(values[i]);
    }
    nano_wait(100);
    rfm9x_nss_deselect();
    nano_wait(1000);
}


//===========================================================================
// Read from RFM9X Register
//===========================================================================
uint8_t rfm9x_read_register(uint8_t reg) {
    rfm9x_nss_select();
    nano_wait(100);
    spi1_lora_transfer(reg & 0x7F);
    nano_wait(50);
    uint8_t value = spi1_lora_transfer(0xff); //dummy byte and receive data
    nano_wait(100);
    rfm9x_nss_deselect();
    nano_wait(1000);
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
void rfm9x_set_frequency(uint32_t freq_hz) {
    char buf[128];
    float fstep = 61.03515625f; // 32 MHz / 2^19
    uint32_t frf = (uint32_t)(freq_hz / fstep);

    uint8_t msb = (frf >> 16) & 0xFF;
    uint8_t mid = (frf >> 8) & 0xFF;
    uint8_t lsb = frf & 0xFF;

    // Write FRF registers
    rfm9x_write_register(0x06, msb);
    nano_wait(1000);
    //rfm9x_write_register(0x07, mid);
    //rfm9x_write_register(0x08, lsb);

    // Read them back to verify
    uint8_t read_msb = rfm9x_read_register(0x06);
    uint8_t read_mid = rfm9x_read_register(0x07);
    uint8_t read_lsb = rfm9x_read_register(0x08);

    sprintf(buf, "Wrote FRF: %02X %02X %02X\r\n", msb, mid, lsb);
    uart_send_string(buf);
    sprintf(buf, "Read  FRF: %02X %02X %02X\r\n", read_msb, read_mid, read_lsb);
    uart_send_string(buf);

    uint32_t frf_readback = (read_msb << 16) | (read_mid << 8) | read_lsb;
    float actual_freq = frf_readback * fstep;
    sprintf(buf, "Actual Frequency: %.2f Hz\r\n", actual_freq);
    uart_send_string(buf);
}


//===========================================================================
// Set TX Power
//===========================================================================

void RFO_max_output(void) {
    rfm9x_nss_select();
    nano_wait(1000);
    rfm9x_write_register(0x09, 0xFF);
    rfm9x_write_register(0x4D, 0x87);
    rfm9x_nss_deselect();
    nano_wait(1000);
}

void PA_boost(void) { //NOT WORKING should return 0x8f in reg
    rfm9x_set_mode(0x81);
    nano_wait(10000);

    rfm9x_write_register(0x09, 0xFF); //RegPaConfig: PA_BOOST, MaxPower=7, OutputPower=15
    nano_wait(1000);

    rfm9x_write_register(0x4D, 0x07); //RegPaDac: Enable +20 dBm on PA_BOOST
    nano_wait(1000);

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
    if (len > 255) len = 255;

    rfm9x_set_mode(0x81); // Standby
    rfm9x_write_register(0x0D, 0x00); // FIFO pointer

    for (uint8_t i = 0; i < len; i++) {
        rfm9x_write_register(0x00, message[i]);
    }

    rfm9x_write_register(0x22, len); // Payload length
    rfm9x_set_mode(0x83); // Transmit

    while (!(rfm9x_read_register(0x12) & 0x08)); // Wait for TxDone
    rfm9x_write_register(0x12, 0x08); // Clear TxDone

    rfm9x_set_mode(0x81); // Return to Standby
}


//===========================================================================
// Setup RX Transmission (LoRa Mode)
//===========================================================================
uint8_t rfm9x_try_receive_message(char* buffer, uint8_t max_len) {
    // Ensure the radio is in RX Continuous mode
    rfm9x_set_mode(0x05); //RegOpMode = RXCONTINUOUS

    uint8_t irq_flags = rfm9x_read_register(0x12);

    if (irq_flags & 0x40) { //RxDone
        if (irq_flags & 0x20) { //PayloadCrcError
            uart_send_string("[RX] CRC error. Dropping packet.\r\n");
            rfm9x_write_register(0x12, 0xFF); // clear all IRQ flags
            return 0;
        }

        rfm9x_write_register(0x12, 0xFF); //clear all IRQs
        uint8_t len = rfm9x_read_register(0x13);
        if (len >= max_len) len = max_len - 1;
        uint8_t fifo_rx_current_addr = rfm9x_read_register(0x10);
        rfm9x_write_register(0x0D, fifo_rx_current_addr); //set FIFO pointer to current RX addr

        for (uint8_t i = 0; i < len; i++) {
            buffer[i] = rfm9x_read_register(0x00);
        }
        buffer[len] = '\0';

        return len;
    }

    return 0; // No valid message
}
void listen_and_timestamp_rx(void) {
    char buffer[64] = {0};
    uint8_t len = rfm9x_try_receive_message(buffer, sizeof(buffer));
    if (len > 0) {
        char buf[128];
        sprintf(buf, "[RX] Alert received at t=%lu ms\r\n", system_time);
        uart_send_string(buf);
        sprintf(buf, "[RX] Message content: %s\r\n", buffer);
        uart_send_string(buf);
    }
}

void transmit_alert_with_timestamp(void) {
    const char* msg = "EMERGENCY_ALERT";
    uint32_t tx_time = system_time;
    char buf[128];
    sprintf(buf, "[TX] Alert triggered at t=%lu ms\r\n", tx_time);
    uart_send_string(buf);
    rfm9x_transmit_message(msg);
    sprintf(buf, "[TX] Alert sent at t=%lu ms\r\n", system_time);
    uart_send_string(buf);
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
    sprintf(buf, "  RegPaConfig       (0x09) = 0x%02X\r\n\n", paconfig);
    //uart_send_string(buf);
    //sprintf(buf, "  RegFrf:           %02X %02X %02X\r\n", frf_msb, frf_mid, frf_lsb);
    uart_send_string(buf);
    sprintf(buf, "  RegFrfMSB         (0x06) = 0x%02X (0xE4)\r\n", frf_msb);
    uart_send_string(buf);
    sprintf(buf, "  RegFrfMID         (0x07) = 0x%02X (0xC0)\r\n", frf_mid);
    uart_send_string(buf);
    sprintf(buf, "  RegFrfLSB         (0x08) = 0x%02X (0x00)\r\n\n", frf_lsb);
    uart_send_string(buf);
    sprintf(buf, "  RegFifoAddrPtr    (0x0D) = 0x%02X\r\n", fifo_addr);
    uart_send_string(buf);
    sprintf(buf, "  RegPayloadLength  (0x22) = 0x%02X (%d)\r\n", payload_len, expected_payload_len);
    uart_send_string(buf);
    sprintf(buf, "  RegIrqFlags       (0x12) = 0x%02X\r\n", irq_flags);
    uart_send_string(buf);
    sprintf(buf, "  RegVersion        (0x42) = 0x%02X (SX1278)\r\n", version);
    uart_send_string(buf);
}

void test_rfm9x_registers() {
    uart_send_string("Testing SPI Communication with RFM9X...\r\n");
    
    for (uint8_t reg = 0x00; reg <= 0x7F; reg++) {  //read all 128 registers
        uint8_t value = rfm9x_read_register(reg);// - (reg != 0));
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
    if (len == 0 || len >= 63) return;

    rfm9x_set_mode(0x82); // standby
    nano_wait(10000);

    rfm9x_write_register(0x0D, 0x00); // FIFO pointer = base addr
    for (uint8_t i = 0; i < len; i++) {
        rfm9x_write_register(0x00, message[i]);
    }

    rfm9x_write_register(0x10, 0x00); // RegFifoRxCurrentAddr = 0
    rfm9x_write_register(0x13, len);  // RegRxNbBytes = message length
    rfm9x_write_register(0x12, 0x40); // Set RxDone flag
    uint8_t rx_addr = rfm9x_read_register(0x10);
    rfm9x_write_register(0x0D, rx_addr);

    rfm9x_set_mode(0x05); // Switch to RX continuous mode

    uart_send_string("Fake ALERT message injected.\r\n");

    char buf[64] = {0};
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = rfm9x_read_register(0x00);
    }
    buf[len] = '\0';

    uart_send_string("What I get back: ");
    uart_send_string(buf);
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