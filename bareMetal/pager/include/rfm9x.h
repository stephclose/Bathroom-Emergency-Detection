#ifndef RFM9X_H
#define RFM9X_H

#include <stdint.h>

void init_spi1_lora(void);
void rfm9x_init(void);
void rfm9x_reset(void);
uint8_t rfm9x_read_register(uint8_t reg);
void rfm9x_write_register(uint8_t reg, uint8_t value);
void rfm9x_set_mode(uint8_t mode);
void rfm9x_set_frequency(uint32_t freq);
void rfm9x_set_tx_power(uint8_t power);
void rfm9x_send_packet(uint8_t *data, uint8_t length);
uint8_t rfm9x_receive_packet(uint8_t *buffer, uint8_t max_length);
void rfm9x_nss_select(void);
void rfm9x_nss_deselect(void);
void test_rfm9x_registers(void);
void test_spi_loopback(void);
void check_miso_status(void);
void test_send_self(void);
void debug_spi_gpio(void);
void test_sck_pin(void);
void check_rfm9x_version(void);
void rfm9x_test_suite(void);
void test_spi_manual_transfer(void);
void debug_spi_registers(void);
void test_rfm9x_basic_communication(void);
#endif // RFM9X_H
