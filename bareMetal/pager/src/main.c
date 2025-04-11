#include "rfm9x.h"
#include "display.h"
#include "support.h"
#include "button.h"
#include "clock.h"
#include "uart.h"
#include "debug.h"
#include <stdio.h>


void lcd_handler() {
    i2c1_init();
    lcd_init();
    lcd_send_string("STM32");
    lcd_set_cursor(1, 0); 
    lcd_send_string(" & PCF8574");
}

void rfm9x_init(void) {
    rfm9x_reset();
    nano_wait(100000);
    init_spi1_lora();
    nano_wait(100000);

    test_rfm9x_registers();

    rfm9x_enter_lora_mode();
    nano_wait(10000);
    rfm9x_set_mode(0x81); // standby
    nano_wait(10000);
}

int main(void) {
    internal_clock();
    init_systick();
    uart_init();
    //pc7_led_init();

    rfm9x_init();

    //rfm9x_write_register(0x05, 0xE4);
    //nano_wait(1000);
    //rfm9x_write_register(0x06, 0x00);
    //nano_wait(1000);
    //rfm9x_write_register(0x07, 0xC0);
    //nano_wait(1000);

    //set frequency and power
    //rfm9x_set_frequency(915);
    //rfm9x_set_tx_power(14);

    uart_send_string("LoRa TX/RX Ready\r\n");
    const char* message = "Hello!";
    while (1) {
        //TX
        //test_rfm9x_write_read_frf_registers();
        rfm9x_transmit_message(message);
        rfm9x_print_tx_config(strlen(message));
        nano_wait(1000000000);

        //RX
        rfm9x_write_register(0x12, 0xFF);
        rfm9x_set_mode(0x05);

        char rx_buf[64];
        uint8_t len = rfm9x_try_receive_message(rx_buf, sizeof(rx_buf));
        if (len > 0) {
            uart_send_string("Got message: ");
            uart_send_string(rx_buf);
            uart_send_string("\r\n");
        } else {
            uart_send_string("No message received...\r\n");
        }
        __WFI(); //sleep until interrupt (saves power)
    }
}
