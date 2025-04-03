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

int main(void) {
    internal_clock();
    init_systick();
    uart_init();
    pc7_led_init();

    rfm9x_reset();
    init_spi1_lora();
    test_rfm9x_registers();
    nano_wait(1000000);
    test_rfm9x_basic_communication();
    nano_wait(1000000);
    
    //fake_rx_custom_message("blahblahblbh!");

    //uint8_t buffer[64];
    //uint8_t len = rfm9x_receive_packet(buffer, sizeof(buffer));
    //if (len > 0) {
    //    buffer[len] = '\0';
    //    uart_send_string("Received: ");
    //    uart_send_string((char*)buffer);
    //    uart_send_string("\r\n");
    //}


    while (1) {
        uint8_t irq_flags = rfm9x_read_register(0x12); //IRQ Flags
        if (irq_flags & 0x40) { // RX Done
            rfm9x_write_register(0x12, 0xFF); //clear all IRQ flags
            uint8_t buffer[64];
            uint8_t len = rfm9x_receive_packet(buffer, sizeof(buffer));
            buffer[len] = '\0';
            uart_send_string("Got message: ");
            uart_send_string((char*)buffer);
            uart_send_string("\r\n");
            rfm9x_set_mode(0x05);
        }
    __WFI();
}

}
