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

    //send packet
    uint8_t msg[] = "Hello from STM32!";
    rfm9x_send_packet(msg, sizeof(msg) - 1);

    uart_send_string("Packet sent!\r\n");
    toggle_pc7_debug(2, 200);

    uint8_t buffer[32];
    uint8_t len = rfm9x_receive_packet(buffer, sizeof(buffer));

    if (len > 0) {
        buffer[len] = '\0';// null-terminate
        uart_send_string("Received Packet: ");
        uart_send_string((char*)buffer);
        uart_send_string("\r\n");
        toggle_pc7_debug(3, 100);
    } else {
        uart_send_string("No packet received\r\n");
    }

    //recieve packet
    uart_send_string("Listening...\r\n");

    while (1) {
        __WFI(); // Sleep forever
    }
}
