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
    
    fake_rx_custom_message("blahblahblbh!");

    while (1) {
        __WFI(); //sleep forever
    }
}
