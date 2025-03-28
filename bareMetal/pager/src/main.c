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
    while(1){
        /*uart_send_string("Pulling NSS LOW\r\n");
        rfm9x_nss_select();
        while (!(SPI1->SR & SPI_SR_TXE));
        SPI1->DR = 0xAA;
        uart_send_string("TX 0xAA sent, waiting...\r\n");

        uart_send_string("Releasing NSS\r\n");
        nano_wait(10000000);
        rfm9x_nss_deselect();
        nano_wait(10000000);*/
        //test_spi_loopback();
        test_rfm9x_registers();
        nano_wait(1000000);

    }

    test_rfm9x_basic_communication();


    while (1) {
        __WFI(); // Sleep forever
    }
}
