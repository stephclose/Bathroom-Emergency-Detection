#include "rfm9x.h"
#include "display.h"
#include "support.h"
#include "button.h"
#include "clock.h"
#include "uart.h"
#include <stdio.h>


void lcd_handler() {
    i2c1_init();
    lcd_init();
    lcd_send_string("STM32");
    lcd_set_cursor(1, 0); 
    lcd_send_string(" & PCF8574");
}

void rfm9x_handler(){
    
    uart_send_string("Starting RFM9X Handler...\r\n");
    
    rfm9x_reset();
    toggle_pc7_debug(1, 500);

    init_spi1_lora();
    toggle_pc7_debug(1, 500);

    rfm9x_init();
    toggle_pc7_debug(1, 500);

    debug_spi_gpio();
    
    test_spi_loopback();
    test_spi_echo_loopback();
    test_miso_pin();
    test_spi_echo_loopback();
    //test_rfm9x_registers();

    //check if RFM9x is responding
    //char debug_buf[50];
    //sprintf(debug_buf, "RFM9X Version: 0x%02X\r\n", version);
    //uart_send_string(debug_buf);

    uart_send_string("Checking SPI Read...\r\n");
    uint8_t version = rfm9x_read_register(0x42);
    uart_send_string("SPI Read Done!\r\n");

    if (version != 0x12) {
        uart_send_string("ERROR: RFM9X Not Detected!\r\n");
        toggle_pc7_debug(5, 200); //blink 5 times fast if error
        while (1); // HALT
    }

    uart_send_string("RFM9X Detected! Version OK\r\n");
    toggle_pc7_debug(1, 1000); //long blink for success

    /*
    //check RSSI
    uint8_t rssi = rfm9x_read_register(0x1B);
    sprintf(debug_buf, "RFM9X RSSI: -%u dBm\r\n", rssi);
    uart_send_string(debug_buf);

    //testing TX 
    WARNING DO NOT RUN THIS UNLESS ANT IS ATTACHED
    uart_send_string("Testing LoRa Transmission...\r\n");
    uint8_t msg[] = "Hello, LoRa!";
    rfm9x_send_packet(msg, sizeof(msg) - 1);
    toggle_pc7_debug(1, 500); // Blink LED after send
    }
    //
    for (uint8_t reg = 0x00; reg <= 0x7F; reg++) {
        char buf[50];
        uint8_t val = rfm9x_read_register(reg);
        sprintf(buf, "Reg 0x%02X = 0x%02X\r\n", reg, val);
        uart_send_string(buf);
    }

    while (1) {
        uint8_t msg[] = "Hello, LoRa!"; //send message
        rfm9x_send_packet(msg, sizeof(msg) - 1);
        printf("LoRa Packet Sent: %s\n", msg);
        toggle_pc7_debug(1, 500); // Blink 1 time after send

        //receive
        uint8_t rx_buffer[32];
        uint8_t length = rfm9x_receive_packet(rx_buffer, sizeof(rx_buffer));
        if (length > 0) {
            rx_buffer[length] = '\0';  //null-terminate received message
            uart_send_string("Received LoRa Message: ");
            uart_send_string((char*)rx_buffer);
            uart_send_string("\r\n");
            toggle_pc7_debug(3, 300);
        } else {
            printf("No LoRa Message Received.\n");
            toggle_pc7_debug(1, 2000);
        }   
    }*/
}

int main(void) {

    internal_clock();
    init_systick();
    uart_init();
    //button_init(); //slow, fix 
    pc7_led_init(); //debugging onboard leds
    pc8_led_init();
    lcd_handler();

    toggle_pc8_debug(1, 500);
    rfm9x_handler();


    while (1){
        __WFI(); //wait for interrupt
    }
}