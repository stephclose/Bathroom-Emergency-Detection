#include "rfm9x.h"
#include "display.h"
#include "support.h"
#include "usart.h" //debugging
#include <stdio.h>

void lcd_handler() {
    i2c1_init();
    lcd_init();
    lcd_send_string("Hello World!");
    lcd_set_cursor(1, 0); 
    lcd_send_string("STM32 & PCF8574");
}

void rfm9x_handler(){
    init_spi1_lora();
    rfm9x_reset();
    rfm9x_init();

    uint8_t version = rfm9x_read_register(0x42);
    if (version != 0x12) {
        printf("Error: RFM9X Not Detected! Read: 0x%02X\n", version);
        while (1); //HALT
    }
    printf("RFM9X Detected! Version: 0x%02X\n", version);

    while (1) {
        uint8_t msg[] = "Hello, LoRa!"; //send message
        rfm9x_send_packet(msg, sizeof(msg) - 1);
        printf("LoRa Packet Sent: %s\n", msg);
        nano_wait(5000000);

        //receive
        uint8_t rx_buffer[32];
        uint8_t length = rfm9x_receive_packet(rx_buffer, sizeof(rx_buffer));
        if (length > 0) {
            rx_buffer[length] = '\0';  //null-terminate received message
            printf("Received LoRa Message: %s\n", rx_buffer);
        } else {
            printf("No LoRa Message Received.\n");
        }
    }
}

int main(void) {

    usart_init();
    lcd_handler();
    rfm9x_handler();

    while (1);
}


//button interrupt