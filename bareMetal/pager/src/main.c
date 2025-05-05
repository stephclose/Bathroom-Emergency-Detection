#include "rfm9x.h"
#include "display.h"
#include "support.h"
#include "button.h"
#include "clock.h"
#include "uart.h"
#include "debug.h"
#include "state.h"
#include "buzzer.h"
#include "vibrator.h"
#include "hc12.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h> 

// === System State ===
volatile lcd_state_t lcd_state = LCD_STATE_STANDBY;
uint32_t emergency_trigger_time = 0;
uint32_t alert_display_time = 0;

// === Forward Declarations ===
void lcd_display_standby(void);
void lcd_display_emergency(const char* msg);

// === LoRa Initialization ===
void rfm9x_init(void) {
    rfm9x_reset();
    nano_wait(100000);
    init_spi1_lora();
    nano_wait(100000);
    //test_rfm9x_registers();
    rfm9x_enter_lora_mode();
    rfm9x_set_mode(0x81);
    nano_wait(10000);
    rfm9x_set_frequency(915000000);
    nano_wait(10000);
    RFO_max_output();
    nano_wait(100000);
    //rfm9x_write_register(0x1D, 0x72);
    //rfm9x_write_register(0x1E, 0xC4);
}

int main(void) {

    //1. Initialize all subsystems ===
    internal_clock();
    init_systick();
    uart_init();
    i2c1_init();
    rfm9x_init();
    nano_wait(60000);
    button_init();

    buzzer_init();
    vibrator_motor_init();

    configure_pb2_exti();
    uart5_init();
    lcd_off();
    //usart4_init();

    // === Transmit a character manually on USART5 ===
    while (!(USART5->ISR & USART_ISR_TXE));  //wait until transmit register empty
    USART5->TDR = 'U';
    while (!(USART5->ISR & USART_ISR_TC));

    lcd_state_t prev_state = -1;

    while (1) {
        if (lcd_state != prev_state) {
            switch (lcd_state) {
                case LCD_STATE_STANDBY:
                    uart_send_string("[STATE] Entered STANDBY mode\r\n");
                    lcd_off();
                    break;

                case LCD_STATE_EMERGENCY:
                    uart_send_string("[STATE] Entered EMERGENCY mode\r\n");
                    //-----
                    //RX RECIEVED HERE (simulate with button press for now)
                    //-----
                    lcd_display_emergency("3"); //Replace with real ID later

                case LCD_STATE_ACKNOWLEDGED:
                    uart_send_string("[STATE] Entered ACKNOWLEDGED mode\r\n");
                    lcd_clear();
                    lcd_send_string("Emergency");
                    lcd_set_cursor(1, 0);
                    lcd_send_string("Acknowledged");

                    // === TX to Main MCU, emergency alert ===
                    const char* rf_message = "EMERGENCY_ALERT";
                    uart_send_string("[RF] Transmitting emergency message...\r\n");
                    rfm9x_transmit_message(rf_message);
                    delay_ms(100);
                    rfm9x_print_tx_config(strlen(rf_message));
                    emergency_trigger_time = get_system_time_ms();
                    delay_ms(100);
                    rfm9x_set_mode(0x82);

                    delay_ms(500);
                    lcd_off();
                    lcd_state = LCD_STATE_STANDBY;
                    break;
            }
            prev_state = lcd_state;
        }
    }
}

// === STANDBY DISPLAY ===
void lcd_display_standby(void) {
    lcd_off();
}

// === EMERGENCY DISPLAY ===
void lcd_display_emergency(const char* bathroom_id) {
    lcd_init();
    alert_display_time = get_system_time_ms();

    char buf[64];
    sprintf(buf, "[TIMING] Alert displayed at %lu ms (delay = %lu ms)\r\n",
            alert_display_time,
            alert_display_time - emergency_trigger_time);
    uart_send_string(buf);

    lcd_clear();
    lcd_send_string("**EMERGENCY**");
    lcd_set_cursor(1, 0);
    lcd_send_string("Bathroom ");
    lcd_send_string((char*) bathroom_id);
    vibrator_motor_on();

    while (1) {
        __WFI();
        buzzer_on();
        delay_ms(100);
        buzzer_off();
        delay_ms(100);
        if (lcd_state == LCD_STATE_ACKNOWLEDGED){
            vibrator_motor_off();
            break;  //Acknowledge received
        }
    }

}
