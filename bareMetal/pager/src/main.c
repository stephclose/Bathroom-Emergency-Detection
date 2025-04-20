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
#include <stdio.h>

volatile lcd_state_t lcd_state = LCD_STATE_STANDBY;
uint32_t emergency_trigger_time = 0;
uint32_t alert_display_time = 0;


void lcd_display_standby(void);
void lcd_display_emergency(const char* msg);

void rfm9x_init(void) {
    rfm9x_reset();
    nano_wait(100000);
    init_spi1_lora();
    nano_wait(100000);

    //test_rfm9x_registers();
    rfm9x_enter_lora_mode();
    nano_wait(10000);
    //rfm9x_set_frequency(915000000); //915 MHz = 0xE4C000
    rfm9x_set_mode(0x82); // standby
    nano_wait(10000);
}

int main(void) {
    //1. Initialize all subsystems ===
    internal_clock();
    init_systick();
    uart_init();
    setup_tim2_pwm_pa15();
    i2c1_init();
    lcd_init();
    rfm9x_init();
    button_init();
    buzzer_init();
    vibrator_motor_init();
    configure_pb2_exti();
    //configure_pa0_exti(); 

    // === VIBRATOR TEST: ON FOR 5 SECONDS ===
    uart_send_string("System initialized.\r\n");

    //uart_send_string("Running vibrator motor test...\r\n");
    //vibrator_motor_on();
    //delay_ms(20000);  // 5 seconds
    //vibrator_motor_off();
    //uart_send_string("Vibrator motor test complete.\r\n");

    //2. Initial LCD display ===
    lcd_display_standby();

    lcd_state_t prev_state = -1;

    while (1) {
        if (lcd_state != prev_state) {
            switch (lcd_state) {
                case LCD_STATE_STANDBY:
                    uart_send_string("[STATE] Entered STANDBY mode\r\n");
                    lcd_display_standby();
                    break;

                case LCD_STATE_EMERGENCY:
                    uart_send_string("[STATE] Entered EMERGENCY mode\r\n");
                    lcd_display_emergency("3"); //Replace "3" with real ID later
                    buzzer_test_gpio_toggle();
                    break;

                case LCD_STATE_ACKNOWLEDGED:
                    uart_send_string("[STATE] Entered ACKNOWLEDGED mode\r\n");
                    lcd_clear();
                    lcd_send_string("Emergency");
                    lcd_set_cursor(1, 0);
                    lcd_send_string("Acknowledged");
                    break;
            }
            prev_state = lcd_state;
        }

        buzzer_poll();

        // === TEST TRANSMIT/RECEIVE ===
        uart_send_string("Manual FRF write...\r\n");
        rfm9x_set_mode(0x81); // standby
        nano_wait(10000);

        rfm9x_write_register(0x06, 0xE4);
        nano_wait(1000);
        rfm9x_write_register(0x07, 0x00);
        nano_wait(1000);
        rfm9x_write_register(0x08, 0xC0);
        nano_wait(1000);

        char buf[64];
        sprintf(buf, "Readback FRF: %02X %02X %02X\r\n",
        rfm9x_read_register(0x06),
        rfm9x_read_register(0x07),
        rfm9x_read_register(0x08));
        uart_send_string(buf);

        const char* message = "EMERGENCY_ALERT";
        uart_send_string("[TEST] Transmitting message...\r\n");
        rfm9x_transmit_message(message);
        rfm9x_print_tx_config(strlen(message));
        nano_wait(100000000);

        char rx_buf[64] = {0};
        uint8_t len = rfm9x_try_receive_message(rx_buf, sizeof(rx_buf));

        if (len > 0) {
            uart_send_string("[TEST] Message received: ");
            uart_send_string(rx_buf);
            uart_send_string("\r\n");
            emergency_trigger_time = get_system_time_ms();
            lcd_state = LCD_STATE_EMERGENCY;
        } else {
            uart_send_string("[TEST] No message received.\r\n");
        }

        nano_wait(500000000);

        //transmit_alert_with_timestamp();
        //delay_ms(5000);
        //listen_and_timestamp_rx();
    }
}
  
        //TX
        //rfm9x_transmit_message(message);
        //rfm9x_print_tx_config(strlen(message));
        //nano_wait(1000000000);

        //RX
        //rfm9x_write_register(0x12, 0xFF); //clear IRQ
        //rfm9x_set_mode(0x05);

        //char rx_buf[64];
        //uint8_t len = rfm9x_try_receive_message(rx_buf, sizeof(rx_buf));
        //nano_wait(1000000000);
        //if (len > 0) {
        //    uart_send_string("Got message: ");
        //    uart_send_string(rx_buf);
        //    uart_send_string("\r\n");
        //} else {
        //    uart_send_string("No message received...\r\n");
       // }
        //nano_wait(1000000000);
//}

// === STANDBY DISPLAY ===
void lcd_display_standby(void) {
    lcd_clear();
    lcd_send_string("System Ready");
    lcd_set_cursor(1, 0);
    lcd_send_string("Awaiting Alert");
}

// === EMERGENCY DISPLAY ===
void lcd_display_emergency(const char* bathroom_id) {
    alert_display_time = get_system_time_ms();  // <--- Place it here

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

    buzzer_on();  //start constant buzzer
    vibrator_motor_on();

    while (lcd_state == LCD_STATE_EMERGENCY) {
        __WFI();  //sleep until interrupt
    }

    buzzer_off();  //stop when acknowledged
    vibrator_motor_off();
}
