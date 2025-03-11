#include "stm32f0xx_hal.h"
#include "error_handler.h"

void Error_Handler(void) {
    // Optional: Print an error message (if UART is enabled)
    // uart_send_string("System Error! Halting...\r\n");

    // Optional: Blink LED on error (use your actual GPIO pin)
    while (1) {
        // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        // HAL_Delay(500);
    }
}
