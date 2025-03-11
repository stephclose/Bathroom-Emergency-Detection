#include "i2c_lcd.h"
#include "stm32f0xx_hal.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

#define LCD_ADDR (0x27 << 1)
#define LCD_BACKLIGHT 0x08
#define LCD_CMD_CLEAR_DISPLAY 0x01
#define LCD_CMD_RETURN_HOME 0x02
#define LCD_CMD_ENTRY_MODE_SET 0x04
#define LCD_CMD_DISPLAY_CONTROL 0x08
#define LCD_CMD_FUNCTION_SET 0x20
#define LCD_CMD_SET_CGRAM_ADDR 0x40
#define LCD_CMD_SET_DDRAM_ADDR 0x80

#define LCD_ENTRY_MODE_LEFT 0x02
#define LCD_ENTRY_MODE_SHIFT_OFF 0x00

#define LCD_DISPLAY_ON 0x04
#define LCD_CURSOR_OFF 0x00
#define LCD_BLINK_OFF 0x00

#define LCD_FUNCTION_4BIT 0x00
#define LCD_FUNCTION_2LINE 0x08
#define LCD_FUNCTION_5x8DOTS 0x00

static void lcd_write(I2C_HandleTypeDef *hi2c, uint8_t data, uint8_t rs) {
    uint8_t high_nibble = data & 0xF0;
    uint8_t low_nibble = (data << 4) & 0xF0;
    uint8_t data_arr[4];

    data_arr[0] = high_nibble | rs | LCD_BACKLIGHT | 0x04; // En = 1
    data_arr[1] = high_nibble | rs | LCD_BACKLIGHT;        // En = 0
    data_arr[2] = low_nibble | rs | LCD_BACKLIGHT | 0x04;  // En = 1
    data_arr[3] = low_nibble | rs | LCD_BACKLIGHT;         // En = 0

    HAL_I2C_Master_Transmit(hi2c, LCD_ADDR, data_arr, sizeof(data_arr), HAL_MAX_DELAY);
}

void lcd_send_cmd(I2C_HandleTypeDef *hi2c, uint8_t cmd) {
    lcd_write(hi2c, cmd, 0x00);
}

void lcd_send_data(I2C_HandleTypeDef *hi2c, uint8_t data) {
    lcd_write(hi2c, data, 0x01);
}

void lcd_init(I2C_HandleTypeDef *hi2c) {
    HAL_Delay(50); 
    lcd_send_cmd(hi2c, 0x30);
    HAL_Delay(5);
    lcd_send_cmd(hi2c, 0x30);
    HAL_Delay(1);
    lcd_send_cmd(hi2c, 0x30);
    lcd_send_cmd(hi2c, 0x20); 

    lcd_send_cmd(hi2c, LCD_CMD_FUNCTION_SET | LCD_FUNCTION_4BIT | LCD_FUNCTION_2LINE | LCD_FUNCTION_5x8DOTS);
    lcd_send_cmd(hi2c, LCD_CMD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
    lcd_send_cmd(hi2c, LCD_CMD_CLEAR_DISPLAY);
    HAL_Delay(2);
    lcd_send_cmd(hi2c, LCD_CMD_ENTRY_MODE_SET | LCD_ENTRY_MODE_LEFT | LCD_ENTRY_MODE_SHIFT_OFF);
}

void lcd_send_string(I2C_HandleTypeDef *hi2c, char *str) {
    while (*str) {
        lcd_send_data(hi2c, (uint8_t)(*str));
        str++;
    }
}

void lcd_clear(I2C_HandleTypeDef *hi2c) {
    lcd_send_cmd(hi2c, LCD_CMD_CLEAR_DISPLAY);
    HAL_Delay(2);
}

void lcd_set_cursor(I2C_HandleTypeDef *hi2c, uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? (0x80 + col) : (0xC0 + col);
    lcd_send_cmd(hi2c, address);
}
