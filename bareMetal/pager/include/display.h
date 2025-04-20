#ifndef LCD_H
#define LCD_H

#include <stdint.h>

#define LCD_ADDR (0x27 << 1)  //cefault I2C address for PCF8574

//LCD Commands
#define LCD_CMD_CLEAR_DISPLAY    0x01
#define LCD_CMD_RETURN_HOME      0x02
#define LCD_CMD_ENTRY_MODE       0x06
#define LCD_CMD_DISPLAY_ON       0x0C
#define LCD_CMD_FUNCTION_SET     0x28  //4-bit, 2-line, 5x8 font
#define LCD_CMD_SET_CURSOR       0x80  //cursor position command

void i2c1_init(void);
void lcd_init(void);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_send_string(char *str);
void lcd_clear(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
int alert_triggered(void);
int emergency_cleared(void);

#endif // LCD_H
