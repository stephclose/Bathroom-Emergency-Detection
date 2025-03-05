#include "stm32f0xx.h"
#include "display.h"
#include "support.h"

//===========================================================================
// I2C1 Initialization (PB6 = SCL, PB7 = SDA)
//===========================================================================
void i2c1_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; //enable I2C1 clock
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable GPIOB clock

    //configure PB6 (SCL) and PB7 (SDA) as AF1
    GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOB->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);

    GPIOB->AFR[0] |= (1 << (6 * 4)) | (1 << (7 * 4)); //set AF1 for I2C1

    GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7; //open-drain
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7);
    GPIOB->PUPDR |= (1 << (6 * 2)) | (1 << (7 * 2)); //enable pull-ups

    I2C1->TIMINGR = 0x00303D5B; //100 kHz I2C timing
    I2C1->CR1 |= I2C_CR1_PE; //enable I2C1
}

//===========================================================================
// Write Function
//===========================================================================
void i2c1_write(uint8_t addr, uint8_t data) {
    while (I2C1->ISR & I2C_ISR_BUSY);
    I2C1->CR2 = (addr & 0xFE) | (1 << 16); //set addr & 1 byte w
    I2C1->CR2 |= I2C_CR2_START; //generate start condition

    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = data;

    while (!(I2C1->ISR & I2C_ISR_TC));
    I2C1->CR2 |= I2C_CR2_STOP;
}

//===========================================================================
// LCD Command Function (Send Command to LCD)
//===========================================================================
void lcd_send_cmd(uint8_t cmd) {
    uint8_t upper = (cmd & 0xF0);
    uint8_t lower = ((cmd << 4) & 0xF0);

    i2c1_write(LCD_ADDR, upper | 0x0C); //en = 1, RS = 0
    i2c1_write(LCD_ADDR, upper | 0x08); //en = 0
    i2c1_write(LCD_ADDR, lower | 0x0C); //en = 1, RS = 0
    i2c1_write(LCD_ADDR, lower | 0x08); //en = 0
}

//===========================================================================
// LCD Data Function (Send Character to LCD)
//===========================================================================
void lcd_send_data(uint8_t data) {
    uint8_t upper = (data & 0xF0);
    uint8_t lower = ((data << 4) & 0xF0);

    i2c1_write(LCD_ADDR, upper | 0x0D); //en = 1, RS = 1
    i2c1_write(LCD_ADDR, upper | 0x09); //en = 0
    i2c1_write(LCD_ADDR, lower | 0x0D); //en = 1, RS = 1
    i2c1_write(LCD_ADDR, lower | 0x09); //en = 0
}

//===========================================================================
// LCD Initialization
//===========================================================================
void lcd_init(void) {
    nano_wait(500000);
    lcd_send_cmd(0x33); //initialize LCD
    lcd_send_cmd(0x32); //set to 4-bit mode
    lcd_send_cmd(LCD_CMD_FUNCTION_SET);
    lcd_send_cmd(LCD_CMD_DISPLAY_ON);
    lcd_send_cmd(LCD_CMD_ENTRY_MODE);
    lcd_send_cmd(LCD_CMD_CLEAR_DISPLAY);
    nano_wait(2000000);
}

//===========================================================================
// LCD String Function (Print String to LCD)
//===========================================================================
void lcd_send_string(char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

//===========================================================================
// LCD Clear Display
//===========================================================================
void lcd_clear(void) {
    lcd_send_cmd(LCD_CMD_CLEAR_DISPLAY);
    nano_wait(2000000);
}

//===========================================================================
// LCD Set Cursor Position
//===========================================================================
void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t pos = (row == 0) ? (LCD_CMD_SET_CURSOR + col) : (LCD_CMD_SET_CURSOR + 0x40 + col);
    lcd_send_cmd(pos);
}
