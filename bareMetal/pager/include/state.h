#ifndef STATE_H
#define STATE_H

typedef enum {
    LCD_STATE_STANDBY,
    LCD_STATE_EMERGENCY,
    LCD_STATE_ACKNOWLEDGED
} lcd_state_t;

extern volatile lcd_state_t lcd_state;

#endif
