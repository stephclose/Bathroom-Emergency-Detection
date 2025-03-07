#ifndef DEBUG_H
#define DEBUG_H

#include <stdint.h>

void pc7_led_init(void);
void toggle_pc7_debug(int times, int delay_ms);
void pc8_led_init(void);
void toggle_pc8_debug(int times, int delay_ms);


#endif // RFM9X_H