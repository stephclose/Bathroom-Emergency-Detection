#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

void buzzer_on(void);
void buzzer_off(void);
void buzzer_init(void);
void buzzer_pulse(uint8_t times);


#endif // BUZZER_H
