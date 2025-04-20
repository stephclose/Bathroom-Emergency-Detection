#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

void setup_tim2_pwm_pa15(void);
void buzzer_test_gpio_toggle(void);

void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);
void buzzer_poll(void);


#endif // BUZZER_H
