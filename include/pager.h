#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f0xx.h"

#define DOOR_CLOSED 0
#define DOOR_OPEN 1
#define MOVEMENT_DETECTED 1
#define NO_MOVEMENT 0
#define EMERGENCY_ALERT_TRIGGERED 1
#define FALSE_ALARM 0


void internal_clock();