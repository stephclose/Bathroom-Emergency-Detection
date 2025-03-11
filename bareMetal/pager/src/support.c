#include "stm32f0xx.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}