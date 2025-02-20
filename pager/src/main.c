#include <pager.h>
#include <stdlib.h>
#include "stm32f0xx.h"


//configure GPIOC
void enable_GPIOC(void){
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    //GPIOC->MODER &= ~0xffff;
    //GPIOC->MODER |= 0x55 << 8;
}

void spi_enable(void){
    
}

int main(void) {
    internal_clock();
    
}
