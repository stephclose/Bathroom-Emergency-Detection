#include "stm32f0xx.h"
#include "clock.h"
#include "reedSwitch.h"
#include "uart.h"
#include <stdio.h>

volatile uint8_t doorState = 1; // door starts open
volatile uint32_t system_time = 0; // global system time



void checkLSE() 
{
    if (RCC->BDCR & RCC_BDCR_LSERDY) 
    {
        printf("LSE 32.768 kHz Clock is running!\n");
    } 
    else 
    {
        printf("LSE Clock FAILED to start.\n");
    }
}

void SysTick_Handler(void) 
{
    system_time++; // Increments every 1ms
}

int main(void) {
    
    // init system clock
    //internal_clock(); 
    //setup_lse_clock();
    //checkLSE();

    // init reedswitch
    uart_init();
    printf("Testing Reed Switch...\r\n");
    
    // initalize systick timer
    SysTick_Config(SystemCoreClock / 1000);

    reedSwitch_init();

    // init rf module
    // init bat monitor
    // enter sleep mode
    
    
    while (1) 
    {
        // enter sleep mode wait for wakeup
        //sleepModeEnter(); 
        

        // if door state open send rf message
        // if closed send rf message

        // read battery voltage
        // if lower than 3.3 send message
    
    }
}
