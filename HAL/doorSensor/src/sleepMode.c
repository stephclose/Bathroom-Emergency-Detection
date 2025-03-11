#include "stm32f091xc.h"
#include "sleepMode.h"
#include "batMonitor.h"
// manages low power mode 
// wakes on door open close event 

// put into low power
void sleepModeEnter(void)
{
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    __WFI(); // wait for int to wake
}

void RTC_WKUP_IRQHandler(void)
{
    if (RTC->ISR & RTC_ISR_WUTF) // wake up flag 
    { 
        RTC->ISR &= ~RTC_ISR_WUTF; // clear flag
        
        check_battery_voltage(); // check bat not implemented yet
    }
}

void setup_RTC_wakeup(void) // for hourly wake
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;  
    PWR->CR |= PWR_CR_DBP;  
    RCC->BDCR |= RCC_BDCR_LSEON;  // enable ext crystal
    while (!(RCC->BDCR & RCC_BDCR_LSERDY));
    RCC->BDCR |= RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_LSE; // LSE as rtc clock
    RTC->WPR = 0xCA; 
    RTC->WPR = 0x53;
    RTC->CR &= ~RTC_CR_WUTE; // turn off wakeup Timer
    RTC->ISR |= RTC_ISR_INIT; // init mode
    while (!(RTC->ISR & RTC_ISR_INITF)); // wait stabalize
    RTC->WUTR = 3600; // set wakeup time 1hr
    RTC->CR |= RTC_CR_WUTIE | RTC_CR_WUCKSEL_2; // enable wake up timer
    RTC->ISR &= ~RTC_ISR_INIT; // exit init mode
    RTC->CR |= RTC_CR_WUTE; // enabl wakeup timer
    RTC->WPR = 0xFF; 

    NVIC_EnableIRQ(RTC_IRQn);
}


