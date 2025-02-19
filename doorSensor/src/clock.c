#include "stm32f0xx.h"
#include "system_stm32f0xx.h"


// sets up internal clock and external osc
// manages timing functions

void internal_clock()
{
    // enable HSI
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0); // wait until hsi is stable

    // config pll
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL | RCC_CFGR_PLLXTPRE);
    RCC->CFGR |= (RCC_CFGR_PLLSRC_HSI_DIV2 | RCC_CFGR_PLLMUL12);

    // enable pll
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);

    // set flash for 48MHz operation
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

    // set as system clock
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // until pll used

    // prescalers for AHB APB
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  
    RCC->CFGR |= RCC_CFGR_PPRE_DIV1; 

    SystemCoreClockUpdate();
}

void setup_lse_clock()
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_DBP;

    RCC->BDCR |= RCC_BDCR_LSEON;
    while (!(RCC->BDCR & RCC_BDCR_LSERDY));  

    // lse as rtc clock
    RCC->BDCR &= ~RCC_BDCR_RTCSEL;
    RCC->BDCR |= RCC_BDCR_RTCSEL_LSE;

    RCC->BDCR |= RCC_BDCR_RTCEN;
}