#include "clock.h"
// sets up internal clock and external osc
// manages timing functions

volatile uint32_t system_time = 0;  //global system timer in ms

// config HSI for system clocking
void internal_clock()
{
    // enable HSI
    RCC->CR |= RCC_CR_HSION; // enable hsi osc
    while ((RCC->CR & RCC_CR_HSIRDY) == 0); // wait until hsi is stable

    // config pll
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL | RCC_CFGR_PLLXTPRE); // clear any prev sets
    RCC->CFGR |= (RCC_CFGR_PLLSRC_HSI_DIV2 | RCC_CFGR_PLLMUL12); // pll source to 48MHz

    // enable pll
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0); // stable 

    // set flash for 48MHz operation
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

    // set as system clock for pll
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL; // select as source
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // until pll used

    // prescalers for AHB APB
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  
    RCC->CFGR |= RCC_CFGR_PPRE_DIV1; 

    SystemCoreClockUpdate(); // update global var 
}

// config lse osc 
void setup_lse_clock()
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN; 
    PWR->CR |= PWR_CR_DBP; // power interface clock for rtc 

    RCC->BDCR |= RCC_BDCR_LSEON; // write to reg 
    while (!(RCC->BDCR & RCC_BDCR_LSERDY));  

    // lse as rtc clock
    RCC->BDCR &= ~RCC_BDCR_RTCSEL; 
    RCC->BDCR |= RCC_BDCR_RTCSEL_LSE;

    RCC->BDCR |= RCC_BDCR_RTCEN; // enable rtc 
}

void SysTick_Handler(void) 
{
    system_time++; // inc every 1ms
}

void init_systick(void) 
{
    SysTick_Config(SystemCoreClock / 1000); // 1ms systick interrupt
}

void delay_ms(int ms) // delay for blinking light test 
{
    for (int i = 0; i < ms * 8000; i++) {
        __NOP();
    }
}
