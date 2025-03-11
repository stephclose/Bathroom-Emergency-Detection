#include "stm32f0xx.h"

void internal_clock(){
    RCC->CR &= ~RCC_CR_HSEON;
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE_DIV1;
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMUL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI_DIV2 | RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV1 | RCC_CFGR_PLLMUL12);
    RCC->CR |= RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_PLLRDY) == 0);
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL);
}