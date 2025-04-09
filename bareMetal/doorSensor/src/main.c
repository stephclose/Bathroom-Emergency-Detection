#include "stm32f0xx.h"
#include "clock.h"
#include "reedSwitch.h"
#include "uart.h"
//#include "rf.h"
//#include "spi.h"
#include "rfm9x.h"
#include <stdio.h>
#include <string.h>

static volatile uint8_t doorState = 1; // door starts open
static volatile uint8_t lastChangeTime = 0;
extern volatile uint32_t system_time; // update by systick
#define DEBOUNCE 50 // 50 ms debounce delay

#define NSS_PIN   4  // PA4
#define RST_PIN   9  // PA9
#define RFM9X_REG_VERSION  0x42
#define EXPECTED_VERSION   0x12

int main(void) 
{
    internal_clock(); // configure hsi clock
    init_systick(); // initialize systick timer
    uart_init(); // uart initalization 
    setbuf(stdout, NULL);  // disable buffering on stdout for immediate prints
 
    //spi_init();
    rfm9x_reset();
    init_spi1_lora();      // if you are using the dedicated LoRa SPI config
    //rfm9x_set_frequency(915);     // 915 MHz US
    //rfm9x_set_tx_power(14);       // dBm
    //rfm9x_set_mode(0x01);         // Standby mode
    nano_wait(5000000);    // short 
    // rfm9x_set_frequency(915);
    // rfm9x_set_tx_power(14);
    // rfm9x_set_mode(0x03); 


    printf("\n=== RFM69 Register Check ===\n");
    test_rfm9x_registers();

    test_rfm9x_basic_communication();
    nano_wait(1000000);


    // if (ENABLE_RF_TRANSMIT == 1) printf("RFM69 init done!\r\n");
    printf("System: Initialized. Initializing Door sensor.\r\n");

    reedSwitch_init(); // initalize reed switch gpio
    // while(1)
    // {
    //     rfm9x_read_register(0x02);
    //     nano_wait(500000);

    // }
    //reedSwitchEnableInt(); // reed switch EXTI

    //delay_ms(100);
    //printf("System Initialized. Monitoring Door State...\r\n");
    //delay_ms(200);

    // PC6 LED output for reed switch. When aligned with magnet lights red
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~(0x3 << (6 * 2)); // Set PC6 as output
    GPIOC->MODER |= (0x1 << (6 * 2)); // Clear bit for output mode

    // if (ENABLE_RF_TRANSMIT == 1) rf_testWriteReg(0x05, 0xAA);

    while (1) 
    {
        uint32_t currentTime = system_time;

        // uint8_t version = rfm_readReg(RFM69_REG_VERSION);

        //printf("[TEST] RFM69 RegVersion = 0x%02X (expected 0x%02X)\r\n",
              // version, EXPECTED_VERSION);


        if ((currentTime - lastChangeTime) >= DEBOUNCE)
        {
            uint8_t newState = getState(); // 1=OPEN, 0=CLOSED
            if (newState != doorState)
            {
                doorState = newState;
                lastChangeTime = currentTime;

                char message[32];
                if (doorState == 0)
                {
                    GPIOC->ODR |= (1 << 6);
                    printf("Door ID: 1, CLOSED.\r\n");
                    sprintf(message, "Door ID: 1, CLOSED");
                    const char msg[] = "Door ID:1 CLOSED";
                    //rfm9x_send_packet((uint8_t *)msg, strlen(msg));
                    uint8_t buffer[64];
                    strcpy((char*)buffer, msg);
                    uart_send_string("Sending packet...\r\n");
                    rfm9x_send_packet(buffer, strlen((char*)buffer));
                    uart_send_string("Sent!\r\n");
            

                    //if (ENABLE_RF_TRANSMIT)
                        // rf_transmit("Door CLOSED");
                }
                else
                {
                    GPIOC->ODR &= ~(1 << 6); // turn led on 
                    printf("Door ID: 1, OPEN.\r\n");
                    sprintf(message, "Door ID: 1, OPEN");
                    const char msg[] = "Door ID:1 OPEN";
                    //rfm9x_send_packet((uint8_t *)msg, strlen(msg));

                    //if (ENABLE_RF_TRANSMIT)
                        // rf_transmit("Door OPEN");
                }

                // //rf_transmit(message);
                // printf("[STATE CHANGE] %s\r\n", message);
                // if (ENABLE_RF_TRANSMIT)
                //     rf_transmit(message);
            }
        }
        delay_ms(1000);

    }
}


// static void init_gpio_rfm(void)
// {
//     RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

//     // NSS pin (PA4) as output, set High
//     GPIOA->MODER &= ~(3U << (NSS_PIN * 2));
//     GPIOA->MODER |=  (1U << (NSS_PIN * 2));
//     rfm_nss_high();

//     // RST pin (PA9) as output
//     GPIOA->MODER &= ~(3U << (RST_PIN * 2));
//     GPIOA->MODER |=  (1U << (RST_PIN * 2));
// }

// Provides a short delay loop
static void short_delay(volatile int count)
{
    // Just burn CPU cycles
    while(count--) {
        __NOP();
    }
}

// // Resets the RFM69 by pulling RESET low then high
// static void reset_rfm69(void)
// {
//     printf("[TEST] Resetting the RFM69...\r\n");

//     // Drive RESET (PA9) LOW
//     GPIOA->BRR = (1U << RST_PIN);
//     short_delay(100000); // ~ some microseconds or ms

//     // Drive RESET (PA9) HIGH
//     GPIOA->BSRR = (1U << RST_PIN);
//     short_delay(100000); // ~ some microseconds or ms

//     printf("[TEST] RFM69 reset toggled.\r\n");

//     // Per datasheet, the RFM69 might need up to 100 us for reset to complete,
//     // but let's wait a few ms to be sure
//     delay_ms(5);
// }

// // Pull NSS (PA4) low with small delay
// static void rfm_nss_low(void)
// {
//     GPIOA->BRR = (1U << NSS_PIN);
//     short_delay(50);
// }

// // Drive NSS (PA4) high with small delay
// static void rfm_nss_high(void)
// {
//     short_delay(50);
//     GPIOA->BSRR = (1U << NSS_PIN);
//     short_delay(50);
// }

// // Basic read from RFM69 register
// static uint8_t rfm_readReg(uint8_t addr)
// {
//     rfm_nss_low();

//     // Send address with MSB=0 => read
//     // spi_transfer(addr & 0x7F);
//     // Next byte read from MISO
//     // uint8_t val = spi_transfer(0x00);

//     rfm_nss_high();
//     return val;
// }  
/*
    int main(void)
    {
        // 1) System clock & SysTick
        internal_clock();    // Should configure PLL => 48 MHz
        init_systick();      // Increments system_time
    
        // 2) Initialize UART
        uart_init();         // Configure PA2 => TX, PA3 => RX, 115200 @ 48 MHz
    
        // 3) Disable buffering so we see prints immediately
        setbuf(stdout, NULL);
    
        // 4) Simple test loop
        while (1)
        {
            printf("Hello from the minimal test\r\n");
            // ~0.5s delay
            for (volatile int i = 0; i < 800000; i++);
        }
    }
*/