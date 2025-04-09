#include "stm32f0xx.h"
#include "rf.h"
#include "spi.h"
#include "clock.h"
#include <string.h>
#include <stdio.h>

extern volatile uint32_t system_time;

#define NSS   4   // PA4
#define DIO0  8   // PA8
#define RST   9   // PA9

#define TIMEOUT 100
#define MAX_PAYLOAD_LEN 64

// SX1276 register addresses
#define REG_FIFO             0x00
#define REG_OPMODE           0x01
#define REG_FRF_MSB          0x06
#define REG_FRF_MID          0x07
#define REG_FRF_LSB          0x08
#define REG_PA_CONFIG        0x09
#define REG_LNA              0x0C
#define REG_FIFO_ADDR_PTR    0x0D
#define REG_FIFO_TX_BASE     0x0E
#define REG_FIFO_RX_BASE     0x0F
#define REG_FIFO_RX_CURRENT  0x10
#define REG_IRQ_FLAGS        0x12
#define REG_RX_NB_BYTES      0x13
#define REG_PKT_RSSI_VALUE   0x1A
#define REG_MODEM_CONFIG1    0x1D
#define REG_MODEM_CONFIG2    0x1E
#define REG_PREAMBLE_MSB     0x20
#define REG_PREAMBLE_LSB     0x21
#define REG_PAYLOAD_LENGTH   0x22
#define REG_MODEM_CONFIG3    0x26
#define REG_FREQ_ERROR       0x28
#define REG_DETECTION_OPT    0x31
#define REG_DETECTION_THRESH 0x37
#define REG_SYNC_WORD        0x39
#define REG_DIO_MAPPING1     0x40
#define REG_VERSION          0x42

// LoRa mode op values
#define LONG_RANGE_MODE      0x80
#define SLEEP_MODE           0x00
#define STDBY_MODE           0x01
#define TX_MODE              0x03

// --- SPI Helpers ---
static inline void NSS_Low(void) {
    GPIOA->BRR = (1u << NSS);
}
static inline void NSS_High(void) {
    GPIOA->BSRR = (1u << NSS);
}
static void write_reg(uint8_t addr, uint8_t val) {
    NSS_Low();
    spi_transfer(addr | 0x80);
    spi_transfer(val);
    NSS_High();
}
static uint8_t read_reg(uint8_t addr) {
    NSS_Low();
    spi_transfer(addr & 0x7F);
    uint8_t val = spi_transfer(0x00);
    NSS_High();
    return val;
}

// --- Initialization ---
void rf_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // NSS (PA4)
    GPIOA->MODER &= ~(0x3 << (NSS * 2));
    GPIOA->MODER |=  (0x1 << (NSS * 2));
    NSS_High();

    // RESET (PA9)
    GPIOA->MODER &= ~(0x3 << (RST * 2));
    GPIOA->MODER |=  (0x1 << (RST * 2));

    // DIO0 (PA8) input
    GPIOA->MODER &= ~(0x3 << (DIO0 * 2));

    // Reset pulse
    GPIOA->BRR = (1u << RST);
    delay_ms(10);
    GPIOA->BSRR = (1u << RST);
    delay_ms(10);

    // Enter LoRa sleep mode
    write_reg(REG_OPMODE, LONG_RANGE_MODE | SLEEP_MODE);
    delay_ms(10);

    // Set standby mode
    write_reg(REG_OPMODE, LONG_RANGE_MODE | STDBY_MODE);

    // Set frequency to 915 MHz
    write_reg(REG_FRF_MSB, 0xE4);
    write_reg(REG_FRF_MID, 0xC0);
    write_reg(REG_FRF_LSB, 0x00);

    // TX base address
    write_reg(REG_FIFO_TX_BASE, 0x00);
    write_reg(REG_FIFO_ADDR_PTR, 0x00);

    // LNA boost
    write_reg(REG_LNA, 0x23);

    // Set output power to max
    write_reg(REG_PA_CONFIG, 0x8F); // PA_BOOST, max power

    // Modem config: BW=125kHz, CR=4/5, Explicit header
    write_reg(REG_MODEM_CONFIG1, 0x72);
    write_reg(REG_MODEM_CONFIG2, 0x74); // SF=7
    write_reg(REG_MODEM_CONFIG3, 0x04); // LowDataRateOptimize off

    // Preamble length
    write_reg(REG_PREAMBLE_MSB, 0x00);
    write_reg(REG_PREAMBLE_LSB, 0x08);

    // Set Sync Word to 0x12 (public)
    write_reg(REG_SYNC_WORD, 0x12);

    // Clear IRQ flags
    write_reg(REG_IRQ_FLAGS, 0xFF);

    printf("[RF INIT] SX1276 Version: 0x%02X\r\n", read_reg(REG_VERSION));
    printf("[RF INIT] SX1276/RFM95W Init complete.\r\n");
}

// --- Transmit ---
void rf_transmit(const char *msg) 
{
    uint8_t len = (uint8_t)strlen(msg);
    if (len > MAX_PAYLOAD_LEN) len = MAX_PAYLOAD_LEN;

    write_reg(REG_OPMODE, LONG_RANGE_MODE | STDBY_MODE);
    write_reg(REG_FIFO_ADDR_PTR, 0x00);

    NSS_Low();
    spi_transfer(REG_FIFO | 0x80);
    for (uint8_t i = 0; i < len; i++) {
        spi_transfer(msg[i]);
    }
    NSS_High();

    write_reg(REG_PAYLOAD_LENGTH, len);
    write_reg(REG_OPMODE, LONG_RANGE_MODE | TX_MODE);

    printf("[TX] Sending: \"%s\"\r\n", msg);

    uint32_t start = system_time;
    while (!(GPIOA->IDR & (1 << DIO0))) {
        if (system_time - start > TIMEOUT) {
            printf("[TX ERROR] Timeout waiting for TX done.\r\n");
            return;
        }
    }

    // Clear IRQ
    write_reg(REG_IRQ_FLAGS, 0xFF);
    printf("[TX] Done.\r\n");
}



// #include "stm32f0xx.h"
// #include "rf.h"
// #include "spi.h"
// #include "clock.h"
// #include <string.h>
// #include <stdio.h>

// // Definitions for registers and constants.
// #define NSS 4        // PA4 is used as NSS.
// #define DI0 8        // PA8 is used as DIO0.
// #define RST 9        // PA9 is used as RESET.
// #define TIMEOUT 50
// #define RFM69_REG_FIFO 0x00
// #define RF_OPMODE_STANDBY 0x04
// #define RFM69_REG_VERSION 0x10
// #define RFM69_REG_PREAMBLEMSB 0x2C
// #define RFM69_REG_PREAMBLELSB 0x2D
// #define RFM69_REG_SYNCCONFIG  0x2E
// #define RFM69_REG_SYNCVALUE1  0x2F
// #define RFM69_REG_SYNCVALUE2  0x30
// #define RFM69_REG_PACKETCONFIG1 0x37
// #define RFM69_REG_PAYLOADLENGTH 0x38
// #define RFM69_REG_FIFOTHRES   0x3C
// #define RFM69_REG_PALEVEL     0x11
// #define RFM69_REG_LNA         0x18
// #define RFM69_REG_RXBW        0x19
// #define RFM69_REG_RSSITHRESH  0x29
// #define RFM69_REG_DIOMAPPING1 0x25
// #define RFM69_REG_IRQFLAGS1   0x27
// #define RF_OPMODE_SEQUENCER_ON 0x80
// #define RF_OPMODE_LISTEN_OFF   0x00
// #define RFM69_REG_OPMODE      0x01
// #define RFM69_REG_FRFMSB      0x07
// #define RFM69_REG_FRFMID      0x08
// #define RFM69_REG_FRFLSB      0x09
// #define RF_OPMODE_TX         0x0C
// #define RF_OPMODE_RX         0x10

// // Transmission settings.
// #define TX_ATTEMPTS 3
// #define TX_DELAY_MS 100  // 100 ms delay between transmissions

// // Debug-enabled inline functions for toggling NSS.
// static inline void NSS_Low(void) {
//     printf("[RF SPI] NSS LOW\r\n");
//     GPIOA->BRR = (1u << NSS);
//     for (volatile int i = 0; i < 100; i++) __NOP();
// }
// static inline void NSS_High(void) {
//     for (volatile int i = 0; i < 100; i++) __NOP();
//     GPIOA->BSRR = (1u << NSS);
//     printf("[RF SPI] NSS HIGH\r\n");
// }

// // Forward declarations of internal helper functions.
// static uint8_t rfm_readReg(uint8_t addr);
// static void    rfm_writeReg(uint8_t addr, uint8_t val);
// static void    rfm_setMode(uint8_t mode);
// static uint8_t rfm_waitForModeReady(uint32_t msTimeout);

// // PUBLIC FUNCTION: Print chip version.
// void rf_printVersion(void)
// {
//     uint8_t ver = rfm_readReg(RFM69_REG_VERSION);
//     uint8_t full_rev = (ver >> 4) & 0x0F;
//     uint8_t metal_rev = ver & 0x0F;
//     printf("[RF] RegVersion: 0x%02X (Full Revision: %d, Metal Mask Revision: %d)\r\n",
//            ver, full_rev, metal_rev);
// }

// // PUBLIC FUNCTION: RF initialization routine.
// void rf_init(void)
// {
//     printf("[RF INIT] Starting RF module initialization...\r\n");
    
//     // Enable clock for GPIOA (for NSS, RST, DI0).
//     RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    
//     // Configure NSS (PA4) as output.
//     GPIOA->MODER &= ~(0x3 << (NSS * 2));
//     GPIOA->MODER |= (0x1 << (NSS * 2));
//     NSS_High();

//     // Configure RST (PA9) as output.
//     GPIOA->MODER &= ~(0x3 << (RST * 2));
//     GPIOA->MODER |= (0x1 << (RST * 2));
    
//     // Configure DI0 (PA8) as input.
//     GPIOA->MODER &= ~(0x3 << (DI0 * 2));

//     // Reset the RF module.
//     printf("[RF INIT] Resetting RF module...\r\n");
//     GPIOA->BRR = (1u << RST);    // Set RST low.
//     for (volatile int i = 0; i < 30000; i++) __NOP();
//     GPIOA->BSRR = (1u << RST);   // Set RST high.
//     for (volatile int i = 0; i < 30000; i++) __NOP();
//     printf("[RF INIT] RF module reset complete.\r\n");

//     // Enter Standby mode.
//     rfm_setMode(RF_OPMODE_STANDBY);
//     if (rfm_waitForModeReady(50))
//         printf("[RF INIT] Standby mode reached.\r\n");
//     else
//         printf("[RF INIT ERROR] Standby mode not reached.\r\n");

//     // Read and print the version register.
//     uint8_t ver = rfm_readReg(RFM69_REG_VERSION);
//     printf("[RF INIT] RFM69 Version = 0x%02X (expected: 0x24)\r\n", ver);

//     printf("[RF INIT] Minimal RF initialization complete.\r\n");
// }

// // PUBLIC FUNCTION: Transmit a message using the RF module three times.
// void rf_transmit(const char *text)
// {
//     for (int attempt = 0; attempt < TX_ATTEMPTS; attempt++) {
//         printf("[TX] Attempt %d/%d: Starting RF Transmission...\r\n", attempt + 1, TX_ATTEMPTS);
//         uint8_t length = (uint8_t)strlen(text);
//         if (length > 60)
//             length = 60;  // Limit payload length for demo

//         // Step 1: Switch to Standby mode.
//         printf("[TX] Switching to Standby mode...\r\n");
//         rfm_setMode(RF_OPMODE_STANDBY);
//         if (rfm_waitForModeReady(50))
//             printf("[TX] Standby mode confirmed.\r\n");
//         else
//             printf("[TX ERROR] Standby mode timeout.\r\n");

//         // Step 2: Write payload to FIFO.
//         printf("[TX] Writing payload to FIFO...\r\n");
//         NSS_Low();
//         spi_transfer(RFM69_REG_FIFO | 0x80);  // FIFO write command.
//         spi_transfer(length);                 // Write payload length.
//         for (uint8_t i = 0; i < length; i++) {
//             spi_transfer(text[i]);
//         }
//         NSS_High();
//         printf("[TX] Payload '%s' written to FIFO.\r\n", text);

//         // Step 3: Switch to TX mode.
//         printf("[TX] Switching to Transmit mode...\r\n");
//         rfm_setMode(RF_OPMODE_TX);
//         uint8_t opmode = rfm_readReg(RFM69_REG_OPMODE);
//         printf("[TX] RegOpMode after TX command: 0x%02X\r\n", opmode);

//         // Step 4: Wait for Packet Sent confirmation.
//         printf("[TX] Waiting for Packet Sent confirmation (monitoring DIO0)...\r\n");
//         uint32_t start = system_time;
//         while (1) {
//             if (GPIOA->IDR & (1 << DI0)) {  // If DIO0 is high...
//                 uint8_t flags2 = rfm_readReg(0x28);  // Read IRQFLAGS2 register.
//                 if (flags2 & 0x08)
//                     printf("[TX] Packet Sent confirmed (IRQFLAGS2=0x%02X).\r\n", flags2);
//                 else
//                     printf("[TX ERROR] DIO0 high but PacketSent bit not set (IRQFLAGS2=0x%02X).\r\n", flags2);
//                 break;
//             }
//             if ((system_time - start) > TIMEOUT) {
//                 printf("[TX ERROR] Timeout waiting for Packet Sent confirmation.\r\n");
//                 break;
//             }
//         }

//         // Step 5: Return to Standby mode.
//         printf("[TX] Returning to Standby mode...\r\n");
//         rfm_setMode(RF_OPMODE_STANDBY);
//         if (rfm_waitForModeReady(50))
//             printf("[TX] Successfully returned to Standby mode.\r\n");
//         else
//             printf("[TX ERROR] Failed to return to Standby mode.\r\n");

//         // If not the last attempt, wait 100 ms before the next transmission.
//         if (attempt < TX_ATTEMPTS - 1)
//             delay_ms(TX_DELAY_MS);
//     }
// }

// // PUBLIC FUNCTION: Test write to a register.
// void rf_testWriteReg(uint8_t reg, uint8_t val) {
//     printf("[RF TEST] Writing 0x%02X to register 0x%02X...\r\n", val, reg);
//     rfm_writeReg(reg, val);
//     uint8_t check = rfm_readReg(reg);
//     printf("[RF TEST] Register 0x%02X now = 0x%02X\r\n", reg, check);
// }

// //////////////////////
// // Static functions //
// //////////////////////

// static uint8_t rfm_readReg(uint8_t addr)
// {
//     NSS_Low();
//     for (volatile int d = 0; d < 50; d++) __NOP();
//     spi_transfer(addr & 0x7F); // Read command (MSB cleared)
//     uint8_t val = spi_transfer(0x00);
//     NSS_High();
//     printf("[RF] Read Reg 0x%02X = 0x%02X\r\n", addr, val);
//     return val;
// }

// static void rfm_writeReg(uint8_t addr, uint8_t val)
// {
//     NSS_Low();
//     for (volatile int d = 0; d < 50; d++) __NOP();
//     spi_transfer(addr | 0x80); // Write command (MSB set)
//     spi_transfer(val);
//     NSS_High();
//     printf("[RF] Wrote 0x%02X to Reg 0x%02X\r\n", val, addr);
// }

// static void rfm_setMode(uint8_t mode)
// {
//     // Compose the operation mode command with Sequencer enabled and Listen disabled.
//     uint8_t op = RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | (mode & 0x1C);
//     printf("[RF] Setting OPMODE to 0x%02X\r\n", op);
//     rfm_writeReg(RFM69_REG_OPMODE, op);
// }

// static uint8_t rfm_waitForModeReady(uint32_t msTimeout)
// {
//     uint32_t start = system_time;
//     while (1) {
//         uint8_t irq1 = rfm_readReg(RFM69_REG_IRQFLAGS1);
//         if (irq1 & 0x80) {
//             printf("[RF] ModeReady confirmed.\r\n");
//             return 1;
//         }
//         if ((system_time - start) > msTimeout) {
//             printf("[RF WARNING] ModeReady timeout after %lu ms, proceeding...\r\n", msTimeout);
//             return 0;
//         }
//     }
// }
