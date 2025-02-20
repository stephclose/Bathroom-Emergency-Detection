#include <pager.h>

#define RFM69_REG_OPMODE 0x01
#define RFM69_REG_VERSION 0x10
#define RFM69_REG_IRQFLAGS1 0x27
#define RFM69_REG_IRQFLAGS2 0x28
#define STANDBY_MODE 0x04
#define TRANSMIT_MODE 0x0C

void enable_GPIOC(void){
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~(3U << (8 * 2)); 
    GPIOC->MODER |= (2U << (8 * 2)); 
    GPIOC->AFR[1] &= ~(0xF << ((8 - 8) * 4));
    GPIOC->AFR[1] |= (1U << ((8 - 8) * 4)); 

}

void enable_GPIOB(void){
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~(3U << (8 * 2));  //Clear mode bits for PB8
    GPIOB->MODER |= (1U << (8 * 2));   //Set PB8 as output
    GPIOB->MODER &= ~(3U << (0 * 2));  //Clear mode bits for PB0
    GPIOB->MODER |= (0U << (0 * 2));   //Set PB0 as input
    GPIOB->PUPDR &= ~(3U << (0 * 2));  //Clear pull-up/pull-down for PB0
    GPIOB->PUPDR |= (2U << (0 * 2));   //Set PB0 as pull-down
}

void init_TIM3(void) {
    // Enable TIM3 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 0;  //prescale value
    TIM3->ARR = 7999; //auto-reload value for desired frequency

    TIM3->CCMR2 &= ~(TIM_CCMR2_OC3M);
    TIM3->CCMR2 |= (6U << TIM_CCMR2_OC3M_Pos); //PWM mode 1
    TIM3->CCMR2 |= TIM_CCMR2_OC3PE; //preload register on OC3
    TIM3->CCR3 = (TIM3->ARR + 1) / 2; //50%
    TIM3->CCER |= TIM_CCER_CC3E; //output
    TIM3->CR1 |= TIM_CR1_CEN; //TIM3 counter
}

void spi_enable(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Enable SPI1 clock

    //SPI1 in Master mode, CPOL=0, CPHA=0, 8-bit data
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_1 | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR2 = 0;
    SPI1->CR1 |= SPI_CR1_SPE; //Enable SPI1

    //PB3(SCK), PB4(MISO), PB5(MOSI) as alternate function AF0(SPI1)
    GPIOB->MODER &= ~((3U << (3 * 2)) | (3U << (4 * 2)) | (3U << (5 * 2)));
    GPIOB->MODER |= ((2U << (3 * 2)) | (2U << (4 * 2)) | (2U << (5 * 2)));

    //AF0 for PB3, PB4, PB5
    GPIOB->AFR[0] &= ~((0xF << (3 * 4)) | (0xF << (4 * 4)) | (0xF << (5 * 4)));

}

uint8_t spi_transfer(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE)); //TX buffer is empty
    SPI1->DR = data; //Send data
    while (!(SPI1->SR & SPI_SR_RXNE)); //Wait to receive data
    return SPI1->DR;
}

void NSS_Low(void) {
    GPIOB->ODR &= ~(1U << 6);
}

void NSS_High(void) {
    GPIOB->ODR |= (1U << 6);
}

void write_rfm69_register(uint8_t addr, uint8_t value) {
    NSS_Low();
    spi_transfer(addr | 0x80);
    spi_transfer(value);
    NSS_High();
}

uint8_t read_rfm69_register(uint8_t addr) {
    NSS_Low();
    spi_transfer(addr & 0x7F); //clear MSB
    uint8_t value = spi_transfer(0x00);
    NSS_High();
    return value;
}

void set_rfm69_mode(uint8_t mode) {
    write_rfm69_register(RFM69_REG_OPMODE, (read_rfm69_register(RFM69_REG_OPMODE) & 0xE3) | mode);
    while (!(read_rfm69_register(0x27) & 0x80));
}

void send_rfm69_packet(uint8_t *data, uint8_t length) {
    if (length > 66) {
        length = 66; //RFM69 FIFO size
    }

    set_rfm69_mode(STANDBY_MODE);

    NSS_Low();
    spi_transfer(0x80); //FIFO write command
    spi_transfer(length);
    for (uint8_t i = 0; i < length; i++) {
        spi_transfer(data[i]);
    }
    NSS_High();
    set_rfm69_mode(TRANSMIT_MODE);
    while (!(read_rfm69_register(0x28) & 0x40));

    set_rfm69_mode(STANDBY_MODE);
}


void init_exti() {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //enable SYSCFG clock
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB; //map PB0 to EXTI0
    EXTI->IMR |= EXTI_IMR_IM0;  //unmask EXTI0
    EXTI->RTSR |= EXTI_RTSR_TR0; //rising edge trigger
    NVIC_EnableIRQ(EXTI0_1_IRQn); //enable interrupt
}

void EXTI0_1_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) { //check if EXTI0 triggered
        EXTI->PR |= EXTI_PR_PR0; //clear interrupt flag

        GPIOB->ODR |= (1U << 8); //set PB8 high
        uint8_t received = spi_transfer(0x00);  //read received data
        if (received) {
            //process received data
        }
        GPIOB->ODR &= ~(1U << 8); //set PB8 low
    }
}

int main(void) {
    enable_GPIOC();
    init_TIM3();
    //enable_GPIOB();
    //spi_enable();
    //init_exti();

    //uint8_t version = read_rfm69_register(0x10);
    //if (version == 0x24) {
        // Version matches; proceed with initialization
    //} else {
        // Handle version mismatch
   //}

    //uint8_t message[] = "HELLO";
    //send_rfm69_packet(message, sizeof(message) - 1);

    while (1) {
    }
}

