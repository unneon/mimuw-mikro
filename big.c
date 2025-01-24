#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include "lis35de.h"

#define I2C_SCL_GPIO GPIOB
#define I2C_SCL_PIN 8
#define I2C_SDA_GPIO GPIOB
#define I2C_SDA_PIN 9

#define LED_RED_GPIO GPIOA
#define LED_RED_PIN 6
#define LED_GREEN_GPIO GPIOA
#define LED_GREEN_PIN 7

#define I2C_SPEED_HZ 100000

#define HSI_HZ 16000000U
#define PCLK1_HZ HSI_HZ
#define BAUD 9600U
#define PCLK1_MHZ 16

#define LIS35DE_I2C_ADDR 0x1C

#define WAIT_FOR(condition) while (!(condition)) {}

#define USART_BUFFER_CAPACITY 8192

static char usart_buffer_data[USART_BUFFER_CAPACITY];

static void usart_force_send(int length) {
    DMA1_Stream6->M0AR = (uint32_t) usart_buffer_data;
    DMA1_Stream6->NDTR = length;
    DMA1_Stream6->CR |= DMA_SxCR_EN;
}

static void accelerometer_write_register(int register_number, unsigned char value) {
    I2C1->CR1 |= I2C_CR1_START;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_SB);
    I2C1->DR = LIS35DE_I2C_ADDR << 1;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_ADDR);
    I2C1->SR2;

    I2C1->DR = register_number;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_TXE);
    I2C1->DR = value;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_BTF);
    I2C1->CR1 |= I2C_CR1_STOP;
}

static unsigned char accelerometer_read_register(int register_number) {
    I2C1->CR1 |= I2C_CR1_START;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_SB);
    I2C1->DR = LIS35DE_I2C_ADDR << 1;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_ADDR);
    I2C1->SR2;
    I2C1->DR = register_number;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_BTF);

    I2C1->CR1 |= I2C_CR1_START;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_SB);
    I2C1->DR = LIS35DE_I2C_ADDR << 1 | 1;
    I2C1->CR1 &= ~I2C_CR1_ACK;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_ADDR);
    I2C1->SR2;

    I2C1->CR1 |= I2C_CR1_STOP;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_RXNE);

    return *(signed char*)(&I2C1->DR);
}

static int accelerometer_read_x(void) {
    return (signed char) accelerometer_read_register(LIS35DE_OUTX);
}

static int accelerometer_read_y(void) {
    return (signed char) accelerometer_read_register(LIS35DE_OUTY);
}

static int accelerometer_read_z(void) {
    return (signed char) accelerometer_read_register(LIS35DE_OUTZ);
}

static int debug_string(const char* text, int bytes_written) {
    for (;*text;++text) {
        usart_buffer_data[bytes_written++] = *text;
    }
    return bytes_written;
}

static int debug_int(int value, int bytes_written) {
    if (value < 0) {
        usart_buffer_data[bytes_written++] = '-';
        value = -value;
    } else {
        usart_buffer_data[bytes_written++] = ' ';
    }
    int digit = 1;
    while (10 * digit <= value)
        digit *= 10;
    if (digit < 100)
        usart_buffer_data[bytes_written++] = ' ';
    if (digit < 10)
        usart_buffer_data[bytes_written++] = ' ';
    while (digit > 0) {
        usart_buffer_data[bytes_written++] = '0' + value / digit;
        value %= digit;
        digit /= 10;
    }
    return bytes_written;
}

static void accelerometer_debug(void) {
    int x_axis = accelerometer_read_x();
    int y_axis = accelerometer_read_y();
    int z_axis = accelerometer_read_z();
    // int click_src = accelerometer_read_register(LIS35DE_CLICKSRC);

    int bytes_written = 0;
    bytes_written = debug_string("x: ", bytes_written);
    bytes_written = debug_int(x_axis, bytes_written);
    bytes_written = debug_string(", y: ", bytes_written);
    bytes_written = debug_int(y_axis, bytes_written);
    bytes_written = debug_string(", z: ", bytes_written);
    bytes_written = debug_int(z_axis, bytes_written);
    // bytes_written = debug_string(", clicknya: ", bytes_written);
    // bytes_written = debug_int(click_src, bytes_written);
    bytes_written = debug_string("\r\n", bytes_written);
    usart_force_send(bytes_written);
}

static void led_red_on(void) {
    LED_RED_GPIO->BSRR = 1 << (LED_RED_PIN + 16);
}

static void led_red_off(void) {
    LED_RED_GPIO->BSRR = 1 << LED_RED_PIN;
}

static void led_green_on(void) {
    LED_GREEN_GPIO->BSRR = 1 << (LED_GREEN_PIN + 16);
}

static void led_green_off(void) {
    LED_GREEN_GPIO->BSRR = 1 << LED_GREEN_PIN;
}

int main() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    __NOP();

    DMA1_Stream6->CR = 4u << 25 | DMA_SxCR_PL_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;
    DMA1_Stream6->PAR = (uint32_t) &USART2->DR;
    DMA1->HIFCR = DMA_HIFCR_CTCIF6;
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    GPIOafConfigure(GPIOA, 2, GPIO_OType_PP, GPIO_Fast_Speed, GPIO_PuPd_NOPULL, GPIO_AF_USART2);
    GPIOafConfigure(I2C_SCL_GPIO, I2C_SCL_PIN, GPIO_OType_OD, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_I2C1);
    GPIOafConfigure(I2C_SDA_GPIO, I2C_SDA_PIN, GPIO_OType_OD, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_I2C1);

    USART2->CR1 = USART_CR1_TE;
    USART2->CR2 = 0;
    USART2->CR3 = USART_CR3_DMAT;
    USART2->BRR = (PCLK1_HZ + (BAUD / 2U)) / BAUD;
    USART2->CR1 |= USART_CR1_UE;

    I2C1->CR1 = 0;
    I2C1->CCR = (PCLK1_MHZ * 1000000) / (I2C_SPEED_HZ << 1);
    I2C1->CR2 = PCLK1_MHZ;
    I2C1->TRISE = PCLK1_MHZ + 1;
    I2C1->CR1 |= I2C_CR1_PE;

    accelerometer_write_register(LIS35DE_CTRL1, LIS35DE_CTRL1_PD | LIS35DE_CTRL1_XEN | LIS35DE_CTRL1_YEN | LIS35DE_CTRL1_ZEN);
    accelerometer_write_register(LIS35DE_CTRL3, LIS35DE_CTRL3_I1CFG_CLICK);
    accelerometer_write_register(LIS35DE_CLICKCFG, LIS35DE_CLICKCFG_SINGLEZ | LIS35DE_CLICKCFG_SINGLEY | LIS35DE_CLICKCFG_SINGLEX);
    accelerometer_write_register(LIS35DE_CLICKTIMELIMIT, (1 << 8) - 1);
    accelerometer_write_register(LIS35DE_CLICKLATENCY, (1 << 8) - 1);
    accelerometer_write_register(LIS35DE_CLICKWINDOW, (1 << 8) - 1);

    WAIT_FOR((DMA1_Stream6->CR & DMA_SxCR_EN) == 0 && (DMA1->HISR & DMA_HISR_TCIF6) == 0);
    accelerometer_debug();

    GPIOinConfigure(GPIOA, 1, GPIO_PuPd_NOPULL, EXTI_Mode_Interrupt, EXTI_Trigger_Rising);
    NVIC_EnableIRQ(EXTI1_IRQn);

    led_red_off();
    led_green_off();

    GPIOoutConfigure(LED_RED_GPIO, LED_RED_PIN, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL);
    GPIOoutConfigure(LED_GREEN_GPIO, LED_GREEN_PIN, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL);

    // PSC is set as high as possible to reduce power usage (?), while cleanly
    // dividing 16 MHz into 250 Hz, which means 3 seconds from the task
    // statement translate precisely to 750 ticks.
    TIM3->PSC = 64000;
    TIM3->ARR = (1 << 16) - 1;

    // We will be using two compares to handle timeouts of red LED (single
    // click) and green LED (double click) respectively. However, interrupts in
    // the DIER register will only be enabled after we receive the click signal.
    TIM3->SR = ~(TIM_SR_CC1IF | TIM_SR_CC2IF);
    TIM3->EGR = TIM_EGR_UG;

    TIM3->CR1 = TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM3_IRQn);

    for (;;) {
        __NOP();
    }
}

void EXTI1_IRQHandler(void) {
    EXTI->PR = EXTI_PR_PR1;

    if ((TIM3->DIER & TIM_DIER_CC1IE) == 0) {
        TIM3->DIER = TIM_DIER_CC1IE;
        TIM3->CCR1 = TIM3->CNT + 750;
        led_red_on();
    } else {
        TIM3->CCR1 = (TIM3->CCR1 + 750) % (1 << 16);
    }
}

void TIM3_IRQHandler(void) {
    TIM3->SR = ~TIM_SR_CC1IF;

    TIM3->DIER &= ~TIM_DIER_CC1IE;
    led_red_off();
}

void DMA1_Stream6_IRQHandler() {
    if (DMA1->HISR & DMA_HISR_TCIF6) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF6;
        accelerometer_debug();
    }
}
