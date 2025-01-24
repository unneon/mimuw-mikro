#include <gpio.h>
#include <stm32.h>
#include "lis35de.h"

#define PCLK1_MHZ 16
#define PCLK1_HZ (PCLK1_MHZ * 1'000'000)

#define LED_RED_GPIO GPIOA
#define LED_RED_PIN 6
#define LED_GREEN_GPIO GPIOA
#define LED_GREEN_PIN 7

#define I2C_SCL_GPIO GPIOB
#define I2C_SCL_PIN 8
#define I2C_SDA_GPIO GPIOB
#define I2C_SDA_PIN 9
#define I2C_SPEED_HZ 100'000

#define LIS35DE_I2C_ADDR 0x1C
#define LIS35DE_INT1_GPIO GPIOA
#define LIS35DE_INT1_PIN 1

#define WAIT_FOR(condition) while (!(condition)) {}

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

static void led_initialize(void) {
    led_red_off();
    led_green_off();

    GPIOoutConfigure(LED_RED_GPIO, LED_RED_PIN, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL);
    GPIOoutConfigure(LED_GREEN_GPIO, LED_GREEN_PIN, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL);
}

static void timer_initialize(void) {
    // PSC is set as high as possible to reduce power usage (?), while cleanly
    // dividing 16 MHz into 250 Hz, which means 3 seconds from the task
    // statement translate precisely to 750 ticks.
    TIM3->PSC = 64000;
    TIM3->ARR = UINT16_MAX;

    // We will be using two compares to handle timeouts of red LED (single
    // click) and green LED (double click) respectively. However, interrupts in
    // the DIER register will only be enabled after we receive the click signal.
    TIM3->SR = ~(TIM_SR_CC1IF | TIM_SR_CC2IF);
    TIM3->EGR = TIM_EGR_UG;

    TIM3->CR1 = TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM3_IRQn);
}

static void timer_1_start(void) {
    TIM3->CCR1 = TIM3->CNT + 750;
    TIM3->DIER |= TIM_DIER_CC1IE;
}

static void timer_1_stop(void) {
    TIM3->DIER &= ~TIM_DIER_CC1IE;
}

static void timer_1_extend(void) {
    TIM3->CCR1 = (TIM3->CCR1 + 750) % (1 << 16);
}

static int timer_1_is_active(void) {
    return (TIM3->DIER & TIM_DIER_CC1IE) != 0;
}

static void timer_2_start(void) {
    TIM3->CCR2 = TIM3->CNT + 750;
    TIM3->DIER |= TIM_DIER_CC2IE;
}

static void timer_2_stop(void) {
    TIM3->DIER &= ~TIM_DIER_CC2IE;
}

static void timer_2_extend(void) {
    TIM3->CCR2 = (TIM3->CCR2 + 750) % (1 << 16);
}

static int timer_2_is_active(void) {
    return (TIM3->DIER & TIM_DIER_CC2IE) != 0;
}

static void i2c_initialize(void) {
    GPIOafConfigure(I2C_SCL_GPIO, I2C_SCL_PIN, GPIO_OType_OD, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_I2C1);
    GPIOafConfigure(I2C_SDA_GPIO, I2C_SDA_PIN, GPIO_OType_OD, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_I2C1);

    I2C1->CR1 = 0;
    I2C1->CCR = PCLK1_HZ / (I2C_SPEED_HZ << 1);
    I2C1->CR2 = PCLK1_MHZ;
    I2C1->TRISE = PCLK1_MHZ + 1;
    I2C1->CR1 |= I2C_CR1_PE;
}

static void accelerometer_write_register(unsigned register_number, unsigned value) {
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

static unsigned accelerometer_read_register(unsigned register_number) {
    I2C1->CR1 |= I2C_CR1_START;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_SB);
    I2C1->DR = LIS35DE_I2C_ADDR << 1;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_ADDR);
    I2C1->SR2;
    I2C1->DR = register_number;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_BTF);

    I2C1->CR1 |= I2C_CR1_START;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_SB);
    I2C1->DR = (LIS35DE_I2C_ADDR << 1) | 1;
    I2C1->CR1 &= ~I2C_CR1_ACK;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_ADDR);
    I2C1->SR2;

    I2C1->CR1 |= I2C_CR1_STOP;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_RXNE);

    return I2C1->DR;
}

static void accelerometer_initialize(void) {
    accelerometer_write_register(LIS35DE_CTRL1, LIS35DE_CTRL1_PD
        | LIS35DE_CTRL1_XEN | LIS35DE_CTRL1_YEN | LIS35DE_CTRL1_ZEN);
    accelerometer_write_register(LIS35DE_CTRL3, LIS35DE_CTRL3_I1CFG_CLICK);
    accelerometer_write_register(LIS35DE_CLICKCFG, LIS35DE_CLICKCFG_LIR
        | LIS35DE_CLICKCFG_DOUBLEZ | LIS35DE_CLICKCFG_DOUBLEY | LIS35DE_CLICKCFG_DOUBLEX
        | LIS35DE_CLICKCFG_SINGLEZ | LIS35DE_CLICKCFG_SINGLEY | LIS35DE_CLICKCFG_SINGLEX);
    accelerometer_write_register(LIS35DE_CLICKTIMELIMIT, UINT8_MAX);
    accelerometer_write_register(LIS35DE_CLICKLATENCY, UINT8_MAX);
    accelerometer_write_register(LIS35DE_CLICKWINDOW, UINT8_MAX);

    GPIOinConfigure(LIS35DE_INT1_GPIO, LIS35DE_INT1_PIN, GPIO_PuPd_NOPULL, EXTI_Mode_Interrupt, EXTI_Trigger_Rising);
    NVIC_EnableIRQ(EXTI1_IRQn);
}

static unsigned accelerometer_read_click(void) {
    return accelerometer_read_register(LIS35DE_CLICKSRC);
}

static int accelerometer_contains_single_click(unsigned click_source) {
    return (click_source & (LIS35DE_CLICKSRC_SINGLEX | LIS35DE_CLICKSRC_SINGLEY | LIS35DE_CLICKSRC_SINGLEZ)) != 0;
}

static int accelerometer_contains_double_click(unsigned click_source) {
    return (click_source & (LIS35DE_CLICKSRC_DOUBLEX | LIS35DE_CLICKSRC_DOUBLEY | LIS35DE_CLICKSRC_DOUBLEZ)) != 0;
}

int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN | RCC_APB1ENR_TIM3EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    __NOP();

    led_initialize();
    timer_initialize();
    i2c_initialize();
    accelerometer_initialize();

    for (;;);
}

void EXTI1_IRQHandler(void) {
    EXTI->PR = EXTI_PR_PR1;

    unsigned click_source = accelerometer_read_click();

    if (accelerometer_contains_single_click(click_source)) {
        if (!timer_1_is_active()) {
            timer_1_start();
            led_red_on();
        } else {
            timer_1_extend();
        }
    }

    if (accelerometer_contains_double_click(click_source)) {
        if (!timer_2_is_active()) {
            timer_2_start();
            led_green_on();
        } else {
            timer_2_extend();
        }
    }
}

void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_CC1IF) {
        timer_1_stop();
        led_red_off();
        TIM3->SR = ~TIM_SR_CC1IF;
    }

    if (TIM3->SR & TIM_SR_CC2IF) {
        timer_2_stop();
        led_green_off();
        TIM3->SR = ~TIM_SR_CC2IF;
    }
}
