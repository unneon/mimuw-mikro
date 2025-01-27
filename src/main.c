#include <gpio.h>
#include <stm32.h>
#include "i2c.h"
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

static unsigned char global_clicksrc;
static unsigned global_sleep_semaphore = 0;

static void accelerometer_initialize_1(void);
static void accelerometer_initialize_2(void);
static void accelerometer_initialize_3(void);
static void accelerometer_initialize_4(void);
static void on_read_clicksrc(void);

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

// TODO: Document how timers work in this program?

// TODO: Extract 750 (750 timer ticks = 3 seconds) to a constant.

static void timer_1_start(void) {
    TIM3->CCR1 = TIM3->CNT + 750;
    TIM3->DIER |= TIM_DIER_CC1IE;
}

static void timer_1_stop(void) {
    TIM3->DIER &= ~TIM_DIER_CC1IE;
}

static void timer_1_extend(void) {
    TIM3->CCR1 = (TIM3->CCR1 + 750) % UINT16_MAX;
    if (TIM3->SR & TIM_SR_CC1IF) {
        TIM3->SR = ~TIM_SR_CC1IF;
    }
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
    TIM3->CCR2 = (TIM3->CCR2 + 750) % UINT16_MAX;
    if (TIM3->SR & TIM_SR_CC2IF) {
        TIM3->SR = ~TIM_SR_CC2IF;
    }
}

static int timer_2_is_active(void) {
    return (TIM3->DIER & TIM_DIER_CC2IE) != 0;
}

static void i2c_initialize(void) {
    GPIOafConfigure(I2C_SCL_GPIO, I2C_SCL_PIN, GPIO_OType_OD, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_I2C1);
    GPIOafConfigure(I2C_SDA_GPIO, I2C_SDA_PIN, GPIO_OType_OD, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_I2C1);

    I2C1->CR1 = 0;
    I2C1->CCR = PCLK1_HZ / (I2C_SPEED_HZ << 1);
    I2C1->CR2 = PCLK1_MHZ | I2C_CR2_ITEVTEN;
    I2C1->TRISE = PCLK1_MHZ + 1;
    I2C1->CR1 |= I2C_CR1_PE;

    NVIC_EnableIRQ(I2C1_EV_IRQn);
}

static void sleep_initialize(void) {
    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk | SCB_SCR_SLEEPDEEP_Msk;
    PWR->CR &= ~PWR_CR_PDDS;
    PWR->CR |= PWR_CR_LPDS;
}

static void sleep_prevent_deep(void) {
    if (++global_sleep_semaphore == 1) {
        SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    }
}

static void sleep_allow_deep(void) {
    if (--global_sleep_semaphore == 0) {
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    }
}

static constexpr unsigned char ACCELEROMETER_INITSEQ_1[4] = {
    LIS35DE_CTRL1 | LIS35DE_AUTOINCREMENT,
    LIS35DE_CTRL1_PD
        | LIS35DE_CTRL1_XEN | LIS35DE_CTRL1_YEN | LIS35DE_CTRL1_ZEN,
    0,
    LIS35DE_CTRL3_I1CFG_CLICK,
};

static constexpr unsigned char ACCELEROMETER_INITSEQ_2[2] = {
    LIS35DE_CLICKCFG,
    LIS35DE_CLICKCFG_LIR
        | LIS35DE_CLICKCFG_DOUBLEZ | LIS35DE_CLICKCFG_DOUBLEY | LIS35DE_CLICKCFG_DOUBLEX
        | LIS35DE_CLICKCFG_SINGLEZ | LIS35DE_CLICKCFG_SINGLEY | LIS35DE_CLICKCFG_SINGLEX,
};

// TODO: Investigate what these parameters actually do.
static constexpr unsigned char ACCELEROMETER_INITSEQ_3[6] = {
    LIS35DE_CLICKTHSYX | LIS35DE_AUTOINCREMENT,
    0,
    0,
    UINT8_MAX,
    UINT8_MAX,
    UINT8_MAX,
};

static void accelerometer_initialize_1(void) {
    sleep_prevent_deep();
    i2c_write_read(LIS35DE_I2C_ADDR, ACCELEROMETER_INITSEQ_1, 4, nullptr, 0, accelerometer_initialize_2);
}

static void accelerometer_initialize_2(void) {
    i2c_write_read(LIS35DE_I2C_ADDR, ACCELEROMETER_INITSEQ_2, 2, nullptr, 0, accelerometer_initialize_3);
}

static void accelerometer_initialize_3(void) {
    i2c_write_read(LIS35DE_I2C_ADDR, ACCELEROMETER_INITSEQ_3, 6, nullptr, 0, accelerometer_initialize_4);
}

static void accelerometer_initialize_4(void) {
    GPIOinConfigure(LIS35DE_INT1_GPIO, LIS35DE_INT1_PIN, GPIO_PuPd_NOPULL, EXTI_Mode_Interrupt, EXTI_Trigger_Rising);
    NVIC_EnableIRQ(EXTI1_IRQn);
    RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGEN;
    sleep_allow_deep();
}

static int accelerometer_contains_single_click(unsigned clicksrc) {
    return (clicksrc & (LIS35DE_CLICKSRC_SINGLEX | LIS35DE_CLICKSRC_SINGLEY | LIS35DE_CLICKSRC_SINGLEZ)) != 0;
}

static int accelerometer_contains_double_click(unsigned clicksrc) {
    return (clicksrc & (LIS35DE_CLICKSRC_DOUBLEX | LIS35DE_CLICKSRC_DOUBLEY | LIS35DE_CLICKSRC_DOUBLEZ)) != 0;
}


int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN | RCC_APB1ENR_PWREN | RCC_APB1ENR_TIM3EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    __NOP();

    led_initialize();
    timer_initialize();
    i2c_initialize();
    sleep_initialize();
    accelerometer_initialize_1();

    for (;;)
        __WFI();
}

// TODO: Document interrupt priorities being the same.

// TODO: Document latching and clearing interrupt flags in both handlers.

void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_CC1IF) {
        timer_1_stop();
        led_red_off();
        sleep_allow_deep();
        TIM3->SR = ~TIM_SR_CC1IF;
    }

    if (TIM3->SR & TIM_SR_CC2IF) {
        timer_2_stop();
        led_green_off();
        sleep_allow_deep();
        TIM3->SR = ~TIM_SR_CC2IF;
    }
}

void EXTI1_IRQHandler(void) {
    EXTI->PR = EXTI_PR_PR1;

    i2c_write_read(LIS35DE_I2C_ADDR, &LIS35DE_CLICKSRC, 1, &global_clicksrc, 1, on_read_clicksrc);

    sleep_prevent_deep();
}

static void on_read_clicksrc(void) {
    sleep_allow_deep();

    // TODO: Figure out how to handle multiple clicks in a single interrupt.

    if (accelerometer_contains_single_click(global_clicksrc)) {
        if (!timer_1_is_active()) {
            timer_1_start();
            led_red_on();
            sleep_prevent_deep();
        } else {
            timer_1_extend();
        }
    }

    if (accelerometer_contains_double_click(global_clicksrc)) {
        if (!timer_2_is_active()) {
            timer_2_start();
            led_green_on();
            sleep_prevent_deep();
        } else {
            timer_2_extend();
        }
    }
}
