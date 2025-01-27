#include <gpio.h>
#include <stddef.h>
#include <stm32.h>
#include "config.h"
#include "i2c.h"
#include "lis35de.h"

// Some of the logic in the program has to board-specific given the material
// taught in this course, so the hardware description will also be located
// here. Other source files do not contain hidden non-portable behavior.

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

// PSC is set as high as possible to reduce power usage (?), while cleanly
// dividing 16 MHz into 250 Hz. The 3 second period specified in the task
// description then occurs exactly every 750 ticks.
#define TIMER_PSC 64'000
#define TIMER_3S 750

static_assert(CONFIG_CLICK_THRESHOLD_MG % 500 == 0 && CONFIG_CLICK_THRESHOLD_MG >= 500 && CONFIG_CLICK_THRESHOLD_MG <= 7'500, "Click threshold must range from 0.5g to 7.5g with a step of 0.5g.");
static_assert(CONFIG_CLICK_TIMELIMIT_US % 500 == 0 && CONFIG_CLICK_TIMELIMIT_US >= 0 && CONFIG_CLICK_TIMELIMIT_US <= 127'500, "Click time limit must range from 0ms to 127.5ms with a step of 0.5ms.");
static_assert(CONFIG_CLICK_LATENCY_US % 1'000 == 0 && CONFIG_CLICK_LATENCY_US >= 0 && CONFIG_CLICK_LATENCY_US <= 255'000, "Click latency must range from 0ms to 255ms with a step of 1ms.");
static_assert(CONFIG_CLICK_WINDOW_US % 1'000 == 0 && CONFIG_CLICK_WINDOW_US >= 0 && CONFIG_CLICK_WINDOW_US <= 255'000, "Click window must range from 0ms to 255ms with a step of 1ms.");

// All accesses to the global variables happen in interrupt handlers, with one
// exception. The `global_sleep_semaphore` variable is accessed in
// `accelerometer_initialize_1`, but no other code will access it until the
// accelerometer is fully initialized and sends an interrupt. Thanks to the
// fact that all interrupts are configured to the same default priority, using
// these variables is safe without disabling interrupts.
static unsigned char global_clicksrc;
static unsigned global_sleep_semaphore = 0;

// These functions are forward-declared so that their reading order corresponds
// to execution order, making the accelerometer initialization sequence and
// click interrupt handling easier to understand.
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

// This program uses the TIM3 timer to keep track of the 3 second intervals
// specified in the task description. The overflow event is ignored, and CCR1
// and CCR2 are used for single and double clicks respectively. The associated
// interrupts are only active when the LED is active, and are disabled in the
// interrupt handler when the timer expires. When we need to extend the timer,
// we first increase the CCR value, and then clear the SR register to make sure
// the interrupt did not fire as the timer was being extended.

static void timer_initialize(void) {
    TIM3->PSC = TIMER_PSC;
    TIM3->ARR = UINT16_MAX;
    TIM3->SR = ~(TIM_SR_CC1IF | TIM_SR_CC2IF);
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM3_IRQn);
}

static void timer_1_start(void) {
    TIM3->CCR1 = (uint16_t) (TIM3->CNT + TIMER_3S);
    TIM3->DIER |= TIM_DIER_CC1IE;
}

static void timer_1_stop(void) {
    TIM3->DIER &= ~TIM_DIER_CC1IE;
}

static void timer_1_extend(void) {
    TIM3->CCR1 = (uint16_t) (TIM3->CCR1 + TIMER_3S);
    TIM3->SR = ~TIM_SR_CC1IF;
}

static int timer_1_is_active(void) {
    return (TIM3->DIER & TIM_DIER_CC1IE) != 0;
}

static void timer_2_start(void) {
    TIM3->CCR2 = (uint16_t) (TIM3->CNT + TIMER_3S);
    TIM3->DIER |= TIM_DIER_CC2IE;
}

static void timer_2_stop(void) {
    TIM3->DIER &= ~TIM_DIER_CC2IE;
}

static void timer_2_extend(void) {
    TIM3->CCR2 = (uint16_t) (TIM3->CCR2 + TIMER_3S);
    TIM3->SR = ~TIM_SR_CC2IF;
}

static int timer_2_is_active(void) {
    return (TIM3->DIER & TIM_DIER_CC2IE) != 0;
}

// Logic driving I2C communication is located in the `i2c.c` file, as it is
// complex and not board-specific. The initialization is board-specific, so it
// is located here instead.

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

// After initialization, the application runs entirely in interrupt handlers.
// When waiting for an event from an accelerometer, we can enter stop mode
// (here referred to as deep sleep) as the accelerometer is connected over the
// EXTI interface which can wake the processor from stop mode. However, when an
// LED is active or an I2C transfer is active, we have to wait for a TIM3 or
// I2C1 interrupt which would be disabled in stop mode. To handle concurrency
// correctly, we use a semaphore keeping track of the number of reasons we
// cannot enter deep sleep.

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

// According to LIS35DE documentation, it's possible to write of read multiple
// registers in a row using the LIS35DE_AUTOINCREMENT bit. Thanks to this, the
// initialization sequence of the accelerometer can be reduced to 3 I2C
// sequences.

static constexpr unsigned char ACCELEROMETER_INITSEQ_1[] = {
    LIS35DE_CTRL1 | LIS35DE_AUTOINCREMENT,
    LIS35DE_CTRL1_PD | LIS35DE_CTRL1_ZEN,
    0,
    LIS35DE_CTRL3_I1CFG_CLICK,
};

static constexpr unsigned char ACCELEROMETER_INITSEQ_2[] = {
    LIS35DE_CLICKCFG,
    LIS35DE_CLICKCFG_LIR | LIS35DE_CLICKCFG_SINGLEZ | LIS35DE_CLICKCFG_DOUBLEZ,
};

static constexpr unsigned char ACCELEROMETER_INITSEQ_3[] = {
    LIS35DE_CLICKTHSYX | LIS35DE_AUTOINCREMENT,
    ((CONFIG_CLICK_THRESHOLD_MG / 500) << 4) | (CONFIG_CLICK_THRESHOLD_MG / 500),
    CONFIG_CLICK_THRESHOLD_MG / 500,
    CONFIG_CLICK_TIMELIMIT_US / 500,
    CONFIG_CLICK_LATENCY_US / 1'000,
    CONFIG_CLICK_WINDOW_US / 1'000,
};

static void accelerometer_initialize_1(void) {
    sleep_prevent_deep();

    i2c_write_read(LIS35DE_I2C_ADDR, ACCELEROMETER_INITSEQ_1, sizeof(ACCELEROMETER_INITSEQ_1), nullptr, 0, accelerometer_initialize_2);
}

static void accelerometer_initialize_2(void) {
    i2c_write_read(LIS35DE_I2C_ADDR, ACCELEROMETER_INITSEQ_2, sizeof(ACCELEROMETER_INITSEQ_2), nullptr, 0, accelerometer_initialize_3);
}

static void accelerometer_initialize_3(void) {
    i2c_write_read(LIS35DE_I2C_ADDR, ACCELEROMETER_INITSEQ_3, sizeof(ACCELEROMETER_INITSEQ_3), nullptr, 0, accelerometer_initialize_4);
}

static void accelerometer_initialize_4(void) {
    GPIOinConfigure(LIS35DE_INT1_GPIO, LIS35DE_INT1_PIN, GPIO_PuPd_NOPULL, EXTI_Mode_Interrupt, EXTI_Trigger_Rising);

    NVIC_EnableIRQ(EXTI1_IRQn);

    RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGEN;

    sleep_allow_deep();
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

    __WFI();
    unreachable();
}

// All interrupts use the same priority, which serves as lightweight yet robust
// mutex. The `on_read_clicksrc` function will also only be called from an
// interrupt context (`I2C1_EV_IRQHandler` in `i2c.c` after being passed as a
// callback).

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
    // The LIR (latch interrupt request) bit set in the accelerometer
    // initialize sequence ensures the accelerometers will not send a next
    // interrupt until the CLICKSRC register is read. This ensures this
    // interrupt handler will not start a new I2C sequence before the previous
    // one finishes.
    EXTI->PR = EXTI_PR_PR1;

    i2c_write_read(LIS35DE_I2C_ADDR, &LIS35DE_CLICKSRC, 1, &global_clicksrc, 1, on_read_clicksrc);

    sleep_prevent_deep();
}

static void on_read_clicksrc(void) {
    sleep_allow_deep();

    if (global_clicksrc & LIS35DE_CLICKSRC_SINGLEZ) {
        if (!timer_1_is_active()) {
            led_red_on();
            timer_1_start();
            sleep_prevent_deep();
        } else {
            timer_1_extend();
        }
    }

    if (global_clicksrc & LIS35DE_CLICKSRC_DOUBLEZ) {
        if (!timer_2_is_active()) {
            led_green_on();
            timer_2_start();
            sleep_prevent_deep();
        } else {
            timer_2_extend();
        }
    }
}
