#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include <string.h>

#define RED_LED_GPIO GPIOA
#define RED_LED_PIN 6
#define GREEN_LED_GPIO GPIOA
#define GREEN_LED_PIN 7
#define BLUE_LED_GPIO GPIOB
#define BLUE_LED_PIN 0
#define I2C_SCL_GPIO GPIOB
#define I2C_SCL_PIN 8
#define I2C_SDA_GPIO GPIOB
#define I2C_SDA_PIN 9

#define I2C_SPEED_HZ 100000

#define PCLK1_MHZ 16

#define LIS35DE_ADDR 0x1C

#define WAIT_FOR(condition) while (!(condition)) {}

void accelerometer_write_register(int register_number, int value) {
    I2C1->CR1 |= I2C_CR1_START;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_SB);
    I2C1->DR = LIS35DE_ADDR << 1;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_ADDR);
    I2C1->SR2;

    I2C1->DR = register_number;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_TXE);
    I2C1->DR = value;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_BTF);
    I2C1->CR1 |= I2C_CR1_STOP;
}

int accelerometer_read_register(int register_number) {
    I2C1->CR1 |= I2C_CR1_START;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_SB);
    I2C1->DR = LIS35DE_ADDR << 1;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_ADDR);
    I2C1->SR2;
    I2C1->DR = register_number;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_BTF);

    I2C1->CR1 |= I2C_CR1_START;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_SB);
    I2C1->DR = LIS35DE_ADDR << 1 | 1;
    I2C1->CR1 &= ~I2C_CR1_ACK;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_ADDR);
    I2C1->SR2;

    I2C1->CR1 |= I2C_CR1_STOP;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_RXNE);

    return I2C1->DR;
}

int main() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN | RCC_APB1ENR_TIM3EN;

    __NOP();

    GPIOafConfigure(RED_LED_GPIO, RED_LED_PIN, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_TIM3);
    GPIOafConfigure(GREEN_LED_GPIO, GREEN_LED_PIN, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_TIM3);
    GPIOafConfigure(BLUE_LED_GPIO, BLUE_LED_PIN, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_TIM3);
    GPIOafConfigure(I2C_SCL_GPIO, I2C_SCL_PIN, GPIO_OType_OD, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_I2C1);
    GPIOafConfigure(I2C_SDA_GPIO, I2C_SDA_PIN, GPIO_OType_OD, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_I2C1);

    TIM3->PSC = 1;
    TIM3->ARR = 100;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE
        | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
    TIM3->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P
        | TIM_CCER_CC2E | TIM_CCER_CC2P
        | TIM_CCER_CC3E | TIM_CCER_CC3P;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    I2C1->CR1 = 0;
    I2C1->CCR = (PCLK1_MHZ * 1000000) / (I2C_SPEED_HZ << 1);
    I2C1->CR2 = PCLK1_MHZ;
    I2C1->TRISE = PCLK1_MHZ + 1;
    I2C1->CR1 |= I2C_CR1_PE;

    accelerometer_write_register(0x20, 1 << 6);

    for (;;) {
        TIM3->CCR1 = accelerometer_read_register(0x29);
    }
}

