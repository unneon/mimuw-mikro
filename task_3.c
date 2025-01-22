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

int main() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    __NOP();

    GPIOafConfigure(RED_LED_GPIO, RED_LED_PIN, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_TIM3);
    GPIOafConfigure(GREEN_LED_GPIO, GREEN_LED_PIN, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_TIM3);
    GPIOafConfigure(BLUE_LED_GPIO, BLUE_LED_PIN, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_TIM3);

    TIM3->PSC = 10000;
    TIM3->ARR = 20;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CCR1 = 1;
    TIM3->CCR2 = 1;
    TIM3->CCR3 = 1;
    TIM3->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE
        | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
    TIM3->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P
        | TIM_CCER_CC2E | TIM_CCER_CC2P
        | TIM_CCER_CC3E | TIM_CCER_CC3P;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    for (;;) {
        __NOP();
    }
}

