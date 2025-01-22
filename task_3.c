#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include <string.h>

#define RED_LED_GPIO GPIOA
#define GREEN_LED_GPIO GPIOA
#define BLUE_LED_GPIO GPIOB
#define MODE_BUTTON_GPIO GPIOA

#define RED_LED_PIN 6
#define GREEN_LED_PIN 7
#define BLUE_LED_PIN 0
#define MODE_BUTTON_PIN 0

#define RedLEDon() RED_LED_GPIO->BSRR = 1 << (RED_LED_PIN + 16)
#define RedLEDoff() RED_LED_GPIO->BSRR = 1 << RED_LED_PIN
#define RedLEDstatus() ((RED_LED_GPIO->ODR & (1 << RED_LED_PIN)) == 0)

#define GreenLEDon() GREEN_LED_GPIO->BSRR = 1 << (GREEN_LED_PIN + 16)
#define GreenLEDoff() GREEN_LED_GPIO->BSRR = 1 << GREEN_LED_PIN
#define GreenLEDstatus() ((GREEN_LED_GPIO->ODR & (1 << GREEN_LED_PIN)) == 0)

#define BlueLEDon() BLUE_LED_GPIO->BSRR = 1 << (BLUE_LED_PIN + 16)
#define BlueLEDoff() BLUE_LED_GPIO->BSRR = 1 << BLUE_LED_PIN
#define BlueLEDstatus() ((BLUE_LED_GPIO->ODR & (1 << BLUE_LED_PIN)) == 0)

#define ModeButtonStatus() ((MODE_BUTTON_GPIO->IDR & (1 << MODE_BUTTON_PIN)) != 0)

int main() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    __NOP();

    RedLEDoff();
    GreenLEDoff();
    BlueLEDoff();

    GPIOoutConfigure(RED_LED_GPIO, RED_LED_PIN, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL);
    GPIOoutConfigure(GREEN_LED_GPIO, GREEN_LED_PIN, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL);
    GPIOoutConfigure(BLUE_LED_GPIO, BLUE_LED_PIN, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL);

    GPIOafConfigure(GPIOA, 6, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_TIM3);
    GPIOafConfigure(GPIOA, 7, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_TIM3);

    // TIM3->PSC = 63999;
    TIM3->PSC = 1;
    TIM3->ARR = 749;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CCR1 = 2;
    TIM3->CCR2 = 748;
    TIM3->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1
        | TIM_CCMR1_OC1PE
        | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0
        | TIM_CCMR1_OC2PE;
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC2E | TIM_CCER_CC2P;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    // GPIOinConfigure(MODE_BUTTON_GPIO, MODE_BUTTON_PIN, GPIO_PuPd_UP, EXTI_Mode_Interrupt, EXTI_Trigger_Rising_Falling);
    // NVIC_EnableIRQ(EXTI0_IRQn);

    for (;;) {
        __NOP();
    }
}

// void EXTI0_IRQHandler(void) {
//     EXTI->PR = EXTI_PR_PR0;
//     if (ModeButtonStatus()) {
//         RedLEDon();
//     } else {
//         RedLEDoff();
//     }
// }
