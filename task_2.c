#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include <string.h>

#define RED_LED_GPIO GPIOA
#define GREEN_LED_GPIO GPIOA
#define BLUE_LED_GPIO GPIOB
#define GREEN2_LED_GPIO GPIOA
#define LEFT_BUTTON_GPIO GPIOB
#define RIGHT_BUTTON_GPIO GPIOB
#define UP_BUTTON_GPIO GPIOB
#define DOWN_BUTTON_GPIO GPIOB
#define FIRE_BUTTON_GPIO GPIOB
#define USER_BUTTON_GPIO GPIOC
#define MODE_BUTTON_GPIO GPIOA

#define RED_LED_PIN 6
#define GREEN_LED_PIN 7
#define BLUE_LED_PIN 0
#define GREEN2_LED_PIN 5
#define LEFT_BUTTON_PIN 3
#define RIGHT_BUTTON_PIN 4
#define UP_BUTTON_PIN 5
#define DOWN_BUTTON_PIN 6
#define FIRE_BUTTON_PIN 10
#define USER_BUTTON_PIN 13
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

#define Green2LEDon() GREEN2_LED_GPIO->BSRR = 1 << GREEN2_LED_PIN
#define Green2LEDoff() GREEN2_LED_GPIO->BSRR = 1 << (GREEN2_LED_PIN + 16)
#define Green2LEDstatus() ((GREEN2_LED_GPIO->ODR & (1 << GREEN2_LED_PIN)) != 0)

#define LeftButtonStatus() ((LEFT_BUTTON_GPIO->IDR & (1 << LEFT_BUTTON_PIN)) == 0)
#define RightButtonStatus() ((RIGHT_BUTTON_GPIO->IDR & (1 << RIGHT_BUTTON_PIN)) == 0)
#define UpButtonStatus() ((UP_BUTTON_GPIO->IDR & (1 << UP_BUTTON_PIN)) == 0)
#define DownButtonStatus() ((DOWN_BUTTON_GPIO->IDR & (1 << DOWN_BUTTON_PIN)) == 0)
#define FireButtonStatus() ((FIRE_BUTTON_GPIO->IDR & (1 << FIRE_BUTTON_PIN)) == 0)
#define UserButtonStatus() ((USER_BUTTON_GPIO->IDR & (1 << USER_BUTTON_PIN)) == 0)
#define ModeButtonStatus() ((MODE_BUTTON_GPIO->IDR & (1 << MODE_BUTTON_PIN)) != 0)

#define USART_Mode_Rx_Tx (USART_CR1_RE | USART_CR1_TE)
#define USART_Enable USART_CR1_UE

#define USART_WordLength_8b 0x0000
#define USART_WordLength_9b USART_CR1_M

#define USART_Parity_No 0x0000
#define USART_Parity_Even USART_CR1_PCE
#define USART_Parity_Odd (USART_CR1_PCE | USART_CR1_PS)

#define USART_StopBits_1 0x0000
#define USART_StopBits_0_5 0x1000
#define USART_StopBits_2 0x2000
#define USART_StopBits_1_5 0x3000

#define USART_FlowControl_None 0x0000
#define USART_FlowControl_RTS USART_CR3_RTSE
#define USART_FlowControl_CTS USART_CR3_CTSE

#define HSI_HZ 16000000U
#define PCLK1_HZ HSI_HZ
#define BAUD 9600U

#define LOG_BUFFER_CAPACITY 8192

typedef struct {
    char data[LOG_BUFFER_CAPACITY];
    int window_start;
    int window_length;
    int state[7];
} LogBuffer;

LogBuffer log_buffer = {
    .window_start = 0,
    .window_length = 0,
    .state = {},
};

const char* const BUTTON_NAMES[7] = {
    "LEFT",
    "RIGHT",
    "UP",
    "DOWN",
    "FIRE",
    "USER",
    "MODE",
};

void log_buffer_append_char(char character) {
    log_buffer.data[(log_buffer.window_start + log_buffer.window_length) % LOG_BUFFER_CAPACITY] = character;
    log_buffer.window_length = (log_buffer.window_length + 1) % LOG_BUFFER_CAPACITY;
}

void log_buffer_append(const char* button, const char* action) {
    for (;*button;++button) {
        log_buffer_append_char(*button);
    }
    log_buffer_append_char(' ');
    for (;*action;++action) {
        log_buffer_append_char(*action);
    }
    log_buffer_append_char('\r');
    log_buffer_append_char('\n');
}

void log_buffer_process_button(int index, int new_state) {
    if (!log_buffer.state[index] && new_state) {
        log_buffer.state[index] = new_state;
        log_buffer_append(BUTTON_NAMES[index], "PRESSED");
    } else if (log_buffer.state[index] && !new_state) {
        log_buffer.state[index] = new_state;
        log_buffer_append(BUTTON_NAMES[index], "RELEASED");
    }
}

void log_buffer_process() {
    log_buffer_process_button(0, LeftButtonStatus());
    log_buffer_process_button(1, RightButtonStatus());
    log_buffer_process_button(2, UpButtonStatus());
    log_buffer_process_button(3, DownButtonStatus());
    log_buffer_process_button(4, FireButtonStatus());
    log_buffer_process_button(5, UserButtonStatus());
    log_buffer_process_button(6, ModeButtonStatus());
}

int log_buffer_has_unwritten() {
    return log_buffer.window_length > 0;
}

char log_buffer_pop_next_byte() {
    char character = log_buffer.data[log_buffer.window_start];
    log_buffer.window_start = (log_buffer.window_start + 1) % LOG_BUFFER_CAPACITY;
    log_buffer.window_length -= 1;
    return character;
}

int main() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    __NOP();

    RedLEDoff();
    GreenLEDoff();
    BlueLEDoff();
    Green2LEDoff();

    RedLEDon();
    BlueLEDon();

    GPIOoutConfigure(RED_LED_GPIO,
        RED_LED_PIN,
        GPIO_OType_PP,
        GPIO_Low_Speed,
        GPIO_PuPd_NOPULL);
    GPIOoutConfigure(GREEN_LED_GPIO,
        GREEN_LED_PIN,
        GPIO_OType_PP,
        GPIO_Low_Speed,
        GPIO_PuPd_NOPULL);
    GPIOoutConfigure(BLUE_LED_GPIO,
        BLUE_LED_PIN,
        GPIO_OType_PP,
        GPIO_Low_Speed,
        GPIO_PuPd_NOPULL);
    GPIOoutConfigure(GREEN2_LED_GPIO,
        GREEN2_LED_PIN,
        GPIO_OType_PP,
        GPIO_Low_Speed,
        GPIO_PuPd_NOPULL);

    GPIOafConfigure(GPIOA,
        2,
        GPIO_OType_PP,
        GPIO_Fast_Speed,
        GPIO_PuPd_NOPULL,
        GPIO_AF_USART2);
    GPIOafConfigure(GPIOA,
        3,
        GPIO_OType_PP,
        GPIO_Fast_Speed,
        GPIO_PuPd_UP,
        GPIO_AF_USART2);

    USART2->CR1 = USART_Mode_Rx_Tx | USART_WordLength_8b | USART_Parity_No;
    USART2->CR2 = USART_StopBits_1;
    USART2->CR3 = USART_FlowControl_None;
    USART2->BRR = (PCLK1_HZ + (BAUD / 2U)) / BAUD;

    USART2->CR1 |= USART_Enable;

    for (;;) {
        // log_buffer_process();
        // if (log_buffer_has_unwritten() && USART2->SR & USART_SR_TXE) {
        //     USART2->DR = log_buffer_pop_next_byte();
        // }
    }
}
