#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include <string.h>

#define LEFT_BUTTON_GPIO GPIOB
#define RIGHT_BUTTON_GPIO GPIOB
#define UP_BUTTON_GPIO GPIOB
#define DOWN_BUTTON_GPIO GPIOB
#define FIRE_BUTTON_GPIO GPIOB
#define USER_BUTTON_GPIO GPIOC
#define MODE_BUTTON_GPIO GPIOA

#define LEFT_BUTTON_PIN 3
#define RIGHT_BUTTON_PIN 4
#define UP_BUTTON_PIN 5
#define DOWN_BUTTON_PIN 6
#define FIRE_BUTTON_PIN 10
#define USER_BUTTON_PIN 13
#define MODE_BUTTON_PIN 0

#define LeftButtonStatus() ((LEFT_BUTTON_GPIO->IDR & (1 << LEFT_BUTTON_PIN)) == 0)
#define RightButtonStatus() ((RIGHT_BUTTON_GPIO->IDR & (1 << RIGHT_BUTTON_PIN)) == 0)
#define UpButtonStatus() ((UP_BUTTON_GPIO->IDR & (1 << UP_BUTTON_PIN)) == 0)
#define DownButtonStatus() ((DOWN_BUTTON_GPIO->IDR & (1 << DOWN_BUTTON_PIN)) == 0)
#define FireButtonStatus() ((FIRE_BUTTON_GPIO->IDR & (1 << FIRE_BUTTON_PIN)) == 0)
#define UserButtonStatus() ((USER_BUTTON_GPIO->IDR & (1 << USER_BUTTON_PIN)) == 0)
#define ModeButtonStatus() ((MODE_BUTTON_GPIO->IDR & (1 << MODE_BUTTON_PIN)) != 0)

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

const char* const LOG_BUFFER_MESSAGES[14] = {
    "LEFT PRESSED  \r\n",
    "LEFT RELEASED \r\n",
    "RIGHT PRESSED \r\n",
    "RIGHT RELEASED\r\n",
    "UP PRESSED    \r\n",
    "UP RELEASED   \r\n",
    "DOWN PRESSED  \r\n",
    "DOWN RELEASED \r\n",
    "FIRE PRESSED  \r\n",
    "FIRE RELEASED \r\n",
    "USER PRESSED  \r\n",
    "USER RELEASED \r\n",
    "MODE PRESSED  \r\n",
    "MODE RELEASED \r\n",
};

void log_buffer_push(char message) {
    if (log_buffer.window_length == LOG_BUFFER_CAPACITY) {
        // Out of memory, drop the message as there's nothing better to do?
        return;
    }
    log_buffer.data[(log_buffer.window_start + log_buffer.window_length) % LOG_BUFFER_CAPACITY] = message;
    log_buffer.window_length = (log_buffer.window_length + 1) % LOG_BUFFER_CAPACITY;
}

char log_buffer_pop() {
    char message = log_buffer.data[log_buffer.window_start];
    log_buffer.window_start = (log_buffer.window_start + 1) % LOG_BUFFER_CAPACITY;
    log_buffer.window_length -= 1;
    return message;
}

int log_buffer_is_empty() {
    return log_buffer.window_length == 0;
}

void log_buffer_force_send_message(char message) {
    DMA1_Stream6->M0AR = (uint32_t) LOG_BUFFER_MESSAGES[(int) message];
    DMA1_Stream6->NDTR = 16;
    DMA1_Stream6->CR |= DMA_SxCR_EN;
}

void log_buffer_try_send_message(char message) {
    if ((DMA1_Stream6->CR & DMA_SxCR_EN) == 0 && (DMA1->HISR & DMA_HISR_TCIF6) == 0) {
        log_buffer_force_send_message(message);
    } else {
        log_buffer_push(message);
    }
}

void log_buffer_process_button(int index, int new_state) {
    if (!log_buffer.state[index] && new_state) {
        log_buffer.state[index] = new_state;
        log_buffer_try_send_message(2 * index);
    } else if (log_buffer.state[index] && !new_state) {
        log_buffer.state[index] = new_state;
        log_buffer_try_send_message(2 * index + 1);
    }
}

int main() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    __NOP();

    DMA1_Stream6->CR = 4u << 25 | DMA_SxCR_PL_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;
    DMA1_Stream6->PAR = (uint32_t) &USART2->DR;
    DMA1->HIFCR = DMA_HIFCR_CTCIF6;
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    GPIOafConfigure(GPIOA,
        2,
        GPIO_OType_PP,
        GPIO_Fast_Speed,
        GPIO_PuPd_NOPULL,
        GPIO_AF_USART2);
    USART2->CR1 = USART_CR1_TE;
    USART2->CR2 = 0;
    USART2->CR3 = USART_CR3_DMAT;
    USART2->BRR = (PCLK1_HZ + (BAUD / 2U)) / BAUD;
    USART2->CR1 |= USART_CR1_UE;

    GPIOinConfigure(LEFT_BUTTON_GPIO, LEFT_BUTTON_PIN, GPIO_PuPd_UP, EXTI_Mode_Interrupt, EXTI_Trigger_Rising_Falling);
    GPIOinConfigure(UP_BUTTON_GPIO, UP_BUTTON_PIN, GPIO_PuPd_UP, EXTI_Mode_Interrupt, EXTI_Trigger_Rising_Falling);
    GPIOinConfigure(RIGHT_BUTTON_GPIO, RIGHT_BUTTON_PIN, GPIO_PuPd_UP, EXTI_Mode_Interrupt, EXTI_Trigger_Rising_Falling);
    GPIOinConfigure(DOWN_BUTTON_GPIO, DOWN_BUTTON_PIN, GPIO_PuPd_UP, EXTI_Mode_Interrupt, EXTI_Trigger_Rising_Falling);
    GPIOinConfigure(FIRE_BUTTON_GPIO, FIRE_BUTTON_PIN, GPIO_PuPd_UP, EXTI_Mode_Interrupt, EXTI_Trigger_Rising_Falling);
    GPIOinConfigure(USER_BUTTON_GPIO, USER_BUTTON_PIN, GPIO_PuPd_UP, EXTI_Mode_Interrupt, EXTI_Trigger_Rising_Falling);
    GPIOinConfigure(MODE_BUTTON_GPIO, MODE_BUTTON_PIN, GPIO_PuPd_UP, EXTI_Mode_Interrupt, EXTI_Trigger_Rising_Falling);
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_EnableIRQ(EXTI0_IRQn);

    for (;;) {
        __NOP();
    }
}

void EXTI3_IRQHandler(void) {
    EXTI->PR = EXTI_PR_PR3;
    log_buffer_process_button(0, LeftButtonStatus());
}

void EXTI4_IRQHandler(void) {
    EXTI->PR = EXTI_PR_PR4;
    log_buffer_process_button(1, RightButtonStatus());
}

void EXTI9_5_IRQHandler(void) {
    EXTI->PR = EXTI_PR_PR5 | EXTI_PR_PR6;
    log_buffer_process_button(2, UpButtonStatus());
    log_buffer_process_button(3, DownButtonStatus());
}

void EXTI15_10_IRQHandler(void) {
    EXTI->PR = EXTI_PR_PR10 | EXTI_PR_PR13;
    log_buffer_process_button(4, FireButtonStatus());
    log_buffer_process_button(5, UserButtonStatus());
}

void EXTI0_IRQHandler(void) {
    EXTI->PR = EXTI_PR_PR0;
    log_buffer_process_button(6, ModeButtonStatus());
}

void DMA1_Stream6_IRQHandler() {
    if (DMA1->HISR & DMA_HISR_TCIF6) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF6;
        if (!log_buffer_is_empty()) {
            char message = log_buffer_pop();
            log_buffer_force_send_message(message);
        }
    }
}
