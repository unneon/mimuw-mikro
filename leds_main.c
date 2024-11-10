#include <delay.h>
#include <gpio.h>
#include <stm32.h>

#define RED_LED_GPIO GPIOA
#define GREEN_LED_GPIO GPIOA
#define BLUE_LED_GPIO GPIOB
#define GREEN2_LED_GPIO GPIOA
#define USER_BUTTON_GPIO GPIOC

#define RED_LED_PIN 6
#define GREEN_LED_PIN 7
#define BLUE_LED_PIN 0
#define GREEN2_LED_PIN 5
#define USER_BUTTON_PIN 13

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

#define UserButtonStatus() ((USER_BUTTON_GPIO->IDR & (1 << USER_BUTTON_PIN)) == 0)

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

typedef struct {
    char data[4];
    int length;
} CommandBuffer;

CommandBuffer command_buffer = { .length = 0 };

int get_led(int index) {
    if (index == 0) {
        return RedLEDstatus();
    } else if (index == 1) {
        return GreenLEDstatus();
    } else if (index == 2) {
        return BlueLEDstatus();
    } else {
        return Green2LEDstatus();
    }
}

void set_led(int index, int status) {
    if (index == 0) {
        if (status) {
            RedLEDon();
        } else {
            RedLEDoff();
        }
    } else if (index == 1) {
        if (status) {
            GreenLEDon();
        } else {
            GreenLEDoff();
        }
    } else if (index == 2) {
        if (status) {
            BlueLEDon();
        } else {
            BlueLEDoff();
        }
    } else {
        if (status) {
            Green2LEDon();
        } else {
            Green2LEDoff();
        }
    }
}

void command_buffer_process(char new_char) {
    if (
        (command_buffer.length == 2 && (new_char == '0' || new_char == '1' || new_char == 'T')) ||
        (command_buffer.length == 1 && (new_char == 'R' || new_char == 'G' || new_char == 'B' || new_char == 'g')) ||
        (command_buffer.length == 0 && new_char == 'L')
    ) {
        command_buffer.data[command_buffer.length] = new_char;
        command_buffer.length += 1;
    } else if (new_char == 'L') {
        command_buffer.data[0] = 'L';
        command_buffer.length = 1;
    } else {
        command_buffer.length = 0;
    }

    if (command_buffer.length == 3) {
        int led_index;
        if (command_buffer.data[1] == 'R') {
            led_index = 0;
        } else if (command_buffer.data[1] == 'G') {
            led_index = 1;
        } else if (command_buffer.data[1] == 'B') {
            led_index = 2;
        } else {
            led_index = 3;
        }

        int desired_state;
        if (command_buffer.data[2] == '0') {
            desired_state = 0;
        } else if (command_buffer.data[2] == '1') {
            desired_state = 1;
        } else {
            desired_state = !get_led(led_index);
        }

        set_led(led_index, desired_state);

        command_buffer.length = 0;
    }
}

int main() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    __NOP();

    RedLEDoff();
    GreenLEDoff();
    BlueLEDoff();
    Green2LEDoff();

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
        // if (USART2->SR & USART_SR_RXNE) {
        //     char received_char = USART2->DR;
        //     command_buffer_process(received_char);
        // }

        set_led(0, UserButtonStatus());
        set_led(2, UserButtonStatus());
    }
}
