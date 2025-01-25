#include <delay.h>
#include <gpio.h>
#include <stm32.h>

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

// #define HSI_HZ 16000000U
// #define PCLK1_HZ HSI_HZ
// #define BAUD 9600U
#define PCLK1_MHZ 16

#define LIS35DE_I2C_ADDR 0x1C
#define LIS35DE_CTRL1REG 0x20
#define LIS35DE_CTRL1REG_PD (1 << 6)
#define LIS35DE_CTRL1REG_ZEN (1 << 2)
#define LIS35DE_CTRL1REG_YEN (1 << 1)
#define LIS35DE_CTRL1REG_XEN (1 << 0)
#define LIS35DE_OUTX 0x29
#define LIS35DE_OUTY 0x2B
#define LIS35DE_OUTZ 0x2D

#define WAIT_FOR(condition) while (!(condition)) {}

// #define USART_BUFFER_CAPACITY 8192

// static char usart_buffer_data[USART_BUFFER_CAPACITY];

// static void usart_force_send(int length) {
//     DMA1_Stream6->M0AR = (uint32_t) usart_buffer_data;
//     DMA1_Stream6->NDTR = length;
//     DMA1_Stream6->CR |= DMA_SxCR_EN;
// }

static int abs(int x) {
    return x < 0 ? -x : x;
}

static int int_to_signed_char(int x) {
    return *(signed char*)(&x);
}

static void accelerometer_write_register(int register_number, int value) {
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

static int accelerometer_read_register(int register_number) {
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
    return int_to_signed_char(accelerometer_read_register(LIS35DE_OUTX));
}

static int accelerometer_read_y(void) {
    return int_to_signed_char(accelerometer_read_register(LIS35DE_OUTY));
}

static int accelerometer_read_z(void) {
    return int_to_signed_char(accelerometer_read_register(LIS35DE_OUTZ));
}

// static int debug_string(const char* text, int bytes_written) {
//     for (;*text;++text) {
//         usart_buffer_data[bytes_written++] = *text;
//     }
//     return bytes_written;
// }

// static int debug_int(int value, int bytes_written) {
//     if (value < 0) {
//         usart_buffer_data[bytes_written++] = '-';
//         value = -value;
//     } else {
//         usart_buffer_data[bytes_written++] = ' ';
//     }
//     int digit = 1;
//     while (10 * digit <= value)
//         digit *= 10;
//     if (digit < 100)
//         usart_buffer_data[bytes_written++] = ' ';
//     if (digit < 10)
//         usart_buffer_data[bytes_written++] = ' ';
//     while (digit > 0) {
//         usart_buffer_data[bytes_written++] = '0' + value / digit;
//         value %= digit;
//         digit /= 10;
//     }
//     return bytes_written;
// }

// static void accelerometer_debug(void) {
//     int x_axis = accelerometer_read_x();
//     int y_axis = accelerometer_read_y();
//     int z_axis = accelerometer_read_z();

//     int bytes_written = 0;
//     bytes_written = debug_string("x: ", bytes_written);
//     bytes_written = debug_int(x_axis, bytes_written);
//     bytes_written = debug_string(", y: ", bytes_written);
//     bytes_written = debug_int(y_axis, bytes_written);
//     bytes_written = debug_string(", z: ", bytes_written);
//     bytes_written = debug_int(z_axis, bytes_written);
//     bytes_written = debug_string("\r\n", bytes_written);
//     usart_force_send(bytes_written);
// }

int main() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    // RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN | RCC_APB1ENR_TIM3EN;
    // RCC->APB1ENR |= RCC_APB1ENR_I2C1EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    __NOP();

    // DMA1_Stream6->CR = 4u << 25 | DMA_SxCR_PL_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;
    // DMA1_Stream6->PAR = (uint32_t) &USART2->DR;
    // DMA1->HIFCR = DMA_HIFCR_CTCIF6;
    // NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    // GPIOafConfigure(GPIOA, 2, GPIO_OType_PP, GPIO_Fast_Speed, GPIO_PuPd_NOPULL, GPIO_AF_USART2);
    GPIOafConfigure(RED_LED_GPIO, RED_LED_PIN, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_TIM3);
    GPIOafConfigure(GREEN_LED_GPIO, GREEN_LED_PIN, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_TIM3);
    GPIOafConfigure(BLUE_LED_GPIO, BLUE_LED_PIN, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_TIM3);
    GPIOafConfigure(I2C_SCL_GPIO, I2C_SCL_PIN, GPIO_OType_OD, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_I2C1);
    GPIOafConfigure(I2C_SDA_GPIO, I2C_SDA_PIN, GPIO_OType_OD, GPIO_Low_Speed, GPIO_PuPd_NOPULL, GPIO_AF_I2C1);

    // USART2->CR1 = USART_CR1_TE;
    // USART2->CR2 = 0;
    // USART2->CR3 = USART_CR3_DMAT;
    // USART2->BRR = (PCLK1_HZ + (BAUD / 2U)) / BAUD;
    // USART2->CR1 |= USART_CR1_UE;

    TIM3->PSC = 1000;
    TIM3->ARR = 128;
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

    accelerometer_write_register(LIS35DE_CTRL1REG, LIS35DE_CTRL1REG_PD
        | LIS35DE_CTRL1REG_XEN | LIS35DE_CTRL1REG_YEN | LIS35DE_CTRL1REG_ZEN);

    // WAIT_FOR((DMA1_Stream6->CR & DMA_SxCR_EN) == 0 && (DMA1->HISR & DMA_HISR_TCIF6) == 0);
    // accelerometer_debug();

    for (;;) {
        TIM3->CCR1 = abs(accelerometer_read_x());
        TIM3->CCR2 = abs(accelerometer_read_y());
        TIM3->CCR3 = abs(accelerometer_read_z());
        __NOP();
    }
}

// void DMA1_Stream6_IRQHandler() {
//     if (DMA1->HISR & DMA_HISR_TCIF6) {
//         DMA1->HIFCR = DMA_HIFCR_CTCIF6;
//         accelerometer_debug();
//     }
// }
