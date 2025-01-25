#include <gpio.h>
#include <stm32.h>
#include "i2c.h"

struct I2cState {
    unsigned address;
    const unsigned char* write_data;
    unsigned write_length;
    unsigned char* read_data;
    unsigned read_length;
    int read_started;
    void(*on_finish)(void);
};

static struct I2cState state;

void i2c_write_read(
    unsigned address,
    const unsigned char* write_data,
    unsigned write_length,
    unsigned char* read_data,
    unsigned read_length,
    void(*on_finish)(void)
) {
    state.address = address;
    state.write_data = write_data;
    state.write_length = write_length;
    state.read_data = read_data;
    state.read_length = read_length;
    state.read_started = 0;
    state.on_finish = on_finish;
    I2C1->CR2 |= I2C_CR2_ITBUFEN;
    I2C1->CR1 |= I2C_CR1_START;
}

void I2C1_EV_IRQHandler(void) {
    if (I2C1->SR1 & I2C_SR1_SB) {
        if (state.write_length > 0) {
            I2C1->DR = state.address << 1;
        } else {
            I2C1->DR = (state.address << 1) | 1;
            if (state.read_length <= 1) {
                I2C1->CR1 &= ~I2C_CR1_ACK;
            } else {
                I2C1->CR1 |= I2C_CR1_ACK;
            }
        }
    }

    if (I2C1->SR1 & I2C_SR1_ADDR) {
        I2C1->SR2;
        if (state.read_started && state.read_length == 1) {
            I2C1->CR1 |= I2C_CR1_STOP;
        }
    }

    if (state.write_length > 0 && I2C1->SR1 & I2C_SR1_TXE) {
        I2C1->DR = *state.write_data++;
        if (--state.write_length == 0 && state.read_length == 0) {
            I2C1->CR2 &= ~I2C_CR2_ITBUFEN;
        }
    } else if (state.write_length == 0 && !state.read_started && I2C1->SR1 & I2C_SR1_BTF) {
        if (state.read_length > 0) {
            I2C1->CR1 |= I2C_CR1_START;
            state.read_started = 1;
        } else {
            I2C1->CR1 |= I2C_CR1_STOP;
            state.on_finish();
        }
    } else if (I2C1->SR1 & I2C_SR1_RXNE) {
        *state.read_data++ = I2C1->DR;
        if (--state.read_length == 1) {
            I2C1->CR1 &= ~I2C_CR1_ACK;
            I2C1->CR1 |= I2C_CR1_STOP;
        } else if (state.read_length == 0) {
            I2C1->CR2 &= ~I2C_CR2_ITBUFEN;
            state.on_finish();
        }
    }
}
