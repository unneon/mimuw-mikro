#include <gpio.h>
#include <stm32.h>
#include "i2c.h"

#define WAIT_WHILE(condition) while (condition) {}

static volatile struct {
    unsigned address;
    const unsigned char* write_data;
    unsigned write_length;
    volatile unsigned char* read_data;
    unsigned read_length;
    int read_started;
    void(*on_finish)(void);
} global_state;

void i2c_write_read(
    unsigned address,
    const unsigned char* write_data,
    unsigned write_length,
    volatile unsigned char* read_data,
    unsigned read_length,
    void(*on_finish)(void)
) {
    global_state.address = address;
    global_state.write_data = write_data;
    global_state.write_length = write_length;
    global_state.read_data = read_data;
    global_state.read_length = read_length;
    global_state.read_started = 0;
    global_state.on_finish = on_finish;
    I2C1->CR2 |= I2C_CR2_ITBUFEN;
    I2C1->CR1 |= I2C_CR1_START;
}

void I2C1_EV_IRQHandler(void) {
    if (I2C1->SR1 & I2C_SR1_SB) {
        if (global_state.write_length > 0) {
            I2C1->DR = global_state.address << 1;
        } else {
            I2C1->DR = (global_state.address << 1) | 1;
            if (global_state.read_length <= 1) {
                I2C1->CR1 &= ~I2C_CR1_ACK;
            } else {
                I2C1->CR1 |= I2C_CR1_ACK;
            }
        }
    }

    if (I2C1->SR1 & I2C_SR1_ADDR) {
        I2C1->SR2;
        if (global_state.read_started && global_state.read_length == 1) {
            I2C1->CR1 |= I2C_CR1_STOP;
        }
    }

    if (global_state.write_length > 0 && I2C1->SR1 & I2C_SR1_TXE) {
        I2C1->DR = *global_state.write_data++;
        if (--global_state.write_length == 0 && global_state.read_length == 0) {
            I2C1->CR2 &= ~I2C_CR2_ITBUFEN;
        }
    } else if (global_state.write_length == 0 && !global_state.read_started && I2C1->SR1 & I2C_SR1_BTF) {
        if (global_state.read_length > 0) {
            I2C1->CR1 |= I2C_CR1_START;
            global_state.read_started = 1;
        } else {
            I2C1->CR1 |= I2C_CR1_STOP;
            WAIT_WHILE(I2C1->SR2 & I2C_SR2_BUSY);
            global_state.on_finish();
        }
    } else if (I2C1->SR1 & I2C_SR1_RXNE) {
        *global_state.read_data++ = I2C1->DR;
        if (--global_state.read_length == 1) {
            I2C1->CR1 &= ~I2C_CR1_ACK;
            I2C1->CR1 |= I2C_CR1_STOP;
        } else if (global_state.read_length == 0) {
            I2C1->CR2 &= ~I2C_CR2_ITBUFEN;
            WAIT_WHILE(I2C1->SR2 & I2C_SR2_BUSY);
            global_state.on_finish();
        }
    }
}
