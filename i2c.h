#pragma once

void i2c_write_read(
    unsigned address,
    const unsigned char* write_data,
    unsigned write_length,
    unsigned char* read_data,
    unsigned read_length,
    void(*on_finish)(void)
);
