#pragma once

#include <cstddef>
#include <cstdint>
#include "hardware/i2c.h"

class I2CDevice {
public:
    explicit I2CDevice(i2c_inst_t* i2c, uint8_t address);

protected:
    static uint16_t u16(const uint8_t* b);
    static int16_t s16(const uint8_t* b);
    static int8_t s8(uint8_t b);
    bool readRegisters(uint8_t reg, uint8_t* data, size_t length);
    bool writeRegister(uint8_t reg, uint8_t value);

private:
    i2c_inst_t* i2c_;
    uint8_t address_;

};