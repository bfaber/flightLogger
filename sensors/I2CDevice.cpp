
#include "I2CDevice.h"

uint16_t 
I2CDevice::u16(const uint8_t* b) {
    return uint16_t(b[0]) | (uint16_t(b[1]) << 8);
}

int16_t 
I2CDevice::s16(const uint8_t* b) {
    uint16_t value = u16(b);
    int32_t signed_value = value;
    if (value & 0x8000)
        signed_value -= 65536;
    return static_cast<int16_t>(signed_value);
}

int8_t 
I2CDevice::s8(uint8_t b) {
    return static_cast<int8_t>(
        b < 128 ? int(b) : int(b) - 256);
}

I2CDevice::I2CDevice(i2c_inst_t* i2c, uint8_t address)
    : i2c_(i2c), address_(address) {}

bool
I2CDevice::readRegisters(uint8_t reg, uint8_t* data, size_t length) 
{
    if (i2c_write_blocking(i2c_, address_, &reg, 1, true) != 1)
        return false;

    return i2c_read_blocking(
        i2c_, address_, data, length, false
    ) == static_cast<int>(length);
}

bool 
I2CDevice::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};

    return i2c_write_blocking(
        i2c_, address_, data, 2, false
    ) == 2;
}

