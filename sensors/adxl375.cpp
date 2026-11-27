
#include "adxl375.h"

namespace {
    constexpr uint8_t REG_CHIP_ID     = 0x00;
    constexpr uint8_t REG_BW_RATE     = 0x2C;
    constexpr uint8_t REG_POWER_CTL   = 0x2D;
    constexpr uint8_t REG_INT_SOURCE  = 0x30;
    constexpr uint8_t REG_DATA_FORMAT = 0x31;
    constexpr uint8_t REG_DATA        = 0x32;
    constexpr uint8_t REG_FIFO_CTL    = 0x38;

    constexpr uint8_t CHIP_ID         = 0xE5;
    constexpr uint8_t DATA_READY      = 0x80;
}

ADXL375::ADXL375(i2c_inst_t* i2c, uint8_t address)
    : I2CDevice(i2c, address) {}

bool 
ADXL375::initialize() {
    uint8_t id = 0;

    if (!readRegisters(REG_CHIP_ID, &id, 1))
        return false;

    if (id != CHIP_ID)
        return false;

    // Standby mode
    if (!writeRegister(REG_POWER_CTL, 0x00))
        return false;

    // Right-justified, sign-extended output
    if (!writeRegister(REG_DATA_FORMAT, 0x0B))
        return false;

    // 100 Hz output data rate
    if (!writeRegister(REG_BW_RATE, 0x0A))
        return false;

    // FIFO bypass mode
    if (!writeRegister(REG_FIFO_CTL, 0x00))
        return false;

    // Enable measurement mode
    return writeRegister(REG_POWER_CTL, 0x08);
}

bool
ADXL375::read(RawSample& sample) {
    uint8_t status = 0;

    if (!readRegisters(REG_INT_SOURCE, &status, 1))
        return false;

    if (!(status & DATA_READY))
        return false;

    uint8_t b[6];

    if (!readRegisters(REG_DATA, b, sizeof(b)))
        return false;

    sample.x = s16(b);
    sample.y = s16(b + 2);
    sample.z = s16(b + 4);

    return true;
}

bool ADXL375::read(Sample& sample) {
    RawSample raw{};

    if (!read(raw))
        return false;

    sample.x_g = raw.x * SCALE_FACTOR;
    sample.y_g = raw.y * SCALE_FACTOR;
    sample.z_g = raw.z * SCALE_FACTOR;

    return true;
}

