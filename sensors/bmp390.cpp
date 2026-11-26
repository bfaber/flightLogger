
#include "bmp390.h"

#include <cstdio>
#include <cmath>

namespace {
    constexpr uint8_t REG_CHIP_ID  = 0x00;
    constexpr uint8_t REG_STATUS   = 0x03;
    constexpr uint8_t REG_DATA     = 0x04;
    constexpr uint8_t REG_PWR_CTRL = 0x1B;
    constexpr uint8_t REG_CALIB    = 0x31;
    constexpr uint8_t CHIP_ID      = 0x60;
    constexpr uint8_t REG_OSR      = 0x1C;
    constexpr uint8_t REG_ODR      = 0x1D;

    uint16_t u16(const uint8_t* b) {
        return uint16_t(b[0]) | (uint16_t(b[1]) << 8);
    }

    int16_t s16(const uint8_t* b) {
        uint16_t value = u16(b);
        int32_t signed_value = value;
        if (value & 0x8000)
            signed_value -= 65536;
        return static_cast<int16_t>(signed_value);
    }

    int8_t s8(uint8_t b) {
        return static_cast<int8_t>(
            b < 128 ? int(b) : int(b) - 256);
    }
}

BMP390::BMP390(i2c_inst_t* i2c, uint8_t address)
    : i2c_(i2c), address_(address) {}

bool BMP390::readRegisters(
    uint8_t reg, uint8_t* data, size_t length) {

    if (i2c_write_blocking(i2c_, address_, &reg, 1, true) != 1)
        return false;

    return i2c_read_blocking(
        i2c_, address_, data, length, false
    ) == static_cast<int>(length);
}

bool BMP390::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};

    return i2c_write_blocking(
        i2c_, address_, data, 2, false
    ) == 2;
}

bool BMP390::readFactoryCalibration(
    uint8_t (&bytes)[21]) {

    return readRegisters(0x31, bytes, 21);
}

bool BMP390::readCalibration() {
    uint8_t b[21];

    if (!readRegisters(REG_CALIB, b, sizeof(b)))
        return false;

    // Decode and scale Bosch factory coefficients.
    calib_.t1  = std::ldexp(double(u16(b + 0)),   8);
    calib_.t2  = std::ldexp(double(u16(b + 2)), -30);
    calib_.t3  = std::ldexp(double(s8(b[4])),  -48);

    calib_.p1  = std::ldexp(double(s16(b + 5)) - 16384.0, -20);
    calib_.p2  = std::ldexp(double(s16(b + 7)) - 16384.0, -29);
    calib_.p3  = std::ldexp(double(s8(b[9])),  -32);
    calib_.p4  = std::ldexp(double(s8(b[10])), -37);
    calib_.p5  = std::ldexp(double(u16(b + 11)),  3);
    calib_.p6  = std::ldexp(double(u16(b + 13)), -6);
    calib_.p7  = std::ldexp(double(s8(b[15])), -8);
    calib_.p8  = std::ldexp(double(s8(b[16])), -15);
    calib_.p9  = std::ldexp(double(s16(b + 17)), -48);
    calib_.p10 = std::ldexp(double(s8(b[19])), -48);
    calib_.p11 = std::ldexp(double(s8(b[20])), -65);

    return true;
}

bool BMP390::initialize() {
    uint8_t id = 0;

    if (!readRegisters(REG_CHIP_ID, &id, 1))
        return false;

    if (id != CHIP_ID)
        return false;

    if (!readCalibration())
        return false;

    // 4x pressure oversampling, 1x temperature
    if (!writeRegister(REG_OSR, 0x04))
        return false;

    // 25 Hz output data rate
    if (!writeRegister(REG_ODR, 0x03))
        return false;

    // Enable pressure, temperature, normal mode
    return writeRegister(REG_PWR_CTRL, 0x33);
}

bool BMP390::read(RawSample& sample) {
    uint8_t status = 0;

    if (!readRegisters(REG_STATUS, &status, 1))
        return false;

    if ((status & 0x60) != 0x60)
        return false;

    uint8_t b[6];

    if (!readRegisters(REG_DATA, b, sizeof(b)))
        return false;

    sample.pressure =
        uint32_t(b[0]) |
        (uint32_t(b[1]) << 8) |
        (uint32_t(b[2]) << 16);

    sample.temperature =
        uint32_t(b[3]) |
        (uint32_t(b[4]) << 8) |
        (uint32_t(b[5]) << 16);

    return true;
}

double BMP390::compensateTemperature(
    uint32_t raw, double& t_lin) const {

    double delta = double(raw) - calib_.t1;

    t_lin = delta * calib_.t2
          + delta * delta * calib_.t3;

    return t_lin;
}

double BMP390::compensatePressure(
    uint32_t raw, double t_lin) const {

    const double t2 = t_lin * t_lin;
    const double t3 = t2 * t_lin;
    const double p = double(raw);

    const double offset =
        calib_.p5 +
        calib_.p6 * t_lin +
        calib_.p7 * t2 +
        calib_.p8 * t3;

    const double sensitivity =
        calib_.p1 +
        calib_.p2 * t_lin +
        calib_.p3 * t2 +
        calib_.p4 * t3;

    const double nonlinear =
        p * p * (calib_.p9 + calib_.p10 * t_lin) +
        p * p * p * calib_.p11;

    return offset + p * sensitivity + nonlinear;
}

bool BMP390::read(Sample& sample) {
    RawSample raw{};

    if (!read(raw))
        return false;

    double t_lin = 0;

    sample.temperature_c =
        compensateTemperature(raw.temperature, t_lin);

    sample.pressure_pa =
        compensatePressure(raw.pressure, t_lin);

    return std::isfinite(sample.temperature_c) &&
           std::isfinite(sample.pressure_pa);
}

void BMP390::printCalibration() const {
    printf("Factory calibration coefficients:\n");

    printf("T1=%.8g  T2=%.8g  T3=%.8g\n",
           calib_.t1, calib_.t2, calib_.t3);

    printf("P1=%.8g  P2=%.8g  P3=%.8g  P4=%.8g\n",
           calib_.p1, calib_.p2, calib_.p3, calib_.p4);

    printf("P5=%.8g  P6=%.8g  P7=%.8g  P8=%.8g\n",
           calib_.p5, calib_.p6, calib_.p7, calib_.p8);

    printf("P9=%.8g  P10=%.8g  P11=%.8g\n",
           calib_.p9, calib_.p10, calib_.p11);
}
