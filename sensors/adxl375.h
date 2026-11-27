#pragma once

#include <cstddef>
#include <cstdint>
#include "hardware/i2c.h"
#include "I2CDevice.h"

class ADXL375 : public I2CDevice {
public:
    struct RawSample {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    struct Sample {
        float x_g;
        float y_g;
        float z_g;
    };

    explicit ADXL375(i2c_inst_t* i2c, uint8_t address = 0x53);

    bool initialize();
    bool read(RawSample& sample);
    bool read(Sample& sample);

private:
   static constexpr float SCALE_FACTOR = 0.049f;
};
