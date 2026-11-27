#pragma once

#include <cstddef>
#include <cstdint>
#include "hardware/i2c.h"
#include "I2CDevice.h"

class BMP390 : public I2CDevice {
public:
    struct RawSample {
        uint32_t pressure;
        uint32_t temperature;
    };

    struct Sample {
        double pressure_pa;
        double temperature_c;

        double pressure_hpa() const {
            return pressure_pa / 100.0;
        }

        double temperature_f() const {
            return temperature_c * 1.8 + 32.0;
        }
    };

    explicit BMP390(i2c_inst_t* i2c, uint8_t address = 0x77);

    bool initialize();
    bool read(RawSample& sample);
    bool read(Sample& sample);

    void printCalibration() const;
    bool readFactoryCalibration(uint8_t (&bytes)[21]);
        
private:
    struct Calibration {
        double t1, t2, t3;
        double p1, p2, p3, p4, p5, p6;
        double p7, p8, p9, p10, p11;
    };

    bool readCalibration();
    double compensateTemperature(uint32_t raw, double& t_lin) const;
    double compensatePressure(uint32_t raw, double t_lin) const;

    Calibration calib_{};
};