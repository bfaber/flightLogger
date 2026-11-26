
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <initializer_list>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bmp390.h"

constexpr uint SDA_PIN = 6;
constexpr uint SCL_PIN = 7;

constexpr uint32_t SAMPLE_INTERVAL_MS = 100;
constexpr uint32_t CALIBRATION_MS = 5000;

constexpr double METERS_TO_FEET = 3.28084;

// Relative altitude using the standard atmosphere approximation.
double relativeAltitudeMeters(double pressure_hpa,
                              double reference_hpa) {
    if (pressure_hpa <= 0 || reference_hpa <= 0)
        return 0.0;

    return 44330.0 *
        (1.0 - std::pow(pressure_hpa / reference_hpa,
                        0.19029496));
}

// Average pressure over a stationary calibration interval.
bool calibrate(BMP390& sensor, double& reference_hpa) {
    printf("\nCalibrating reference pressure...\n");
    printf("Keep the sensor stationary for 5 seconds.\n");

    double sum = 0.0;
    uint32_t count = 0;

    const uint32_t start =
        to_ms_since_boot(get_absolute_time());

    while (to_ms_since_boot(get_absolute_time()) - start
           < CALIBRATION_MS) {

        BMP390::Sample sample{};

        if (sensor.read(sample)) {
            sum += sample.pressure_hpa();
            ++count;
        }

        sleep_ms(SAMPLE_INTERVAL_MS);
    }

    if (count == 0) {
        printf("Calibration failed: no measurements.\n");
        return false;
    }

    reference_hpa = sum / count;

    printf("Reference: %.5f hPa (%lu samples)\n",
           reference_hpa,
           static_cast<unsigned long>(count));

    printf("Relative altitude zeroed.\n\n");
    return true;
}

int main() {
    stdio_init_all();

    i2c_init(i2c1, 100000);

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    BMP390 sensor(i2c1, 0x77);

    if (!sensor.initialize()) {
        while (true) {
            printf("BMP390 initialization failed\n");
            sleep_ms(1000);
        }
    }

    double reference_hpa = 0.0;

    if (!calibrate(sensor, reference_hpa)) {
        while (true)
            sleep_ms(1000);
    }

    printf("Altimeter running. Send 'z' to recalibrate.\n");

    while (true) {
        // Check USB serial input for a recalibration request.
        int ch = getchar_timeout_us(0);

        if (ch == 'z' || ch == 'Z') {
            calibrate(sensor, reference_hpa);
        }

        BMP390::Sample sample{};

        if (sensor.read(sample)) {
            double altitude_m = relativeAltitudeMeters(
                sample.pressure_hpa(), reference_hpa);

            double altitude_ft = altitude_m * METERS_TO_FEET;

            printf("Alt: %+7.2f ft | P: %.4f hPa | T: %.2f F\n",
                   altitude_ft,
                   sample.pressure_hpa(),
                   sample.temperature_f());
        }

        sleep_ms(SAMPLE_INTERVAL_MS);
    }
}


