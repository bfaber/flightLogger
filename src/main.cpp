
#include <cstdio>
#include <cstdint>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "bmp390.h"
#include "flight_computer.h"
#include "flash_logger.h"

namespace {
    constexpr uint SDA_PIN = 6;
    constexpr uint SCL_PIN = 7;
    constexpr uint32_t SAMPLE_INTERVAL_US = 100000;

    void initializeI2C() {
        i2c_init(i2c1, 100000);

        gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

        gpio_pull_up(SDA_PIN);
        gpio_pull_up(SCL_PIN);
    }

    void printCommands() {
        printf("\nCommands:\n");
        printf("  r = request new recording\n");
        printf("  Y = confirm erase and start\n");
        printf("  s = stop recording\n");
        printf("  d = dump recording\n");
        printf("  i = logger status\n\n");
    }
}

int main() {
    stdio_init_all();
    initializeI2C();

    BMP390 barometer(i2c1, 0x77);

    if (!barometer.initialize()) {
        while (true) {
            printf("BMP390 initialization failed\n");
            sleep_ms(1000);
        }
    }

    FlashLogger logger;

    if (!logger.initialize()) {
        while (true) {
            printf("Flash logger unavailable\n");
            sleep_ms(2000);
        }
    }

    FlightComputer computer(barometer);

    printf("Flight recorder ready.\n");
    printCommands();

    bool awaiting_confirmation = false;
    uint64_t recording_start_us = 0;

    absolute_time_t next_sample = get_absolute_time();

    while (true) {
        const int ch = getchar_timeout_us(0);

        if (ch != PICO_ERROR_TIMEOUT) {
            if (ch == 'r' && !logger.recording()) {
                awaiting_confirmation = true;
                printf("WARNING: This will erase the old log.\n");
                printf("Send uppercase Y to confirm.\n");
            }
            else if (ch == 'Y' && awaiting_confirmation) {
                awaiting_confirmation = false;

                uint8_t calibration[21];

                if (!barometer.readFactoryCalibration(
                        calibration)) {
                    printf("Calibration read failed.\n");
                }
                else if (logger.start(calibration)) {
                    recording_start_us = time_us_64();
                    printf("Recording started.\n");
                }
                else {
                    printf("Failed to start recording.\n");
                }

                next_sample = get_absolute_time();
            }
            else if (ch == 's') {
                awaiting_confirmation = false;

                if (logger.stop())
                    printf("Recording stopped.\n");
                else
                    printf("ERROR: Final flash write failed.\n");
            }
            else if (ch == 'd') {
                awaiting_confirmation = false;
                logger.dump();
            }
            else if (ch == 'i') {
                awaiting_confirmation = false;
                logger.printInfo();
            }
            else if (ch != '\r' && ch != '\n') {
                awaiting_confirmation = false;
            }
        }

        if (logger.recording() &&
            time_reached(next_sample)) {

            const uint64_t now = time_us_64();

            RawFlightSample sample{};

            if (computer.acquire(
                    static_cast<uint32_t>(
                        now - recording_start_us),
                    sample)) {

                if (!logger.append(sample)) {
                    printf("ERROR: Logging stopped.\n");
                }
            }

            next_sample =
                make_timeout_time_us(SAMPLE_INTERVAL_US);
        }

        sleep_ms(1);
    }
}

