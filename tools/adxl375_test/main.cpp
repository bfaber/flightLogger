#include <cstdio>
#include <cmath>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "adxl375.h"

int main()
{
    stdio_init_all();

    i2c_init(i2c1, 400'000);

    gpio_set_function(6, GPIO_FUNC_I2C);
    gpio_set_function(7, GPIO_FUNC_I2C);

    gpio_pull_up(6);
    gpio_pull_up(7);

    ADXL375 accelerometer(i2c1);

    if (!accelerometer.initialize()) {
        while (true) {
            printf("ADXL375 initialization failed\n");
            sleep_ms(1000);
        }
    }

    while (true) {
        ADXL375::Sample sample{};

        if (accelerometer.read(sample)) {
            float magnitude = std::sqrt(
                sample.x_g * sample.x_g +
                sample.y_g * sample.y_g +
                sample.z_g * sample.z_g
            );

            printf(
                "X: %+7.3f  Y: %+7.3f  Z: %+7.3f"
                "  Magnitude: %.3f g\n",
                sample.x_g,
                sample.y_g,
                sample.z_g,
                magnitude
            );
        }

        sleep_ms(100);
    }
}