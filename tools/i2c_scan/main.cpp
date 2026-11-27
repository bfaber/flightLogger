#include <cstdio>
#include <cstdint>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

void verify_adxl375()
{
    constexpr uint8_t addr = 0x53;
    uint8_t reg = 0x00;
    uint8_t id = 0;

    int result = i2c_write_blocking(
        i2c1, addr, &reg, 1, true);

    if (result != 1)
    {
        printf("Failed to select ADXL375 register\n");
        return;
    }

    result = i2c_read_blocking(
        i2c1, addr, &id, 1, false);

    if (result != 1)
    {
        printf("Failed to read ADXL375 register\n");
        return;
    }

    printf("ADXL375 Device ID: 0x%02X (%s)\n",
           id, id == 0xE5 ? "OK" : "UNEXPECTED");
}

int main()
{
    stdio_init_all();

    i2c_init(i2c1, 400'000);

    gpio_set_function(6, GPIO_FUNC_I2C);
    gpio_set_function(7, GPIO_FUNC_I2C);

    gpio_pull_up(6);
    gpio_pull_up(7);

    while (true)
    {
        printf("\nScanning I2C bus...\n");

        for (uint8_t addr = 0x08; addr <= 0x77; ++addr)
        {
            uint8_t data;

            int result = i2c_read_timeout_us(
                i2c1,
                addr,
                &data,
                1,
                false,
                2000
            );

            if (result >= 0)
                printf("Device found at 0x%02X\n", addr);
        }

        printf("Scan complete.\n");

        sleep_ms(3000);

        printf("\nVerifying ADXL375 accelerometer...\n");
        verify_adxl375();
        printf("Verification complete.\n");
    }
}