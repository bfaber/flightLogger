add_executable(adxl375_test
    main.cpp
    ../../sensors/adxl375.cpp
    ../../sensors/I2CDevice.cpp
)

target_include_directories(adxl375_test PRIVATE
    ../../sensors
)

target_link_libraries(adxl375_test PRIVATE
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(adxl375_test 1)
pico_enable_stdio_uart(adxl375_test 0)

pico_add_extra_outputs(adxl375_test)