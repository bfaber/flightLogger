
#pragma once

#include <cstdint>

struct FlightData {
    uint64_t timestamp_us = 0;

    double pressure_pa = 0.0;
    double temperature_c = 0.0;

    double altitude_m = 0.0;
    double filtered_altitude_m = 0.0;
};
