
#pragma once

#include <cstdint>

struct RawFlightSample {
    uint32_t timestamp_us;
    uint32_t pressure_adc;
    uint32_t temperature_adc;
};

static_assert(sizeof(RawFlightSample) == 12);
