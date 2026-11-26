
#pragma once

#include <cstdint>

#include "bmp390.h"
#include "raw_flight_sample.h"

class FlightComputer {
public:
    explicit FlightComputer(BMP390& barometer);

    bool acquire(uint32_t timestamp_us,
                 RawFlightSample& sample);

private:
    BMP390& barometer_;
};
