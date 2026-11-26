
#include "flight_computer.h"

FlightComputer::FlightComputer(BMP390& barometer)
    : barometer_(barometer) {}

bool FlightComputer::acquire(
    uint32_t timestamp_us,
    RawFlightSample& output) {

    BMP390::RawSample raw{};

    if (!barometer_.read(raw))
        return false;

    output.timestamp_us = timestamp_us;
    output.pressure_adc = raw.pressure;
    output.temperature_adc = raw.temperature;

    return true;
}
