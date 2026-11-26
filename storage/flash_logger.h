
#pragma once

#include <cstddef>
#include <cstdint>

#include "hardware/flash.h"
#include "raw_flight_sample.h"

class FlashLogger {
public:
    static constexpr uint32_t REGION_SIZE = 64 * 1024;
    static constexpr uint32_t REGION_OFFSET =
        PICO_FLASH_SIZE_BYTES - REGION_SIZE;

    static constexpr uint32_t SAMPLES_PER_PAGE = 20;
    static constexpr uint32_t MAX_PAGES =
        REGION_SIZE / FLASH_PAGE_SIZE - 1;

    bool initialize();
    bool start(const uint8_t (&calibration)[21]);
    bool append(const RawFlightSample& sample);
    bool stop();

    void dump() const;
    void printInfo() const;

    bool recording() const { return recording_; }

private:
    bool commitPage();
    bool programPage(uint32_t page_index,
                     const uint8_t* data);
    void preparePage();
    uint32_t scanPages() const;

    uint8_t page_[FLASH_PAGE_SIZE]{};

    uint32_t next_page_ = 1;
    uint32_t committed_samples_ = 0;
    uint32_t buffered_samples_ = 0;

    bool recording_ = false;
    bool available_ = false;
};
