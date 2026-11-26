
#include "flash_logger.h"

#include <cstdio>
#include <cstring>
#include <climits>

#include "pico/flash.h"
#include "hardware/regs/addressmap.h"

extern "C" {
    extern uint8_t __flash_binary_end;
}

namespace {
    constexpr uint32_t LOG_MAGIC  = 0x31474F4C; // LOG1
    constexpr uint32_t PAGE_MAGIC = 0x31474150; // PAG1
    constexpr uint32_t LOG_VERSION = 1;

    constexpr uint32_t HEADER_BYTES = 16;

    const uint8_t* flashAt(uint32_t offset) {
        return reinterpret_cast<const uint8_t*>(
            XIP_BASE + offset);
    }

    void put32(uint8_t* p, uint32_t value) {
        std::memcpy(p, &value, sizeof(value));
    }

    uint32_t get32(const uint8_t* p) {
        uint32_t value;
        std::memcpy(&value, p, sizeof(value));
        return value;
    }

    // FNV-1a checksum, excluding bytes 12..15.
    uint32_t checksum(const uint8_t* page) {
        uint32_t hash = 2166136261u;

        for (uint32_t i = 0; i < FLASH_PAGE_SIZE; ++i) {
            if (i >= 12 && i < 16)
                continue;

            hash ^= page[i];
            hash *= 16777619u;
        }

        return hash;
    }

    struct ProgramArgs {
        uint32_t offset;
        const uint8_t* data;
    };

    void programCallback(void* context) {
        auto* args = static_cast<ProgramArgs*>(context);
        flash_range_program(
            args->offset, args->data, FLASH_PAGE_SIZE);
    }

    void eraseCallback(void* context) {
        auto offset = static_cast<uint32_t*>(
            context);

        flash_range_erase(
            *offset, FlashLogger::REGION_SIZE);
    }

    bool validPage(const uint8_t* page,
                   uint32_t expected_sequence) {
        return get32(page) == PAGE_MAGIC &&
               get32(page + 4) == expected_sequence &&
               get32(page + 8) >= 1 &&
               get32(page + 8) <=
                   FlashLogger::SAMPLES_PER_PAGE &&
               get32(page + 12) == checksum(page);
    }
}

static_assert(
    FlashLogger::REGION_SIZE % FLASH_SECTOR_SIZE == 0);

static_assert(
    FlashLogger::REGION_SIZE < PICO_FLASH_SIZE_BYTES);

static_assert(
    HEADER_BYTES +
    FlashLogger::SAMPLES_PER_PAGE *
        sizeof(RawFlightSample) == FLASH_PAGE_SIZE);

bool FlashLogger::initialize() {
    // Firmware must end before our reserved flash area.
    const uintptr_t firmware_end =
        reinterpret_cast<uintptr_t>(&__flash_binary_end);

    const uintptr_t log_start =
        XIP_BASE + REGION_OFFSET;

    if (firmware_end > log_start) {
        printf("ERROR: Firmware overlaps flash log!\n");
        return false;
    }

    available_ = true;
    next_page_ = scanPages() + 1;
    committed_samples_ = 0;

    for (uint32_t page_index = 1;
         page_index < next_page_; ++page_index) {

        const uint8_t* p = flashAt(
            REGION_OFFSET + page_index * FLASH_PAGE_SIZE);

        committed_samples_ += get32(p + 8);
    }

    printf("Flash logger initialized.\n");
    printInfo();
    return true;
}

uint32_t FlashLogger::scanPages() const {
    const uint8_t* header = flashAt(REGION_OFFSET);

    if (get32(header) != LOG_MAGIC ||
        get32(header + 4) != LOG_VERSION)
        return 0;

    uint32_t count = 0;

    for (uint32_t i = 1; i <= MAX_PAGES; ++i) {
        const uint8_t* p = flashAt(
            REGION_OFFSET + i * FLASH_PAGE_SIZE);

        if (!validPage(p, i))
            break;

        ++count;

        // A partially filled final page ends the log.
        if (get32(p + 8) < SAMPLES_PER_PAGE)
            break;
    }

    return count;
}

void FlashLogger::preparePage() {
    std::memset(page_, 0xFF, sizeof(page_));
    buffered_samples_ = 0;
}

bool FlashLogger::programPage(
    uint32_t page_index, const uint8_t* data) {

    if (!available_ || page_index > MAX_PAGES)
        return false;

    const uint32_t offset =
        REGION_OFFSET + page_index * FLASH_PAGE_SIZE;

    // Never program a page that isn't erased.
    const uint8_t* existing = flashAt(offset);

    for (uint32_t i = 0; i < FLASH_PAGE_SIZE; ++i) {
        if (existing[i] != 0xFF)
            return false;
    }

    ProgramArgs args{offset, data};

    if (flash_safe_execute(
            programCallback, &args, 1000) != PICO_OK)
        return false;

    return std::memcmp(
        flashAt(offset), data, FLASH_PAGE_SIZE) == 0;
}

bool FlashLogger::start(
    const uint8_t (&calibration)[21]) {

    if (!available_ || recording_)
        return false;

    uint32_t offset = REGION_OFFSET;

    if (flash_safe_execute(
            eraseCallback, &offset, 1000) != PICO_OK)
        return false;

    uint8_t header[FLASH_PAGE_SIZE];
    std::memset(header, 0xFF, sizeof(header));

    put32(header, LOG_MAGIC);
    put32(header + 4, LOG_VERSION);
    put32(header + 8, 100000); // nominal sample interval, us

    std::memcpy(header + HEADER_BYTES,
                calibration, 21);

    if (!programPage(0, header))
        return false;

    next_page_ = 1;
    committed_samples_ = 0;
    preparePage();
    recording_ = true;

    return true;
}

bool FlashLogger::commitPage() {
    if (buffered_samples_ == 0)
        return true;

    if (next_page_ > MAX_PAGES)
        return false;

    put32(page_, PAGE_MAGIC);
    put32(page_ + 4, next_page_);
    put32(page_ + 8, buffered_samples_);
    put32(page_ + 12, checksum(page_));

    if (!programPage(next_page_, page_))
        return false;

    committed_samples_ += buffered_samples_;
    ++next_page_;
    preparePage();

    return true;
}

bool FlashLogger::append(
    const RawFlightSample& sample) {

    if (!recording_)
        return false;

    if (next_page_ > MAX_PAGES) {
        recording_ = false;
        return false;
    }

    std::memcpy(
        page_ + HEADER_BYTES +
            buffered_samples_ * sizeof(sample),
        &sample, sizeof(sample));

    ++buffered_samples_;

    if (buffered_samples_ == SAMPLES_PER_PAGE) {
        if (!commitPage()) {
            recording_ = false;
            return false;
        }
    }

    return true;
}

bool FlashLogger::stop() {
    if (!recording_)
        return true;

    if (!commitPage()) {
        recording_ = false;
        return false;
    }

    recording_ = false;
    return true;
}

void FlashLogger::printInfo() const {
    printf("Recording: %s\n",
           recording_ ? "yes" : "no");

    printf("Committed samples: %lu\n",
           static_cast<unsigned long>(
               committed_samples_));

    printf("Buffered samples: %lu\n",
           static_cast<unsigned long>(
               buffered_samples_));

    printf("Capacity: %lu samples\n",
           static_cast<unsigned long>(
               MAX_PAGES * SAMPLES_PER_PAGE));
}

void FlashLogger::dump() const {
    if (recording_) {
        printf("Stop recording before dumping.\n");
        return;
    }

    const uint8_t* header = flashAt(REGION_OFFSET);

    if (get32(header) != LOG_MAGIC ||
        get32(header + 4) != LOG_VERSION) {
        printf("No valid log header.\n");
        return;
    }

    printf("BEGIN_FLIGHT_LOG\n");

    printf("CAL,");
    for (uint32_t i = 0; i < 21; ++i)
        printf("%02X", header[HEADER_BYTES + i]);
    printf("\n");

    for (uint32_t i = 1; i <= MAX_PAGES; ++i) {
        const uint8_t* p = flashAt(
            REGION_OFFSET + i * FLASH_PAGE_SIZE);

        if (!validPage(p, i))
            break;

        const uint32_t count = get32(p + 8);

        for (uint32_t j = 0; j < count; ++j) {
            RawFlightSample sample;

            std::memcpy(
                &sample,
                p + HEADER_BYTES +
                    j * sizeof(sample),
                sizeof(sample));

            printf("S,%lu,%lu,%lu\n",
                static_cast<unsigned long>(
                    sample.timestamp_us),
                static_cast<unsigned long>(
                    sample.pressure_adc),
                static_cast<unsigned long>(
                    sample.temperature_adc));
        }

        if (count < SAMPLES_PER_PAGE)
            break;
    }

    printf("END_FLIGHT_LOG\n");
}
