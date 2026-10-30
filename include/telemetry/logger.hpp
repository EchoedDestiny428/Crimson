#pragma once

#include "pros/rtos.hpp"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string_view>

namespace telemetry {

class Logger {
public:
    bool open(std::string_view extra_header);
    bool is_open() const;
    const char* path() const;

    std::uint32_t entry(const char* key, const char* type);

    void record(std::uint32_t id, const void* data, std::size_t size, std::uint64_t timestamp);
    void record_boolean(std::uint32_t id, bool value, std::uint64_t timestamp);
    void record_int64(std::uint32_t id, std::int64_t value, std::uint64_t timestamp);
    void record_double(std::uint32_t id, double value, std::uint64_t timestamp);
    void record_string(std::uint32_t id, std::string_view value, std::uint64_t timestamp);

    template <std::size_t N>
    void record_struct(std::uint32_t id, const std::array<std::uint8_t, N>& bytes, std::uint64_t timestamp) {
        record(id, bytes.data(), N, timestamp);
    }

    void flush();

    std::uint64_t dropped() const;
    std::uint64_t bytes_written() const;

private:
    static constexpr std::size_t kBufferSize = 16 * 1024;
    static constexpr std::size_t kMaxEntries = 160;
    static constexpr std::uint32_t kMaxFileIndex = 9999;

    struct Entry {
        const char* key;
        std::uint32_t id;
    };

    bool append_record(std::uint32_t id, const void* data, std::size_t size, std::uint64_t timestamp);
    bool append_start(std::uint32_t entry, std::string_view key, std::string_view type, std::uint64_t timestamp);
    std::uint32_t find_entry(const char* key) const;
    bool choose_path();

    std::array<std::array<std::uint8_t, kBufferSize>, 2> buffers_{};
    std::size_t active_ = 0;
    std::size_t used_ = 0;
    std::array<Entry, kMaxEntries> entries_{};
    std::size_t entry_count_ = 0;
    std::FILE* file_ = nullptr;
    char path_[32]{};
    std::atomic<bool> open_{false};
    std::atomic<std::uint32_t> dropped_{0};
    std::atomic<std::uint32_t> bytes_written_{0};
    pros::Mutex buffer_mutex_;
    pros::Mutex file_mutex_;
};

extern Logger logger;

}
