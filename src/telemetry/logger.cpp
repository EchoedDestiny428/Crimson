#include "telemetry/logger.hpp"
#include "lemlib/logger/logger.hpp"
#include "pros/misc.hpp"
#include "telemetry/wpilog.hpp"
#include <bitset>
#include <cstring>
#include <limits>
#include <mutex>

namespace telemetry {

Logger logger;

namespace {

constexpr std::string_view kFilePrefix = "crimson_";
constexpr std::string_view kFileSuffix = ".wpilog";
constexpr std::size_t kIndexDigits = 4;
constexpr std::size_t kListBufferSize = 8192;
constexpr std::size_t kMaxExtraHeader = 116;

std::uint32_t parse_index(std::string_view name) {
    if (name.size() != kFilePrefix.size() + kIndexDigits + kFileSuffix.size() || !name.starts_with(kFilePrefix) ||
        !name.ends_with(kFileSuffix)) {
        return 0;
    }
    std::uint32_t index = 0;
    for (const char digit : name.substr(kFilePrefix.size(), kIndexDigits)) {
        if (digit < '0' || digit > '9') {
            return 0;
        }
        index = index * 10 + static_cast<std::uint32_t>(digit - '0');
    }
    return index;
}

bool file_exists(const char* path) {
    std::FILE* existing = std::fopen(path, "rb");
    if (existing == nullptr) {
        return false;
    }
    std::fclose(existing);
    return true;
}

}

bool Logger::choose_path() {
    static std::bitset<kMaxFileIndex + 1> listed;
    static char names[kListBufferSize];
    if (pros::usd::list_files("/", names, static_cast<std::int32_t>(sizeof(names))) == 1) {
        names[sizeof(names) - 1] = '\0';
        std::string_view remaining(names);
        while (!remaining.empty()) {
            const std::size_t end = remaining.find('\n');
            std::string_view name = remaining.substr(0, end);
            if (!name.empty() && name.back() == '\r') {
                name.remove_suffix(1);
            }
            const std::uint32_t index = parse_index(name);
            if (index > 0 && index <= kMaxFileIndex) {
                listed.set(index);
            }
            if (end == std::string_view::npos) {
                break;
            }
            remaining.remove_prefix(end + 1);
        }
    }

    for (std::uint32_t index = 1; index <= kMaxFileIndex; ++index) {
        if (listed.test(index)) {
            continue;
        }
        std::snprintf(path_, sizeof(path_), "/usd/crimson_%04lu.wpilog", static_cast<unsigned long>(index));
        if (!file_exists(path_)) {
            return true;
        }
    }
    return false;
}

bool Logger::open(std::string_view extra_header) {
    if (is_open()) {
        return true;
    }
    if (pros::usd::is_installed() == 0) {
        lemlib::infoSink()->warn("Telemetry: SD card is not installed, logging is off");
        return false;
    }
    if (!choose_path()) {
        lemlib::infoSink()->warn("Telemetry: no free log file name on the SD card, logging is off");
        return false;
    }
    file_ = std::fopen(path_, "wb");
    if (file_ == nullptr) {
        lemlib::infoSink()->warn("Telemetry: unable to open {}, logging is off", path_);
        return false;
    }

    const std::string_view extra = extra_header.substr(0, kMaxExtraHeader);
    std::array<std::uint8_t, wpilog::file_header_size({}) + kMaxExtraHeader> header{};
    const std::size_t size = wpilog::encode_file_header(header.data(), extra);
    if (std::fwrite(header.data(), 1, size, file_) != size) {
        std::fclose(file_);
        file_ = nullptr;
        lemlib::infoSink()->warn("Telemetry: unable to write to {}, logging is off", path_);
        return false;
    }
    std::fflush(file_);
    bytes_written_ = static_cast<std::uint32_t>(size);
    open_ = true;
    return true;
}

bool Logger::is_open() const {
    return open_;
}

const char* Logger::path() const {
    return path_;
}

std::uint32_t Logger::find_entry(const char* key) const {
    for (std::size_t i = 0; i < entry_count_; ++i) {
        if (std::strcmp(entries_[i].key, key) == 0) {
            return entries_[i].id;
        }
    }
    return wpilog::kControlId;
}

std::uint32_t Logger::entry(const char* key, const char* type) {
    if (!is_open() || key == nullptr || type == nullptr) {
        return wpilog::kControlId;
    }
    for (int attempt = 0; attempt < 2; ++attempt) {
        {
            std::lock_guard<pros::Mutex> lock(buffer_mutex_);
            const std::uint32_t existing = find_entry(key);
            if (existing != wpilog::kControlId) {
                return existing;
            }
            if (entry_count_ >= kMaxEntries) {
                return wpilog::kControlId;
            }
            const auto id = static_cast<std::uint32_t>(entry_count_ + 1);
            if (append_start(id, key, type, pros::micros())) {
                entries_[entry_count_++] = {key, id};
                return id;
            }
        }
        flush();
    }
    ++dropped_;
    return wpilog::kControlId;
}

bool Logger::append_start(std::uint32_t entry, std::string_view key, std::string_view type, std::uint64_t timestamp) {
    const std::size_t payload = wpilog::start_payload_size(key, type);
    const std::size_t header =
        wpilog::record_header_size(wpilog::kControlId, static_cast<std::uint32_t>(payload), timestamp);
    if (used_ + header + payload > kBufferSize) {
        return false;
    }
    std::uint8_t* out = buffers_[active_].data() + used_;
    wpilog::encode_record_header(out, wpilog::kControlId, static_cast<std::uint32_t>(payload), timestamp);
    wpilog::encode_start_payload(out + header, entry, key, type);
    used_ += header + payload;
    return true;
}

bool Logger::append_record(std::uint32_t id, const void* data, std::size_t size, std::uint64_t timestamp) {
    if (size > std::numeric_limits<std::uint32_t>::max()) {
        return false;
    }
    const auto payload = static_cast<std::uint32_t>(size);
    const std::size_t header = wpilog::record_header_size(id, payload, timestamp);
    if (used_ + header + size > kBufferSize) {
        return false;
    }
    std::uint8_t* out = buffers_[active_].data() + used_;
    wpilog::encode_record_header(out, id, payload, timestamp);
    if (size > 0) {
        std::memcpy(out + header, data, size);
    }
    used_ += header + size;
    return true;
}

void Logger::record(std::uint32_t id, const void* data, std::size_t size, std::uint64_t timestamp) {
    if (id == wpilog::kControlId || !is_open()) {
        return;
    }
    std::lock_guard<pros::Mutex> lock(buffer_mutex_);
    if (!append_record(id, data, size, timestamp)) {
        ++dropped_;
    }
}

void Logger::record_boolean(std::uint32_t id, bool value, std::uint64_t timestamp) {
    const std::uint8_t byte = value ? 1 : 0;
    record(id, &byte, sizeof(byte), timestamp);
}

void Logger::record_int64(std::uint32_t id, std::int64_t value, std::uint64_t timestamp) {
    record(id, &value, sizeof(value), timestamp);
}

void Logger::record_double(std::uint32_t id, double value, std::uint64_t timestamp) {
    record(id, &value, sizeof(value), timestamp);
}

void Logger::record_string(std::uint32_t id, std::string_view value, std::uint64_t timestamp) {
    record(id, value.data(), value.size(), timestamp);
}

void Logger::flush() {
    if (!is_open()) {
        return;
    }
    std::lock_guard<pros::Mutex> file_lock(file_mutex_);
    if (file_ == nullptr) {
        return;
    }
    std::size_t index = 0;
    std::size_t size = 0;
    {
        std::lock_guard<pros::Mutex> lock(buffer_mutex_);
        if (used_ == 0) {
            return;
        }
        index = active_;
        size = used_;
        active_ ^= 1;
        used_ = 0;
    }
    const std::size_t written = std::fwrite(buffers_[index].data(), 1, size, file_);
    bytes_written_ += static_cast<std::uint32_t>(written);
    if (written == size && std::fflush(file_) == 0) {
        return;
    }
    ++dropped_;
    open_ = false;
    std::fclose(file_);
    file_ = nullptr;
    lemlib::infoSink()->warn("Telemetry: unable to write to {}, logging is off", path_);
}

std::uint64_t Logger::dropped() const {
    return dropped_;
}

std::uint64_t Logger::bytes_written() const {
    return bytes_written_;
}

}
