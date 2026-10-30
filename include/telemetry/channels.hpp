#pragma once

#include "pros/abstract_motor.hpp"
#include "telemetry/logger.hpp"
#include "telemetry/structs.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace telemetry {

float to_float(double value);
float to_float(std::int32_t value, float scale);
double to_double(double value);

class BooleanChannel {
public:
    explicit BooleanChannel(const char* key);
    void record(bool value, std::uint64_t timestamp);

private:
    std::uint32_t id_;
    bool last_ = false;
    bool logged_ = false;
};

class StringChannel {
public:
    explicit StringChannel(const char* key);
    void record(std::string_view value, std::uint64_t timestamp);

private:
    static constexpr std::size_t kCapacity = 64;

    std::uint32_t id_;
    std::array<char, kCapacity> last_{};
    std::size_t length_ = 0;
    bool logged_ = false;
};

template <typename Struct>
class StructChannel {
public:
    explicit StructChannel(const char* key) : id_(logger.entry(key, Struct::kType)) {}

    void record(const Struct& value, std::uint64_t timestamp) {
        const std::array<std::uint8_t, Struct::kSize> bytes = value.pack();
        if (logged_ && bytes == last_) {
            return;
        }
        logger.record_struct(id_, bytes, timestamp);
        last_ = bytes;
        logged_ = true;
    }

private:
    std::uint32_t id_;
    std::array<std::uint8_t, Struct::kSize> last_{};
    bool logged_ = false;
};

class MotorChannel {
public:
    MotorChannel(pros::AbstractMotor& motor, std::uint8_t index, const char* key, const char* health_key);
    void record(std::uint64_t timestamp) const;
    void record_health(std::uint64_t timestamp) const;

private:
    pros::AbstractMotor* motor_;
    std::uint8_t index_;
    std::uint32_t id_;
    std::uint32_t health_id_;
};

}
