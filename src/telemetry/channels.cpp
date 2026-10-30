#include "telemetry/channels.hpp"
#include "pros/error.h"
#include "telemetry/keys.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace telemetry {

namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kMilli = 0.001f;

}

float to_float(double value) {
    return std::isfinite(value) ? static_cast<float>(value) : kNaN;
}

float to_float(std::int32_t value, float scale) {
    return value == PROS_ERR ? kNaN : static_cast<float>(value) * scale;
}

double to_double(double value) {
    return std::isfinite(value) ? value : std::numeric_limits<double>::quiet_NaN();
}

BooleanChannel::BooleanChannel(const char* key) : id_(logger.entry(key, types::kBoolean)) {}

void BooleanChannel::record(bool value, std::uint64_t timestamp) {
    if (logged_ && value == last_) {
        return;
    }
    logger.record_boolean(id_, value, timestamp);
    last_ = value;
    logged_ = true;
}

StringChannel::StringChannel(const char* key) : id_(logger.entry(key, types::kString)) {}

void StringChannel::record(std::string_view value, std::uint64_t timestamp) {
    const std::size_t stored = std::min(value.size(), kCapacity);
    if (logged_ && value.size() == length_ && std::memcmp(value.data(), last_.data(), stored) == 0) {
        return;
    }
    logger.record_string(id_, value, timestamp);
    std::memcpy(last_.data(), value.data(), stored);
    length_ = value.size();
    logged_ = true;
}

MotorChannel::MotorChannel(pros::AbstractMotor& motor, std::uint8_t index, const char* key, const char* health_key)
    : motor_(&motor),
      index_(index),
      id_(logger.entry(key, structs::Motor::kType)),
      health_id_(logger.entry(health_key, structs::MotorHealth::kType)) {}

void MotorChannel::record(std::uint64_t timestamp) const {
    const structs::Motor sample{
        to_float(motor_->get_position(index_)),        to_float(motor_->get_actual_velocity(index_)),
        to_float(motor_->get_voltage(index_), kMilli), to_float(motor_->get_current_draw(index_), kMilli),
        to_float(motor_->get_power(index_)),           to_float(motor_->get_torque(index_)),
    };
    logger.record_struct(id_, sample.pack(), timestamp);
}

void MotorChannel::record_health(std::uint64_t timestamp) const {
    const structs::MotorHealth sample{
        to_float(motor_->get_temperature(index_)),
        to_float(motor_->get_efficiency(index_)),
        motor_->get_faults(index_),
        motor_->get_flags(index_),
        motor_->is_over_temp(index_) == 1,
        motor_->is_over_current(index_) == 1,
    };
    logger.record_struct(health_id_, sample.pack(), timestamp);
}

}
