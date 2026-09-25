#include "subsystems/elevator.hpp"
#include "lemlib/smartMotor.hpp"
#include "robotconfig.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>

namespace subsystems::elevator {

namespace {

constexpr std::int32_t kMaxVelocity = 600;
constexpr float kSettleRange = 5.0f;

constexpr double kSnapDownFraction = 0.6;

constexpr auto kStages = [] {
    std::array stages{kFlipOut, kStage1, kStage2, kStage3, kStage4, kStage5};
    std::sort(stages.begin(), stages.end());
    return stages;
}();

lemlib::SmartMotor motor(&elevator_motors, kMaxVelocity, kSettleRange);
std::atomic<bool> left_bootup{false};

double min_height() {
    return left_bootup ? kHome : kBootup;
}

double nearest_stage(double current) {
    if (current <= kStages.front()) {
        return kStages.front();
    }
    for (std::size_t i = 1; i < kStages.size(); ++i) {
        const double lower = kStages[i - 1];
        const double upper = kStages[i];
        if (current < upper) {
            return current <= lower + kSnapDownFraction * (upper - lower) ? lower : upper;
        }
    }
    return kStages.back();
}

}

void init() {
    elevator_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    motor.reset();
    motor.setLogging(true);
    motor.start();
}

void hold() {
    const double current = height();
    if (!std::isfinite(current)) {
        motor.holdCurrent();
    } else if (target() < kHome) {
        set_target_below_home(current);
    } else {
        set_target(current);
    }
}

void set_target(double target) {
    if (target >= kHome) {
        left_bootup = true;
    }
    motor.setTarget(static_cast<float>(std::clamp(target, min_height(), kMax)));
}

void set_target_below_home(double target) {
    motor.setTarget(static_cast<float>(std::clamp(target, kBootup, kMax)));
}

void set_manual(int power) {
    const double current = height();
    const bool at_limit = (power > 0 && current >= kMax) || (power < 0 && current <= min_height());
    if (at_limit) {
        hold();
    } else {
        motor.setManual(power);
    }
}

double snap_to_stage() {
    const double current = height();
    if (!std::isfinite(current)) {
        hold();
        return current;
    }
    const double stage = nearest_stage(current);
    set_target(stage);
    return stage;
}

double top_stage() {
    return kStages.back();
}

double height() {
    return motor.getRotation();
}

double target() {
    return motor.getTarget();
}

bool has_left_bootup() {
    return left_bootup;
}

bool is_settled() {
    return motor.isSettled();
}

bool wait_until_settled(std::uint32_t timeout_ms) {
    return motor.waitUntilSettled(timeout_ms);
}

}
