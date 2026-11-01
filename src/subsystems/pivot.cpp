#include "subsystems/pivot.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace subsystems::pivot {

namespace {

constexpr double kSettleDeg = 25.0;             // loose until the bar is tuned. A timeout in macros.cpp aborts the claw if this never hits.
constexpr double kSettleRpm = 15.0;
constexpr std::uint32_t kPollMs = 10;

bool manual_active = false;

double clamped(double motor_deg) {
    return std::clamp(motor_deg, kMinDeg, kMaxDeg);
}

}

void init() {
    pivot_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pivot_motor.tare_position();                // zero is the raised bar. Park it there before booting.
}

void update() {
    const bool out = controller.get_digital(controls::kPivotOut);
    const bool in = controller.get_digital(controls::kPivotIn);
    if (out == in) {
        if (manual_active) {
            manual_active = false;
            const double current = get_pos();
            if (std::isfinite(current)) {
                move_to(current);
            } else {
                pivot_motor.brake();
            }
        }
        return;
    }

    manual_active = true;
    const int power = out ? kManualPower : -kManualPower;
    const double current = get_pos();
    const bool blocked =
        std::isfinite(current) && ((power > 0 && current >= kMaxDeg) || (power < 0 && current <= kMinDeg));
    if (blocked) {
        pivot_motor.brake();
    } else {
        pivot_motor.move(power);
    }
}

void move_to(double motor_deg) {
    manual_active = false;
    pivot_motor.move_absolute(clamped(motor_deg), kVelocity);
}

void stop() {
    manual_active = false;
    pivot_motor.brake();
}

void set_pos(double motor_deg) {
    pivot_motor.set_zero_position(motor_deg);
}

double get_pos() {
    return pivot_motor.get_position();
}

bool is_settled() {
    const double position = get_pos();
    const double velocity = pivot_motor.get_actual_velocity();
    const double target = pivot_motor.get_target_position();
    if (!std::isfinite(position) || !std::isfinite(velocity) || !std::isfinite(target)) {
        return false;
    }
    return std::fabs(position - target) <= kSettleDeg && std::fabs(velocity) <= kSettleRpm;
}

bool wait_until_settled(std::uint32_t timeout_ms) {
    const std::uint32_t start = pros::millis();
    while (!is_settled()) {
        if (pros::millis() - start >= timeout_ms) {
            return false;
        }
        pros::delay(kPollMs);
    }
    return true;
}

}
