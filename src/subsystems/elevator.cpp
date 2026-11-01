#include "subsystems/elevator.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace subsystems::elevator {

namespace {

constexpr double kSettleDeg = 25.0;             // loose until the chain is tuned. Tighten if macros say they arrived while it is still moving.
constexpr double kSettleRpm = 15.0;
constexpr std::uint32_t kPollMs = 10;

bool manual_active = false;

double clamped(double motor_deg) {
    return std::clamp(motor_deg, kDownDeg, kMaxDeg);
}

}

void init() {
    elevator_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    elevator_motors.tare_position_all();        // zero is the pose at startup. Park the carriage at the bottom before booting.
}

void update() {
    const bool up = controller.get_digital(controls::kElevatorUp);
    const bool down = controller.get_digital(controls::kElevatorDown);
    if (up == down) {
        if (manual_active) {
            manual_active = false;
            hold();
        }
        return;
    }

    manual_active = true;
    const int power = up ? kManualPower : -kManualPower;
    const double current = height();
    const bool blocked = std::isfinite(current) &&
                         ((power > 0 && current >= kMaxDeg) || (power < 0 && current <= kDownDeg));
    if (blocked) {
        hold();
    } else {
        elevator_motors.move(power);
    }
}

void stop() {
    manual_active = false;
    hold();
}

void set_target(double motor_deg) {
    manual_active = false;
    elevator_motors.move_absolute(clamped(motor_deg), kVelocity);
}

void hold() {
    const double current = height();
    if (!std::isfinite(current)) {
        elevator_motors.brake();
        return;
    }
    set_target(current);
}

void set_pos(double motor_deg) {
    elevator_motors.set_zero_position_all(motor_deg);
}

double height() {
    return elevator_motors.get_position();
}

bool is_settled() {
    const double position = height();
    const double velocity = elevator_motors.get_actual_velocity();
    const double target = elevator_motors.get_target_position(0);
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
