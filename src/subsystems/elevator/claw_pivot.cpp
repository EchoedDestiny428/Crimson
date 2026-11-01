#include "subsystems/elevator/claw_pivot.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include <cmath>

namespace subsystems::elevator::claw_pivot {

namespace {

constexpr int kManualPower = 127;
constexpr int kMoveVelocity = 200;
constexpr double kSettleMotorDeg = 30.0;
constexpr std::uint32_t kWaitPollMs = 10;

bool manual_active = false;

}

void init() {
    claw_pivot_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    claw_pivot_motor.tare_position();
}

void update() {
    const bool forward = controller.get_digital(controls::kClawPivotForward);
    const bool reverse = controller.get_digital(controls::kClawPivotReverse);

    if (forward || reverse) {
        manual_active = true;
        claw_pivot_motor.move(forward ? kManualPower : -kManualPower);
    } else if (manual_active) {
        manual_active = false;
        claw_pivot_motor.brake();
    }
}

void stop() {
    manual_active = false;
    claw_pivot_motor.brake();
}

void set_pos(double motor_deg) {
    claw_pivot_motor.set_zero_position(motor_deg);
}

void move_to(double motor_deg) {
    claw_pivot_motor.move_absolute(motor_deg, kMoveVelocity);
}

bool is_settled() {
    return std::fabs(claw_pivot_motor.get_target_position() - claw_pivot_motor.get_position()) <= kSettleMotorDeg;
}

bool wait_until_settled(std::uint32_t timeout_ms) {
    const std::uint32_t start = pros::millis();
    while (!is_settled()) {
        if (pros::millis() - start >= timeout_ms) {
            return false;
        }
        pros::delay(kWaitPollMs);
    }
    return true;
}

double get_pos() {
    return claw_pivot_motor.get_position();
}

}
