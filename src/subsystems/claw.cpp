#include "subsystems/claw.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator.hpp"
#include <atomic>
#include <cmath>
#include <cstdint>

namespace subsystems::claw {

namespace {

constexpr int kManualPower = 127;
constexpr int kAutoSpinPower = 127;
constexpr double kHomeTolerance = 30.0;
constexpr std::uint32_t kLoopMs = 20;

std::atomic<int> driver_power{0};
std::atomic<bool> overridden{false};
std::atomic<int> override_power{0};

bool elevator_away_from_home() {
    if (!elevator::has_left_bootup()) {
        return false;
    }
    const double height = elevator::height();
    return std::isfinite(height) && std::fabs(height - elevator::kHome) > kHomeTolerance;
}

void apply(int power) {
    if (power == 0) {
        claw_motor.brake();
    } else {
        claw_motor.move(power);
    }
}

void run() {
    std::uint32_t now = pros::millis();
    while (true) {
        if (driver_power != 0) {
            apply(driver_power);
        } else if (overridden) {
            apply(override_power);
        } else {
            apply(elevator_away_from_home() ? kAutoSpinPower : 0);
        }
        pros::Task::delay_until(&now, kLoopMs);
    }
}

}

void init() {
    claw_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::Task task(run, "Claw");
}

void update() {
    const bool forward = controller.get_digital(controls::kClawForward);
    const bool reverse = controller.get_digital(controls::kClawReverse);
    driver_power = forward ? kManualPower : reverse ? -kManualPower : 0;
}

void spin(int power) {
    override_power = power;
    overridden = true;
}

void stop() {
    spin(0);
}

void automatic() {
    driver_power = 0;
    overridden = false;
}

void set_pos(double motor_deg) {
    claw_motor.set_zero_position(motor_deg);
}

}
