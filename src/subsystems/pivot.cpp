#include "subsystems/pivot.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include <atomic>

namespace subsystems::pivot {

namespace {

constexpr double kGearRatio = 1.0;
constexpr int kManualPower = 127;
constexpr int kMacroVelocity = 300;

std::atomic<bool> macro_active{false};
std::atomic<double> macro_target{0.0};
bool manual_active = false;

}

void init() {
    pivot_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void update() {
    const bool forward = controller.get_digital(controls::kPivotForward);
    const bool reverse = controller.get_digital(controls::kPivotReverse);

    if (forward || reverse) {
        macro_active = false;
        manual_active = true;
        pivot_motor.move(forward ? kManualPower : -kManualPower);
    } else if (manual_active) {
        manual_active = false;
        pivot_motor.brake();
    }
}

void stop() {
    manual_active = false;
    macro_active = false;
    pivot_motor.brake();
}

void rotate_by(double arm_deg) {
    const double base = macro_active ? macro_target.load() : pivot_motor.get_position();
    macro_target = base + arm_deg * kGearRatio;
    macro_active = true;
    pivot_motor.move_absolute(macro_target, kMacroVelocity);
}

}
