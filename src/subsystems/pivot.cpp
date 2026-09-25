#include "subsystems/pivot.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"

namespace subsystems::pivot {

namespace {

constexpr int kManualPower = 127;
constexpr int kMoveVelocity = 200;

bool manual_active = false;

}

void init() {
    pivot_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pivot_motor.tare_position();
}

void update() {
    const bool forward = controller.get_digital(controls::kPivotForward);
    const bool reverse = controller.get_digital(controls::kPivotReverse);

    if (forward || reverse) {
        manual_active = true;
        pivot_motor.move(forward ? kManualPower : -kManualPower);
    } else if (manual_active) {
        manual_active = false;
        pivot_motor.brake();
    }
}

void stop() {
    manual_active = false;
    pivot_motor.brake();
}

void move_to(double motor_deg) {
    pivot_motor.move_absolute(motor_deg, kMoveVelocity);
}

}
