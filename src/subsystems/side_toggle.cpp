#include "subsystems/side_toggle.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"

namespace subsystems::side_toggle {

namespace {

bool manual_active = false;

}

void init() {
    side_toggle_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
}

void update() {
    const bool forward = controller.get_digital(controls::kSideToggleForward);
    const bool reverse = controller.get_digital(controls::kSideToggleReverse);
    if (forward != reverse) {
        manual_active = true;
        side_toggle_motors.move(forward ? kForwardPower : kReversePower);
        return;
    }
    if (manual_active) {
        stop();
    }
}

void stop() {
    manual_active = false;
    side_toggle_motors.brake();
}

void spin(int power) {
    manual_active = false;
    side_toggle_motors.move(power);
}

}
