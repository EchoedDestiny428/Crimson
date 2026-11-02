#include "subsystems/side_toggle.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"

namespace subsystems::side_toggle {

namespace {

bool manual_active = false;

}

void init() {
    side_toggle_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); // coast so the wheels don't drag the piece
}

void update() {
    const bool in = controller.get_digital(controls::kSideToggleIn);
    const bool out = controller.get_digital(controls::kSideToggleOut);
    if (in != out) {
        manual_active = true;
        side_toggle_motor.move(in ? kInPower : kOutPower);
        return;
    }
    if (manual_active) {
        stop();
    }
}

void stop() {
    manual_active = false;
    side_toggle_motor.brake();
}

void spin(int power) {
    manual_active = false;
    side_toggle_motor.move(power);
}

}
