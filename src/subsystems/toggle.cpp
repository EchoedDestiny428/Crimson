#include "subsystems/toggle.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"

namespace subsystems::toggle {

namespace {

bool manual_active = false;

}

void init() {
    toggle_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); // coast when released so the wheels do not drag the piece. Use HOLD if they should stay locked.
}

void update() {
    if (controller.get_digital(controls::kToggleIn)) {
        manual_active = true;
        toggle_motor.move(kInPower);
        return;
    }
    if (manual_active) {
        manual_active = false;
        stop();
    }
}

void stop() {
    manual_active = false;
    toggle_motor.brake();
}

void spin(int power) {
    manual_active = false;
    toggle_motor.move(power);
}

}
