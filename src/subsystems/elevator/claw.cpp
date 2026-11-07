#include "subsystems/elevator/claw.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"

namespace subsystems::elevator::claw {

void update() {
    if (controller.get_digital_new_press(controls::kClawPiston) ||
        controller.get_digital_new_press(controls::kClawToggle)) {
        toggle();
    }
}

void toggle() {
    claw_piston.toggle();
}

void grab() {
    claw_piston.retract();
}

}
