#include "subsystems/elevator/claw.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"

namespace subsystems::elevator::claw {

void update() {
    if (controller.get_digital_new_press(controls::kClawPiston) ||
        controller.get_digital_new_press(controls::kOuttake)) {
        toggle();
    }
}

void toggle() {
    claw_piston.toggle();
}

// Measured on the robot: the piston retracted is the open claw, extended is closed.
void open() {
    claw_piston.retract();
}

void close() {
    claw_piston.extend();
}

void grab() {
    close();
}

}
