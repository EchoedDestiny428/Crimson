#include "subsystems/elevator/claw.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"

namespace subsystems::elevator::claw {

void update() {
    if (controller.get_digital_new_press(controls::kClawPiston)) {
        toggle();
    }
}

void open() {
    claw_piston.extend();
}

void close() {
    claw_piston.retract();
}

void toggle() {
    claw_piston.toggle();
}

bool is_open() {
    return claw_piston.is_extended();
}

std::int32_t distance_mm() {
    return claw_distance.get_distance();
}

}
