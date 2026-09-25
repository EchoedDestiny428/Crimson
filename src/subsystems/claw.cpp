#include "subsystems/claw.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include "util/motor_utils.hpp"

namespace subsystems::claw {

void init() {
    claw_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void update() {
    util::two_button(claw_motor, controls::kClawForward, controls::kClawReverse);
}

void stop() {
    claw_motor.brake();
}

}
