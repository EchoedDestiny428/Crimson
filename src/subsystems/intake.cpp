#include "subsystems/intake.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include "util/motor_utils.hpp"

namespace subsystems::intake {

void update() {
    util::two_button(intake_motor, controls::kIntakeIn, controls::kIntakeOut);
}

void stop() {
    intake_motor.brake();
}

void spin(int power) {
    intake_motor.move(power);
}

}
