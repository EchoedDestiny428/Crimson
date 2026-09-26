#include "subsystems/intake.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include "util/motor_utils.hpp"
#include "util/system_check.hpp"

namespace subsystems::intake {

void update() {
    if (util::system_check::running()) {
        return;
    }
    util::two_button(intake_motor, controls::kIntakeIn, controls::kIntakeOut);
}

void stop() {
    intake_motor.brake();
}

void spin(int power) {
    intake_motor.move(power);
}

void lift() {
    intake_lift.retract();
}

void drop() {
    intake_lift.extend();
}

}
