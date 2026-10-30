#include "subsystems/intake.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include "util/motor_utils.hpp"
#include "util/system_check.hpp"
#include <atomic>

namespace subsystems::intake {

namespace {

std::atomic<bool> raised{false};

}

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
    intake_lift.set_value(HIGH);
    raised = true;
}

void drop() {
    intake_lift.set_value(LOW);
    raised = false;
}

void set_pos(double motor_deg) {
    intake_motor.set_zero_position(motor_deg);
}

bool lifted() {
    return raised;
}

}
