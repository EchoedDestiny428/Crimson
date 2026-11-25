#include "subsystems/elevator/claw.hpp"
#include "config/controls.hpp"
#include "pros/rtos.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator/constants.hpp"
#include "subsystems/elevator/extension.hpp"

namespace subsystems::elevator::claw {

namespace {

bool lift_pending = false; // clamped, waiting kClampLiftDelayMs before lifting
std::uint32_t clamped_at_ms = 0;

// Lifts to kClampLiftCm if the lift is headed anywhere lower, so the clamped cup clears the
// floor. A lift already above that stays where it is.
void lift_off_floor() {
    const double clear_deg = constants::extension_cm_to_motor_deg(constants::kClampLiftCm);
    if (!(extension::target() >= clear_deg)) { // also catches a NaN target
        extension::set_target(clear_deg);
    }
}

}

void update() {
    if (controller.get_digital_new_press(controls::kClawPiston) ||
        controller.get_digital_new_press(controls::kOuttake)) {
        toggle();
        lift_pending = claw_piston.is_extended(); // just clamped; opening cancels a pending lift
        clamped_at_ms = pros::millis();
    }
    if (lift_pending && pros::millis() - clamped_at_ms >= static_cast<std::uint32_t>(constants::kClampLiftDelayMs)) {
        lift_pending = false;
        lift_off_floor();
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

void prime() {
    open();
}

void release() {
    open();
}

}
