#include "subsystems/elevator/claw.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include "util/motor_utils.hpp"

namespace subsystems::elevator::claw {

void init() {
    claw_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void update() {
    util::two_button(claw_motor, controls::kClawIn, controls::kClawOut);
    if (controller.get_digital_new_press(controls::kClawPiston)) {
        toggle();
    }
}

void stop() {
    claw_motor.brake();
}

void spin(int power) {
    claw_motor.move(power);
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

void set_pos(double motor_deg) {
    claw_motor.set_zero_position(motor_deg);
}

}
