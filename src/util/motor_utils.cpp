#include "util/motor_utils.hpp"
#include "robotconfig.hpp"

namespace util {

void two_button(pros::AbstractMotor& motor, controls::Button forward, controls::Button reverse, int power) {
    if (controller.get_digital(forward)) {
        motor.move(power);
    } else if (controller.get_digital(reverse)) {
        motor.move(-power);
    } else {
        motor.brake();
    }
}

}
