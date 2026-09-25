#pragma once

#include "config/controls.hpp"
#include "pros/abstract_motor.hpp"

namespace util {

void two_button(pros::AbstractMotor& motor, controls::Button forward, controls::Button reverse, int power = 127);

}
