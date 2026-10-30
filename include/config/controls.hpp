#pragma once

#include "pros/misc.hpp"

namespace controls {

using Axis = pros::controller_analog_e_t;

inline constexpr Axis kThrottle = pros::E_CONTROLLER_ANALOG_LEFT_Y;
inline constexpr Axis kTurn = pros::E_CONTROLLER_ANALOG_RIGHT_X;

}
