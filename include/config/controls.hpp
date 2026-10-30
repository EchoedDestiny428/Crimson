#pragma once

#include "pros/misc.hpp"

namespace controls {

using Axis = pros::controller_analog_e_t;
using Button = pros::controller_digital_e_t;

inline constexpr Axis kThrottle = pros::E_CONTROLLER_ANALOG_LEFT_Y;
inline constexpr Axis kTurn = pros::E_CONTROLLER_ANALOG_RIGHT_X;

inline constexpr Button kExtensionUp = pros::E_CONTROLLER_DIGITAL_L1;
inline constexpr Button kExtensionDown = pros::E_CONTROLLER_DIGITAL_L2;

inline constexpr Button kPivotForward = pros::E_CONTROLLER_DIGITAL_UP;
inline constexpr Button kPivotReverse = pros::E_CONTROLLER_DIGITAL_DOWN;

inline constexpr Button kClawIn = pros::E_CONTROLLER_DIGITAL_R1;
inline constexpr Button kClawOut = pros::E_CONTROLLER_DIGITAL_R2;
inline constexpr Button kClawPiston = pros::E_CONTROLLER_DIGITAL_A;

}
