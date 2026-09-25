#pragma once

#include "pros/misc.hpp"

namespace controls {

using Axis = pros::controller_analog_e_t;
using Button = pros::controller_digital_e_t;

inline constexpr Axis kThrottle = pros::E_CONTROLLER_ANALOG_LEFT_Y;
inline constexpr Axis kTurn = pros::E_CONTROLLER_ANALOG_RIGHT_X;

inline constexpr Button kIntakeIn = pros::E_CONTROLLER_DIGITAL_R1;
inline constexpr Button kIntakeOut = pros::E_CONTROLLER_DIGITAL_R2;

inline constexpr Button kClawForward = pros::E_CONTROLLER_DIGITAL_UP;
inline constexpr Button kClawReverse = pros::E_CONTROLLER_DIGITAL_DOWN;

inline constexpr Button kPivotForward = pros::E_CONTROLLER_DIGITAL_X;
inline constexpr Button kPivotReverse = pros::E_CONTROLLER_DIGITAL_B;

inline constexpr Button kElevatorUp = pros::E_CONTROLLER_DIGITAL_L1;
inline constexpr Button kElevatorDown = pros::E_CONTROLLER_DIGITAL_L2;
inline constexpr Button kElevatorStepUp = pros::E_CONTROLLER_DIGITAL_RIGHT;
inline constexpr Button kElevatorStepDown = pros::E_CONTROLLER_DIGITAL_LEFT;
inline constexpr Button kElevatorHome = pros::E_CONTROLLER_DIGITAL_Y;

inline constexpr Button kPivotAndRaise = pros::E_CONTROLLER_DIGITAL_A;

}
