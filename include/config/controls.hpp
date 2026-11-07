#pragma once

#include "pros/misc.hpp"

namespace controls {

using Axis = pros::controller_analog_e_t;
using Button = pros::controller_digital_e_t;

inline constexpr Axis kThrottle = pros::E_CONTROLLER_ANALOG_LEFT_Y;
inline constexpr Axis kTurn = pros::E_CONTROLLER_ANALOG_RIGHT_X;

inline constexpr Button kStageDown = pros::E_CONTROLLER_DIGITAL_L1; // down a scoring stage (min L1)
inline constexpr Button kStageUp = pros::E_CONTROLLER_DIGITAL_L2;   // up a scoring stage (max L7)

inline constexpr Button kArmPivotForward = pros::E_CONTROLLER_DIGITAL_UP;
inline constexpr Button kArmPivotReverse = pros::E_CONTROLLER_DIGITAL_DOWN;


inline constexpr Button kClawPiston = pros::E_CONTROLLER_DIGITAL_A;

inline constexpr Button kHome = pros::E_CONTROLLER_DIGITAL_B; // home = intake pose
inline constexpr Button kClawToggle = pros::E_CONTROLLER_DIGITAL_X;  // claw piston toggle (port A), same as A
inline constexpr Button kLiftManualUp = pros::E_CONTROLLER_DIGITAL_R1;   // hold: lift only, 100% up
inline constexpr Button kLiftManualDown = pros::E_CONTROLLER_DIGITAL_R2; // hold: lift only, 70% down
inline constexpr Button kGoalOffset = pros::E_CONTROLLER_DIGITAL_RIGHT; // v1's offset button: cycles alliance/neutral/center

}
