#pragma once

#include "pros/misc.hpp"

namespace controls {

using Axis = pros::controller_analog_e_t;
using Button = pros::controller_digital_e_t;

inline constexpr Axis kThrottle = pros::E_CONTROLLER_ANALOG_LEFT_Y;
inline constexpr Axis kTurn = pros::E_CONTROLLER_ANALOG_RIGHT_X;

inline constexpr Button kStageSelectUp = pros::E_CONTROLLER_DIGITAL_UP;     // select the next stage up (max L7)
inline constexpr Button kStageSelectDown = pros::E_CONTROLLER_DIGITAL_DOWN; // select the next stage down (min L1)
inline constexpr Button kScore = pros::E_CONTROLLER_DIGITAL_L1; // hold: go to the selected stage; release: score


inline constexpr Button kClawPiston = pros::E_CONTROLLER_DIGITAL_A;

inline constexpr Button kHome = pros::E_CONTROLLER_DIGITAL_B; // home = intake pose
inline constexpr Button kOuttake = pros::E_CONTROLLER_DIGITAL_X; // claw toggle, same as A
inline constexpr Button kLiftManualUp = pros::E_CONTROLLER_DIGITAL_R1;   // hold: lift only, 100% up
inline constexpr Button kLiftManualDown = pros::E_CONTROLLER_DIGITAL_R2; // hold: lift only, 70% down
inline constexpr Button kGoalOffset = pros::E_CONTROLLER_DIGITAL_RIGHT; // v1's offset button: cycles alliance/neutral/center

}
