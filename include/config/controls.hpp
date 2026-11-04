#pragma once

#include "pros/misc.hpp"

namespace controls {

using Axis = pros::controller_analog_e_t;
using Button = pros::controller_digital_e_t;

inline constexpr Axis kThrottle = pros::E_CONTROLLER_ANALOG_LEFT_Y;
inline constexpr Axis kTurn = pros::E_CONTROLLER_ANALOG_RIGHT_X;

inline constexpr Button kExtensionUp = pros::E_CONTROLLER_DIGITAL_L1;    // positive string voltage. Assumed to raise the carriage. The lift is a string, not a chain.
inline constexpr Button kExtensionDown = pros::E_CONTROLLER_DIGITAL_L2;
inline constexpr Button kElevatorUp = kExtensionUp;                      // same buttons. The merged cascade code reads kExtensionUp.
inline constexpr Button kElevatorDown = kExtensionDown;

inline constexpr Button kPivotOut = pros::E_CONTROLLER_DIGITAL_UP;       // positive bar degrees. Assumed to swing the claw down toward the stack.
inline constexpr Button kPivotIn = pros::E_CONTROLLER_DIGITAL_DOWN;

inline constexpr Button kClawClose = pros::E_CONTROLLER_DIGITAL_R1;      // sets the piston closed. A tap, not a hold.
inline constexpr Button kClawOpen = pros::E_CONTROLLER_DIGITAL_R2;       // sets the piston open.

inline constexpr Button kToggleIn = pros::E_CONTROLLER_DIGITAL_LEFT;     // hold. Spins the flex wheels. Outtake is not bound yet.

inline constexpr Button kPickup = pros::E_CONTROLLER_DIGITAL_X;
inline constexpr Button kScore = pros::E_CONTROLLER_DIGITAL_Y;
inline constexpr Button kStow = pros::E_CONTROLLER_DIGITAL_B;
inline constexpr Button kNextHeight = pros::E_CONTROLLER_DIGITAL_RIGHT;  // cycles high, mid, low. Does not move the bar. A is the IMU peak reset and is not used here.

}
