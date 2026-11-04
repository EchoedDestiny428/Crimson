#pragma once

#include <cstdint>

namespace ports {

inline constexpr std::int8_t kImu = 3;

inline constexpr std::int8_t kLeftDriveA = -17;
inline constexpr std::int8_t kLeftDriveB = -18;
inline constexpr std::int8_t kRightDriveA = 19;
inline constexpr std::int8_t kRightDriveB = 20;

inline constexpr std::int8_t kVerticalOdom = 4;
inline constexpr std::int8_t kHorizontalOdom = -11;

inline constexpr std::int8_t kElevatorA = 10;        // cascade, bottom motor. Unassigned. 
inline constexpr std::int8_t kElevatorB = -8;        // cascade, second bottom motor. . 
inline constexpr std::int8_t kPivot = 7;            // the bar's own motor, driving it through a chain. Negate if positive degrees swing the claw the wrong way.
inline constexpr std::int8_t kToggle = 6;           // flex-wheel motor. Unassigned. Negate if positive voltage spins the wheels the wrong way.

inline constexpr std::uint8_t kClaw = 'A';          // piston on the brain ADI, not a smart port. Unassigned. HIGH closes; flip claw::kClosedLevel if HIGH opens.

}
