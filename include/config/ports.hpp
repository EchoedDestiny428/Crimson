#pragma once

#include <cstdint>

namespace ports {

inline constexpr std::int8_t kIntake = 1;
inline constexpr std::int8_t kVision = 5;
inline constexpr std::int8_t kImu = 6;

inline constexpr std::int8_t kLeftDriveA = 3;
inline constexpr std::int8_t kLeftDriveB = -20;
inline constexpr std::int8_t kRightDriveA = -2;
inline constexpr std::int8_t kRightDriveB = 19;

inline constexpr std::int8_t kElevatorA = 4;
inline constexpr std::int8_t kElevatorB = -18;

inline constexpr std::int8_t kPivot = 11;
inline constexpr std::int8_t kClaw = 12;

}
