#pragma once

#include <cstdint>

namespace ports {

inline constexpr std::int8_t kIntake = 2;
inline constexpr std::int8_t kVision = 16; // temp
inline constexpr std::int8_t kImu = 10;

inline constexpr std::int8_t kLeftDriveA = 1;
inline constexpr std::int8_t kLeftDriveB = -18;
inline constexpr std::int8_t kRightDriveA = 19;
inline constexpr std::int8_t kRightDriveB = -3;

inline constexpr std::int8_t kElevatorA = 4;
inline constexpr std::int8_t kElevatorB = -20;

inline constexpr std::int8_t kPivot = 5;
inline constexpr std::int8_t kClaw = 8;

inline constexpr std::int8_t kVerticalOdom = 17;
inline constexpr std::int8_t kHorizontalOdom = 21;

inline constexpr char kIntakeLift = 'A';

}
