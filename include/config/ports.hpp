#pragma once

#include <cstdint>

namespace ports {

inline constexpr std::int8_t kIntake = 12;
inline constexpr std::int8_t kVision = 16; // temp
inline constexpr std::int8_t kImu = 20;

inline constexpr std::int8_t kLeftDriveA = 11;
inline constexpr std::int8_t kLeftDriveB = -18;
inline constexpr std::int8_t kRightDriveA = -1;
inline constexpr std::int8_t kRightDriveB = 19;

inline constexpr std::int8_t kElevatorA = -10;
inline constexpr std::int8_t kElevatorB = 8;

inline constexpr std::int8_t kPivot = 7;
inline constexpr std::int8_t kClaw = 4;

inline constexpr std::int8_t kVerticalOdom = 17;
inline constexpr std::int8_t kHorizontalOdom = 16;

inline constexpr char kIntakeLift = 'A';
inline constexpr char kPullToggle = 'B';

}
