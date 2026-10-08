#pragma once

#include <cstdint>

namespace ports {

inline constexpr std::int8_t kIntake = 2;
inline constexpr std::int8_t kVision = 16; // temp
inline constexpr std::int8_t kImu = 10;

inline constexpr std::int8_t kLeftDriveA = 2;
inline constexpr std::int8_t kLeftDriveB = -19;
inline constexpr std::int8_t kRightDriveA = 15;
inline constexpr std::int8_t kRightDriveB = -18;

inline constexpr std::int8_t kElevatorA = -10;
inline constexpr std::int8_t kElevatorB = 8;

inline constexpr std::int8_t kPivot = 5;
inline constexpr std::int8_t kClaw = 3;

inline constexpr std::int8_t kVerticalOdom = 17;
inline constexpr std::int8_t kHorizontalOdom = 21;

inline constexpr char kIntakeLift = 'A';
inline constexpr char kPullToggle = 'B';

}
