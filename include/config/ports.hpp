#pragma once

#include <cstdint>

namespace ports {

// Drive and odometry ports are the new robot's. Everything marked placeholder is a guess.
inline constexpr std::int8_t kVision = 5; // AI Vision camera, from the Crimson branch (tested on the robot)
inline constexpr std::int8_t kImu = 11;

inline constexpr std::int8_t kLeftDriveA = -17;
inline constexpr std::int8_t kLeftDriveB = -18;
inline constexpr std::int8_t kRightDriveA = 19;
inline constexpr std::int8_t kRightDriveB = 20;

inline constexpr std::int8_t kVerticalOdom = 12;
inline constexpr std::int8_t kHorizontalOdom = -16;

inline constexpr std::int8_t kExtensionA = 10;  // from newbot-mechanism_tests (string lift)
inline constexpr std::int8_t kExtensionB = -9;  // from newbot-mechanism_tests

inline constexpr std::int8_t kArmPivot = 7;

inline constexpr std::int8_t kClawDistance = 12; // placeholder

// Side toggle, one 5.5W motor per side, spun together.
inline constexpr std::int8_t kSideToggleLeft = 2;
inline constexpr std::int8_t kSideToggleRight = -3;

inline constexpr char kClawPiston = 'A';

}
