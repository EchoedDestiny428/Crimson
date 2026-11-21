#pragma once

#include <cstdint>

namespace ports {

// Drive and odometry ports are the new robot's. Everything marked placeholder is a guess.
inline constexpr std::int8_t kVision = 5; // AI Vision camera, from the Crimson branch (tested on the robot)
inline constexpr std::int8_t kImu = 11;

inline constexpr std::int8_t kLeftDriveA = -17; // directions from newbot-localization
inline constexpr std::int8_t kLeftDriveB = -18;
inline constexpr std::int8_t kRightDriveA = 19;
inline constexpr std::int8_t kRightDriveB = 20;

inline constexpr std::int8_t kVerticalOdom = 12;
inline constexpr std::int8_t kHorizontalOdom = -16;

inline constexpr std::int8_t kExtensionA = 10;  // from newbot-mechanism_tests (string lift)
inline constexpr std::int8_t kExtensionB = -9;  // from newbot-mechanism_tests

inline constexpr std::int8_t kArmPivot = 7;     // from newbot-mechanism_tests (red cartridge)

inline constexpr std::int8_t kClawDistance = 12; // placeholder: 4 is now the vertical wheel

// Side toggle, one 5.5W motor per side, spun together. Negate one to reverse which way Y spins it.
inline constexpr std::int8_t kSideToggleLeft = 2;
inline constexpr std::int8_t kSideToggleRight = -3; // placeholder: unused port until the right motor is wired

inline constexpr char kClawPiston = 'A';

}
