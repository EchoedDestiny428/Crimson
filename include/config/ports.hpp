#pragma once

#include <cstdint>

namespace ports {

// Drive and odometry ports are the new robot's. Everything marked placeholder is a guess.
inline constexpr std::int8_t kVision = 16; // temp
inline constexpr std::int8_t kImu = 3;

inline constexpr std::int8_t kLeftDriveA = -17; // directions from newbot-localization
inline constexpr std::int8_t kLeftDriveB = -18;
inline constexpr std::int8_t kRightDriveA = 19;
inline constexpr std::int8_t kRightDriveB = 20;

inline constexpr std::int8_t kVerticalOdom = 4;
inline constexpr std::int8_t kHorizontalOdom = -11;

inline constexpr std::int8_t kExtensionA = 10;  // from newbot-mechanism_tests (string lift)
inline constexpr std::int8_t kExtensionB = -9;  // from newbot-mechanism_tests

inline constexpr std::int8_t kArmPivot = 7;     // from newbot-mechanism_tests (red cartridge)

inline constexpr std::int8_t kClawDistance = 12; // placeholder: 4 is now the vertical wheel

inline constexpr std::int8_t kSideToggle = 6;   // side toggle (flex wheels), from newbot-mechanism_tests; not wired yet

inline constexpr char kClawPiston = 'A';

}
