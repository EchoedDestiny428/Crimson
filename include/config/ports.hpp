#pragma once

#include <cstdint>

namespace ports {

inline constexpr std::int8_t kVision = 16; // placeholder
inline constexpr std::int8_t kImu = 3;

inline constexpr std::int8_t kLeftDriveA = -17;
inline constexpr std::int8_t kLeftDriveB = -18;
inline constexpr std::int8_t kRightDriveA = 19;
inline constexpr std::int8_t kRightDriveB = 20;

inline constexpr std::int8_t kVerticalOdom = 4;
inline constexpr std::int8_t kHorizontalOdom = -11;

inline constexpr std::int8_t kExtensionA = 10;
inline constexpr std::int8_t kExtensionB = -8;

inline constexpr std::int8_t kArmPivot = 7;

inline constexpr std::int8_t kClawDistance = 12; // placeholder

inline constexpr std::int8_t kSideToggle = 6; // placeholder

inline constexpr char kClawPiston = 'A';

}
