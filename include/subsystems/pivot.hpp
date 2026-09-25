#pragma once

#include <cstdint>

namespace subsystems::pivot {

inline constexpr double kHomeMotorDeg = 0.0;
inline constexpr double kFlippedMotorDeg = 950;
inline constexpr double kTopMotorDeg = 1100.0;
inline constexpr double kDualSetupDeg = 1200.0;
inline constexpr double kDualPickupDeg = 350.0;

void init();
void update();
void stop();
void move_to(double motor_deg);
bool is_settled();
bool wait_until_settled(std::uint32_t timeout_ms);

}
