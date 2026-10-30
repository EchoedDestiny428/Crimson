#pragma once

#include <cstdint>

namespace subsystems::elevator::pivot {

// Angles in motor degrees. Measure these on the new robot.
inline constexpr double kHomeMotorDeg = 0.0;

void init();
void update();
void stop();
void set_pos(double motor_deg);
void move_to(double motor_deg);
bool is_settled();
bool wait_until_settled(std::uint32_t timeout_ms);
double get_pos();

}
