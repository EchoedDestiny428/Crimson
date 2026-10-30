#pragma once

#include <cstdint>

namespace subsystems::elevator::extension {

// Heights in motor degrees. Measure these on the new robot.
inline constexpr double kHome = 0.0;
inline constexpr double kMax = 3500.0;

void init();
void update();
void stop();
void hold();
void set_pos(double height);
void set_target(double height);
void set_manual(int power);
double height();
double target();
bool is_settled();
bool wait_until_settled(std::uint32_t timeout_ms);

}
