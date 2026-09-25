#pragma once

namespace subsystems::pivot {

inline constexpr double kHomeDeg = 0.0;
inline constexpr double kFlippedDeg = 90.0;

void init();
void update();
void stop();
void move_to(double arm_deg);

}
