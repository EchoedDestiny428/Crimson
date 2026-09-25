#pragma once

namespace subsystems::pivot {

inline constexpr double kHomeMotorDeg = 0.0;
inline constexpr double kFlippedMotorDeg = 1000.0;

void init();
void update();
void stop();
void move_to(double motor_deg);

}
