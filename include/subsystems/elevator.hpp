#pragma once

#include <cstdint>

namespace subsystems::elevator {

inline constexpr double kDownDeg = 0.0;             // boot pose. init() zeros the encoder, so start with the carriage down.
inline constexpr double kMaxDeg = 400.0;            // unmeasured software stop. Jog with L1, read height(), and replace this. Lower it if the chain stalls first.
inline constexpr int kManualPower = 70;             // L1 is positive. Positive is assumed to raise the carriage. If L1 lowers it, negate kElevatorA and kElevatorB.
inline constexpr std::int32_t kVelocity = 80;       // slow on purpose until kMaxDeg is a measured travel.

void init();
void update();
void stop();
void set_target(double motor_deg);                  // chain height only. The bar, not this, sets the claw's height on the stack.
void hold();
void set_pos(double motor_deg);
double height();
bool is_settled();
bool wait_until_settled(std::uint32_t timeout_ms);

}
