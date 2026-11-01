#pragma once

#include <cstdint>

namespace subsystems::pivot {

inline constexpr double kCarryDeg = 0.0;            // boot pose: bar raised, claw clear of the stack. init() zeros the encoder there.
inline constexpr double kHighDeg = 40.0;            // placeholder. Up is positive, assumed to swing the claw down. If Up swings away, negate ports::kPivot, then replace these with get_pos().
inline constexpr double kMidDeg = 80.0;             // placeholder, same sign as kHighDeg.
inline constexpr double kLowDeg = 120.0;            // placeholder, same sign as kHighDeg.
inline constexpr double kPickupDeg = 160.0;         // placeholder, same sign as kHighDeg.
inline constexpr double kMinDeg = -20.0;            // soft stop around the placeholders, not a measured end of travel.
inline constexpr double kMaxDeg = 220.0;            // soft stop past the presets so the first press cannot hit the tower.

inline constexpr int kManualPower = 50;             // kept low until the angles above are measured.
inline constexpr std::int32_t kVelocity = 60;       // kept low until the angles above are measured.

void init();
void update();
void move_to(double motor_deg);                     // one motor, chain transmission to the bar.
void stop();
void set_pos(double motor_deg);
double get_pos();
bool is_settled();
bool wait_until_settled(std::uint32_t timeout_ms);

}
