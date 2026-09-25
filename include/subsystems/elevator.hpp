#pragma once

#include <cstdint>

namespace subsystems::elevator {

inline constexpr double kBootup = 0.0;
inline constexpr double kHome = 150.0;
inline constexpr double kFlipOut = 500.0;
inline constexpr double kStage1 = 600.0;
inline constexpr double kStage2 = 800.0;
inline constexpr double kStage3 = 1000.0;
inline constexpr double kStage4 = 1200.0;
inline constexpr double kStage5 = 1400.0;
inline constexpr double kStage6 = 1600.0;

void init();
void hold();
void set_target(double height);
void set_manual(int power);
void snap_to_stage();
double height();
bool wait_until_settled(std::uint32_t timeout_ms);

}
