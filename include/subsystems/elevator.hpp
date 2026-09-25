#pragma once

#include <cstdint>

namespace subsystems::elevator {

inline constexpr double kBootup = 0.0;
inline constexpr double kHome = 150.0;
inline constexpr double kFlipOut = 500.0;
inline constexpr double kStage1 = 750.0;
inline constexpr double kStage2 = 1000.0;
inline constexpr double kStage3 = 1500.0;
inline constexpr double kStage4 = 2500.0;
inline constexpr double kStage5 = 3500.0;
inline constexpr double kStage6 = 4500.0;
inline constexpr double kMax = 4550.0;


void init();
void hold();
void set_target(double height);
void set_manual(int power);
void snap_to_stage();
double height();
bool wait_until_settled(std::uint32_t timeout_ms);

}
