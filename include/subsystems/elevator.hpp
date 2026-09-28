#pragma once

#include <cstdint>

namespace subsystems::elevator {

inline constexpr double kBootup = 0.0;
inline constexpr double kFlipOutMin = 10.0;
inline constexpr double kHome = 150.0;
inline constexpr double kDualSetup = 20.0;
inline constexpr double kOffset = 140.0;
inline constexpr double kFlipOut = 100.0;
inline constexpr double kStage1 = 650.0;
inline constexpr double kStage2 = 1200.0;
inline constexpr double kStage3 = 1750.0;
inline constexpr double kStage4 = 2450.0;
inline constexpr double kStage5 = 2850.0;
inline constexpr double kMax = 3500.0;

void init();
void set_pos(double height);
void hold();
void set_target(double height);
void set_target_below_home(double height);
void set_manual(int power);
double snap_to_stage(bool offset = false);
double top_stage();
double height();
double target();
bool has_left_bootup();
bool is_settled();
bool wait_until_settled(std::uint32_t timeout_ms);

}
