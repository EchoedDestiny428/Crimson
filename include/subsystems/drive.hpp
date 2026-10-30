#pragma once

#include <cstdint>

namespace subsystems::drive {

inline constexpr std::uint32_t kLoopMs = 20;

void init();
void reset();
void update();

}
