#pragma once

#include <cstdint>

namespace util {

inline constexpr std::uint32_t kLoopMs = 20;

bool driver_control_active();

void start_driver_task(const char* name, void (*update)(), void (*stop)() = nullptr);

}
