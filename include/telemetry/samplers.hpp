#pragma once

#include <cstdint>

namespace telemetry {

std::uint32_t loop_overruns();

}

namespace telemetry::drive_log {

void fast(std::uint64_t timestamp);
void slow(std::uint64_t timestamp);

}

namespace telemetry::mechanism_log {

void fast(std::uint64_t timestamp);
void slow(std::uint64_t timestamp);

}

namespace telemetry::system_log {

void every_tick(std::uint64_t timestamp);
void fast(std::uint64_t timestamp);
void slow(std::uint64_t timestamp);

}

namespace telemetry::vision_log {

void fast(std::uint64_t timestamp);

}
