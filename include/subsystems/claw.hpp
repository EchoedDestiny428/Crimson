#pragma once

#include <cstdint>

namespace subsystems::claw {

inline constexpr bool kClosedLevel = true;          // voltage sent to the piston when closed. Flip this if HIGH opens the claw.
inline constexpr std::uint32_t kTravelMs = 200;     // no piston sensor. Settled means this wait finished. Raise it if the claw is still moving.

void init();
void update();
void stop();                                        // does not move the piston, so leaving driver control does not drop the cone.
void close();
void open();
bool is_closed();
bool is_settled();
bool wait_until_settled(std::uint32_t timeout_ms);

}
