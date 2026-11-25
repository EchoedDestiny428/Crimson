#pragma once

namespace display::motor_temps {

// V5 motors halve their current limit at 55 C and keep cutting it as they heat further.
inline constexpr double kWarnC = 55.0;

// Controller line `line`: while any motor is at or over kWarnC, show the hottest one ("HOT DL2 60C")
// instead of the line's usual text, and rumble once when it first gets hot. True if it used the line,
// so the caller skips its own print this time.
bool print_warning(int line);

}
