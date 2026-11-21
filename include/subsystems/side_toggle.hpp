#pragma once

// Side toggle: a 5.5W motor on each side that just spin together (they don't take a piece);
// brake when released.
namespace subsystems::side_toggle {

inline constexpr int kForwardPower = 100; // negate a ports::kSideToggle* port to flip that side
inline constexpr int kReversePower = -100;

void init();
void update(); // L2 held spins forward, Y held spins reverse, release brakes
void stop();
void spin(int power);

}
