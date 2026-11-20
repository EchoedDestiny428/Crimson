#pragma once

// Right side toggle: a 5.5W motor that just spins (it doesn't take a piece); brakes when released.
namespace subsystems::side_toggle {

inline constexpr int kForwardPower = 100; // negate ports::kSideToggleRight to flip which way forward is
inline constexpr int kReversePower = -100;

void init();
void update(); // L2 held spins forward, Y held spins reverse, release brakes
void stop();
void spin(int power);

}
