#pragma once

// Side toggle: the flex wheels that pull a piece in (from Roy's newbot-mechanism_tests).
namespace subsystems::side_toggle {

inline constexpr int kInPower = 100; // positive pulls the piece in; negate ports::kSideToggle if it spits it out
inline constexpr int kOutPower = -100;

void init();
void update(); // R1 held spins in, R2 held spins out, release coasts
void stop();
void spin(int power);

}
