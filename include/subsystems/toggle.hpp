#pragma once

namespace subsystems::toggle {

inline constexpr int kInPower = 100;                // positive pulls the piece in. If the wheels spit it out, negate ports::kToggle.

void init();
void update();
void stop();
void spin(int power);
}
