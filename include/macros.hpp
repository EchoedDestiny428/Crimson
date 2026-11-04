#pragma once

namespace macros {

void update();
void stop();
void pickup();                                      // bar to the pickup angle, close, bar back to carry. Does not move the string.
void score();                                       // bar to the angle selected with Right, open, bar back to carry. Does not move the string.
void stow();                                        // bar back to carry, then string to constants::kExtensionHomeCm.
void cancel();                                      // L1, L2, the d-pad, R1, or R2 take the motor. A stays the IMU peak reset.

}
