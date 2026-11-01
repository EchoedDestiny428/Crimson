#pragma once

namespace macros {

void update();
void stop();
void pickup();                                      // bar to the pickup angle, close, bar back to carry. Does not move the chain.
void score();                                       // bar to the angle selected with Right, open, bar back to carry. Does not move the chain.
void stow();                                        // bar back to carry, then chain to elevator::kDownDeg.
void cancel();                                      // L1, L2, the d-pad, R1, or R2 take the motor. A stays the IMU peak reset.

}
