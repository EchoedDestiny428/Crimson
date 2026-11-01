#pragma once

namespace subsystems::imu_peaks {

// Controller readout for comparing odom with and without the tilt guard:
//   line 1: Zach's odom, as if the guard never held (x, y, heading)
//   line 2: our guarded odom (x, y) and how many times the guard has triggered
//   line 3: most pitch and roll, degrees from where the robot sat at startup
// A resets the tilt peaks and the guard count. Start after the IMU has calibrated,
// with the robot flat.
void start();

}
