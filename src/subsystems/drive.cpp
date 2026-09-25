#include "subsystems/drive.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include <cmath>

namespace subsystems::drive {

namespace {

constexpr float kThrottleSlew = 40.0f;
constexpr double kForwardPitchSign = 1.0;
constexpr double kTipEnterDeg = 12.0;
constexpr double kTipExitDeg = 6.0;
constexpr int kAntiTipPower = -80;

float slewed_throttle = 0.0f;
bool tip_correcting = false;

bool correcting_forward_tip() {
    const double pitch = imu.get_pitch();
    if (!std::isfinite(pitch)) {
        return tip_correcting;
    }

    const double forward_lean = kForwardPitchSign * pitch;
    if (forward_lean >= kTipEnterDeg) {
        tip_correcting = true;
    } else if (forward_lean <= kTipExitDeg) {
        tip_correcting = false;
    }
    return tip_correcting;
}

}

void init() {
    chassis.calibrate();
}

void reset() {
    slewed_throttle = 0.0f;
    tip_correcting = false;
}

void update() {
    if (correcting_forward_tip()) {
        slewed_throttle = kAntiTipPower;
        chassis.arcade(kAntiTipPower, 0, true);
        return;
    }

    const int throttle = controller.get_analog(controls::kThrottle);
    const int turn = controller.get_analog(controls::kTurn);
    slewed_throttle = lemlib::slew(throttle, slewed_throttle, kThrottleSlew);
    chassis.arcade(static_cast<int>(slewed_throttle), turn);
}

}
