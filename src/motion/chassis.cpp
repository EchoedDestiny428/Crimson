#include "motion/chassis.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

namespace crimson {

namespace {

constexpr std::uint32_t kPeriodMs = 10;
constexpr float kSettleWindowDeg = 90.0f;

}

void Chassis::donut(float theta, int timeout, DonutParams params, bool async) {
    requestMotionStart();
    if (!motionRunning) {
        return;
    }

    if (async) {
        pros::Task task([this, theta, timeout, params] { donut(theta, timeout, params, false); });
        endMotion();
        pros::delay(kPeriodMs);
        return;
    }

    const float start_heading = getPose().theta;
    const lemlib::AngularDirection direction = lemlib::angleError(theta, start_heading, false, params.direction) >= 0
                                                   ? lemlib::AngularDirection::CW_CLOCKWISE
                                                   : lemlib::AngularDirection::CCW_COUNTERCLOCKWISE;

    const float travel = params.forwards ? 1.0f : -1.0f;
    const float push = travel * static_cast<float>(params.fastSpeed - params.slowSpeed) / 2.0f;
    const float max_spin = static_cast<float>(params.fastSpeed + params.slowSpeed) / 2.0f;

    angularPID.reset();
    angularLargeExit.reset();
    angularSmallExit.reset();

    const std::uint32_t start_time = pros::millis();
    std::optional<float> previous_raw_error;
    bool settling = false;
    distTraveled = 0;

    while (motionRunning && pros::millis() - start_time < static_cast<std::uint32_t>(timeout) &&
           !angularLargeExit.getExit() && !angularSmallExit.getExit()) {
        const float heading = getPose().theta;
        distTraveled = std::fabs(heading - start_heading);

        const float raw_error = lemlib::angleError(theta, heading, false);
        if (previous_raw_error && std::fabs(raw_error) < kSettleWindowDeg &&
            (raw_error > 0) != (*previous_raw_error > 0)) {
            settling = true;
        }
        previous_raw_error = raw_error;

        const float error = settling ? raw_error : lemlib::angleError(theta, heading, false, direction);
        angularLargeExit.update(error);
        angularSmallExit.update(error);
        if (params.earlyExitRange > 0 && std::fabs(error) < params.earlyExitRange) {
            break;
        }

        const float spin = std::clamp(angularPID.update(error), -max_spin, max_spin);
        drivetrain.leftMotors->move(static_cast<std::int32_t>(std::lround(push + spin)));
        drivetrain.rightMotors->move(static_cast<std::int32_t>(std::lround(push - spin)));
        pros::delay(kPeriodMs);
    }

    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    endMotion();
}

}
