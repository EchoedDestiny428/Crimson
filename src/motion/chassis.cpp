#include "motion/chassis.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdint>

namespace crimson {

namespace {

constexpr std::uint32_t kPeriodMs = 10;
constexpr float kPassedTargetJumpDeg = 180.0f;

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
    const float initial_error = lemlib::angleError(theta, start_heading, false, params.direction);
    const bool clockwise = initial_error >= 0;
    const lemlib::AngularDirection direction =
        clockwise ? lemlib::AngularDirection::CW_CLOCKWISE : lemlib::AngularDirection::CCW_COUNTERCLOCKWISE;

    const int travel = params.forwards ? 1 : -1;
    const bool left_is_fast = (travel > 0) == clockwise;
    const int fast_power = travel * params.fastSpeed;
    const int slow_power = -travel * params.slowSpeed;
    const int left_power = left_is_fast ? fast_power : slow_power;
    const int right_power = left_is_fast ? slow_power : fast_power;

    const std::uint32_t start_time = pros::millis();
    float previous_error = std::fabs(initial_error);
    distTraveled = 0;

    while (motionRunning && pros::millis() - start_time < static_cast<std::uint32_t>(timeout)) {
        const float heading = getPose().theta;
        const float error = std::fabs(lemlib::angleError(theta, heading, false, direction));
        distTraveled = std::fabs(heading - start_heading);

        if (error <= params.earlyExitRange || error > previous_error + kPassedTargetJumpDeg) {
            break;
        }
        previous_error = error;

        drivetrain.leftMotors->move(left_power);
        drivetrain.rightMotors->move(right_power);
        pros::delay(kPeriodMs);
    }

    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    endMotion();
}

}
