#include "motion/chassis.hpp"
#include "pros/rtos.hpp"
#include "telemetry/telemetry.hpp"
#include "util/angle.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace crimson {

namespace {

using telemetry::structs::Motion;
using telemetry::structs::MotionKind;

constexpr std::uint32_t kPeriodMs = 10;
constexpr float kSettleWindowDeg = 90.0f;
constexpr double kUnused = std::numeric_limits<double>::quiet_NaN();

template <typename Start>
void run_recorded(const Motion& motion, const std::source_location& where, Start start) {
    if (!motion.async) {
        telemetry::motion_started(motion, where);
    }
    start();
    if (motion.async) {
        telemetry::motion_started(motion, where);
    }
}

}

template <typename Apply>
void Chassis::resetPose(Apply apply, const std::source_location& where) {
    rawMutex.take();
    const lemlib::Pose before = getPose();
    apply();
    const lemlib::Pose after = getPose();
    rebaseRaw(before, after);
    rawMutex.give();
    telemetry::pose_reset(before, after, where);
}

void Chassis::setPose(float x, float y, float theta, bool radians, std::source_location where) {
    resetPose([&] { lemlib::Chassis::setPose(x, y, theta, radians); }, where);
}

void Chassis::setPose(lemlib::Pose pose, bool radians, std::source_location where) {
    resetPose([&] { lemlib::Chassis::setPose(pose, radians); }, where);
}

void Chassis::turnToPoint(float x, float y, int timeout, lemlib::TurnToPointParams params, bool async,
                          std::source_location where) {
    run_recorded({MotionKind::TurnToPoint, x, y, kUnused, timeout, static_cast<float>(params.maxSpeed),
                  static_cast<float>(params.minSpeed), params.earlyExitRange, params.forwards, async},
                 where, [&] { lemlib::Chassis::turnToPoint(x, y, timeout, params, async); });
}

void Chassis::turnToHeading(float theta, int timeout, lemlib::TurnToHeadingParams params, bool async,
                            std::source_location where) {
    run_recorded({MotionKind::TurnToHeading, kUnused, kUnused, theta, timeout, static_cast<float>(params.maxSpeed),
                  static_cast<float>(params.minSpeed), params.earlyExitRange, false, async},
                 where, [&] { lemlib::Chassis::turnToHeading(theta, timeout, params, async); });
}

void Chassis::swingToHeading(float theta, lemlib::DriveSide lockedSide, int timeout,
                             lemlib::SwingToHeadingParams params, bool async, std::source_location where) {
    run_recorded({MotionKind::SwingToHeading, kUnused, kUnused, theta, timeout, params.maxSpeed, params.minSpeed,
                  params.earlyExitRange, false, async},
                 where, [&] { lemlib::Chassis::swingToHeading(theta, lockedSide, timeout, params, async); });
}

void Chassis::swingToPoint(float x, float y, lemlib::DriveSide lockedSide, int timeout,
                           lemlib::SwingToPointParams params, bool async, std::source_location where) {
    run_recorded({MotionKind::SwingToPoint, x, y, kUnused, timeout, params.maxSpeed, params.minSpeed,
                  params.earlyExitRange, params.forwards, async},
                 where, [&] { lemlib::Chassis::swingToPoint(x, y, lockedSide, timeout, params, async); });
}

void Chassis::moveToPose(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params, bool async,
                         std::source_location where) {
    run_recorded({MotionKind::MoveToPose, x, y, theta, timeout, params.maxSpeed, params.minSpeed, params.earlyExitRange,
                  params.forwards, async},
                 where, [&] { lemlib::Chassis::moveToPose(x, y, theta, timeout, params, async); });
}

void Chassis::moveToPoint(float x, float y, int timeout, lemlib::MoveToPointParams params, bool async,
                          std::source_location where) {
    run_recorded({MotionKind::MoveToPoint, x, y, kUnused, timeout, params.maxSpeed, params.minSpeed,
                  params.earlyExitRange, params.forwards, async},
                 where, [&] { lemlib::Chassis::moveToPoint(x, y, timeout, params, async); });
}

void Chassis::donut(float theta, int timeout, DonutParams params, bool async, std::source_location where) {
    run_recorded({MotionKind::Donut, kUnused, kUnused, theta, timeout, static_cast<float>(params.fastSpeed),
                  static_cast<float>(params.slowSpeed), params.earlyExitRange, params.forwards, async},
                 where, [&] { runDonut(theta, timeout, params, async); });
}

void Chassis::runDonut(float theta, int timeout, DonutParams params, bool async) {
    requestMotionStart();
    if (!motionRunning) {
        return;
    }

    if (async) {
        pros::Task task([this, theta, timeout, params] { runDonut(theta, timeout, params, false); });
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

bool Chassis::setPoseFromGoal(float goalX, float goalY, GoalPoseParams params, std::source_location where) {
    const lemlib::Pose now = getPose();
    const float heading = lemlib::degToRad(now.theta);
    const float side = params.forwards ? -1.0f : 1.0f;
    const float x = goalX + side * params.distance * std::sin(heading);
    const float y = goalY + side * params.distance * std::cos(heading);
    if (std::hypot(x - now.x, y - now.y) > params.maxCorrection) {
        return false;
    }
    setPose(x, y, now.theta, false, where);
    return true;
}

void Chassis::armRawAnchor() {
    rawMutex.take();
    rawArmed = true;
    rawMutex.give();
}

lemlib::Pose Chassis::getRawPose() {
    rawMutex.take();
    const lemlib::Pose pose = applyRaw(getPose());
    rawMutex.give();
    return pose;
}

lemlib::Pose Chassis::applyRaw(const lemlib::Pose& pose) const {
    const double phi = util::deg_to_rad(raw.theta);
    const double cos_phi = std::cos(phi);
    const double sin_phi = std::sin(phi);
    return {static_cast<float>(pose.x * cos_phi + pose.y * sin_phi + raw.x),
            static_cast<float>(-pose.x * sin_phi + pose.y * cos_phi + raw.y),
            static_cast<float>(pose.theta + raw.theta)};
}

void Chassis::rebaseRaw(const lemlib::Pose& before, const lemlib::Pose& after) {
    if (rawArmed) {
        rawArmed = false;
        raw = {};
        return;
    }

    const double old_phi = util::deg_to_rad(raw.theta);
    const double raw_x = before.x * std::cos(old_phi) + before.y * std::sin(old_phi) + raw.x;
    const double raw_y = -before.x * std::sin(old_phi) + before.y * std::cos(old_phi) + raw.y;

    raw.theta += static_cast<double>(before.theta) - static_cast<double>(after.theta);
    const double phi = util::deg_to_rad(raw.theta);
    raw.x = raw_x - (after.x * std::cos(phi) + after.y * std::sin(phi));
    raw.y = raw_y - (-after.x * std::sin(phi) + after.y * std::cos(phi));
}

}
