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

// setPoseFromGoal's push: against the goal once odometry has moved less than kStoppedInPerTick a tick for kStoppedMs,
// after at least kPushMinMs of pushing (so a robot still rolling in isn't mistaken for stopped).
constexpr float kStoppedInPerTick = 0.01f; // 1 in/s
constexpr std::uint32_t kStoppedMs = 60;
constexpr std::uint32_t kPushMinMs = 80;

struct LemlibPoint {
    float x;
    float y;
};

// Field (x, y) to LemLib's frame. Field heading 0 points along field +X, which is LemLib's +Y.
LemlibPoint to_lemlib(float x, float y) {
    return {-y, x};
}

}

lemlib::Pose Chassis::getPose(bool radians, bool standardPos) {
    const lemlib::Pose lemlib_pose = lemlib::Chassis::getPose(radians);
    return {lemlib_pose.y, -lemlib_pose.x, standardPos ? -lemlib_pose.theta : lemlib_pose.theta};
}

void Chassis::setPose(float x, float y, float theta, bool radians) {
    pose_mutex_.take();
    const lemlib::Pose before = getPose(); // field frame, only to log the correction
    const LemlibPoint point = to_lemlib(x, y);
    lemlib::Chassis::setPose(point.x, point.y, theta, radians);
    reset_dx_ = x - before.x;
    reset_dy_ = y - before.y;
    nudge_x_ = 0.0f;
    nudge_y_ = 0.0f;
    ++pose_resets_;
    pose_mutex_.give();
}

void Chassis::nudge(float dx, float dy) {
    pose_mutex_.take();
    const lemlib::Pose now = lemlib::Chassis::getPose(); // LemLib frame
    const LemlibPoint shift = to_lemlib(dx, dy);           // field (x, y) -> LemLib (-y, x) works for a shift too
    lemlib::Chassis::setPose(now.x + shift.x, now.y + shift.y, now.theta);
    nudge_x_ = nudge_x_ + dx;
    nudge_y_ = nudge_y_ + dy;
    pose_mutex_.give();
}

float Chassis::nudgeX() const {
    return nudge_x_;
}

float Chassis::nudgeY() const {
    return nudge_y_;
}

Chassis::OdomSample Chassis::sample() {
    pose_mutex_.take();
    const OdomSample result{getPose(), nudge_x_, nudge_y_, pose_resets_};
    pose_mutex_.give();
    return result;
}

std::uint32_t Chassis::poseResets() const {
    return pose_resets_;
}

float Chassis::lastResetDx() const {
    return reset_dx_;
}

float Chassis::lastResetDy() const {
    return reset_dy_;
}

std::uint32_t Chassis::poseResetSkips() const {
    return reset_skips_;
}

void Chassis::setPose(lemlib::Pose pose, bool radians) {
    setPose(pose.x, pose.y, pose.theta, radians);
}

void Chassis::turnToPoint(float x, float y, int timeout, lemlib::TurnToPointParams params, bool async) {
    const LemlibPoint point = to_lemlib(x, y);
    lemlib::Chassis::turnToPoint(point.x, point.y, timeout, params, async);
}

void Chassis::swingToPoint(float x, float y, lemlib::DriveSide lockedSide, int timeout,
                           lemlib::SwingToPointParams params, bool async) {
    const LemlibPoint point = to_lemlib(x, y);
    lemlib::Chassis::swingToPoint(point.x, point.y, lockedSide, timeout, params, async);
}

void Chassis::moveToPose(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params, bool async) {
    const LemlibPoint point = to_lemlib(x, y);
    lemlib::Chassis::moveToPose(point.x, point.y, theta, timeout, params, async);
}

void Chassis::moveToPoint(float x, float y, int timeout, lemlib::MoveToPointParams params, bool async) {
    const LemlibPoint point = to_lemlib(x, y);
    lemlib::Chassis::moveToPoint(point.x, point.y, timeout, params, async);
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

bool Chassis::setPoseFromGoal(float goalX, float goalY, GoalPoseParams params) {
    if (params.pushPower > 0) {
        // Push toward the goal until odometry stops: then the robot is really against it.
        waitUntilDone();
        const int power = params.forwards ? params.pushPower : -params.pushPower;
        drivetrain.leftMotors->move(power);
        drivetrain.rightMotors->move(power);
        const std::uint32_t start = pros::millis();
        // Odometry without the camera feed's nudges: the feed may slide the pose while the robot sits against the
        // goal, which isn't the robot moving.
        const auto raw_pose = [this] {
            const OdomSample now = sample();
            return lemlib::Pose(now.pose.x - now.nudge_x, now.pose.y - now.nudge_y, now.pose.theta);
        };
        lemlib::Pose last = raw_pose();
        std::uint32_t stopped_since = 0;
        bool contact = false;
        while (pros::millis() - start < static_cast<std::uint32_t>(std::max(params.pushTimeout, 0))) {
            pros::delay(kPeriodMs);
            const std::uint32_t now_ms = pros::millis();
            const lemlib::Pose pose = raw_pose();
            const bool still = std::hypot(pose.x - last.x, pose.y - last.y) < kStoppedInPerTick;
            last = pose;
            if (!still || now_ms - start < kPushMinMs) {
                stopped_since = 0;
            } else if (stopped_since == 0) {
                stopped_since = now_ms;
            } else if (now_ms - stopped_since >= kStoppedMs) {
                contact = true;
                break;
            }
        }
        drivetrain.leftMotors->move(0);
        drivetrain.rightMotors->move(0);
        if (!contact) { // never stopped against anything: don't trust a reset
            reset_dx_ = 0.0f;
            reset_dy_ = 0.0f;
            ++reset_skips_;
            return false;
        }
    }
    const lemlib::Pose now = getPose();
    const float heading = lemlib::degToRad(now.theta);
    const float side = params.forwards ? -1.0f : 1.0f;
    const float distance =
        params.distance > 0.0f ? params.distance : (params.forwards ? kGoalResetFrontIn : kGoalResetBackIn);
    // Facing heading h in the field frame points along (cos h, -sin h).
    const float x = goalX + side * distance * std::cos(heading);
    const float y = goalY - side * distance * std::sin(heading);
    if (std::hypot(x - now.x, y - now.y) > params.maxCorrection) {
        reset_dx_ = 0.0f; // for the log: this reset was skipped
        reset_dy_ = 0.0f;
        ++reset_skips_;
        return false;
    }
    setPose(x, y, now.theta); // logs the correction
    return true;
}

bool Chassis::resetFromGoalFront(float goalX, float goalY) {
    return setPoseFromGoal(goalX, goalY, {.forwards = true, .maxCorrection = 1000});
}

bool Chassis::resetFromGoalBack(float goalX, float goalY) {
    return setPoseFromGoal(goalX, goalY, {.forwards = false, .maxCorrection = 1000});
}

}
