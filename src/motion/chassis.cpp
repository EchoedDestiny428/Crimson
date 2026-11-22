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

struct LemlibPoint {
    float x;
    float y;
};

// Field (x, y) to LemLib's frame. Field heading 0 points along field +X, which is LemLib's +Y.
LemlibPoint to_lemlib(float x, float y) {
    return {-y, x};
}

// Field-frame unit vectors for curve(), stored in the same {x, y} pair.
LemlibPoint travel(float heading_deg, float direction) {
    const float radians = lemlib::degToRad(heading_deg);
    return {direction * std::cos(radians), -direction * std::sin(radians)};
}

LemlibPoint toward(float from_x, float from_y, float to_x, float to_y) {
    const float length = std::hypot(to_x - from_x, to_y - from_y);
    if (length < 1e-3f) {
        return {0.0f, 0.0f};
    }
    return {(to_x - from_x) / length, (to_y - from_y) / length};
}

// Unit vector bias of the way from a to b; a when bias is 0 or b is zero.
LemlibPoint blend(LemlibPoint a, LemlibPoint b, float bias) {
    if (bias <= 0.0f || (b.x == 0.0f && b.y == 0.0f)) {
        return a;
    }
    const LemlibPoint mixed{(1.0f - bias) * a.x + bias * b.x, (1.0f - bias) * a.y + bias * b.y};
    const float length = std::hypot(mixed.x, mixed.y);
    return length < 1e-3f ? a : LemlibPoint{mixed.x / length, mixed.y / length};
}

}

lemlib::Pose Chassis::getPose(bool radians, bool standardPos) {
    const lemlib::Pose lemlib_pose = lemlib::Chassis::getPose(radians);
    return {lemlib_pose.y, -lemlib_pose.x, standardPos ? -lemlib_pose.theta : lemlib_pose.theta};
}

void Chassis::setPose(float x, float y, float theta, bool radians) {
    const LemlibPoint point = to_lemlib(x, y);
    lemlib::Chassis::setPose(point.x, point.y, theta, radians);
    ++pose_resets_;
}

std::uint32_t Chassis::poseResets() const {
    return pose_resets_;
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

void Chassis::curve(float x, float y, float exitHeading, float entryHeading, float exitLength, float entryLength,
                    int timeout, CurveParams params) {
    // The exit point hangs off where the robot actually stops, not where an async motion still has it.
    waitUntilDone();
    const lemlib::Pose start = getPose();
    const float direction = params.forwards ? 1.0f : -1.0f;

    // Travel directions at the exit and entry points. Facing heading h in the field frame points along (cos h, -sin h).
    const LemlibPoint exitTravel = travel(exitHeading, direction);
    const LemlibPoint entryTravel = travel(entryHeading, direction);

    // Each bias turns its carrot point that fraction of the way toward the other point, so the robot starts
    // curving on the first leg instead of driving straight out and swinging wide.
    const LemlibPoint plainEntry{x - entryTravel.x * entryLength, y - entryTravel.y * entryLength};
    const LemlibPoint exitDir =
        blend(exitTravel, toward(start.x, start.y, plainEntry.x, plainEntry.y), params.exitBias);
    const LemlibPoint exit{start.x + exitDir.x * (exitLength + 2.0f), start.y + exitDir.y * (exitLength + 2.0f)};
    const LemlibPoint entryDir = blend(entryTravel, toward(exit.x, exit.y, x, y), params.entryBias);
    const LemlibPoint entry{x - entryDir.x * entryLength, y - entryDir.y * entryLength};

    moveToPoint(exit.x, exit.y, 800,
                {.forwards = params.forwards, .maxSpeed = static_cast<float>(params.exitMaxSpeed),
                 .minSpeed = static_cast<float>(params.minSpeed), .earlyExitRange = 2.0f});
    moveToPoint(entry.x, entry.y, timeout,
                {.forwards = params.forwards, .maxSpeed = static_cast<float>(params.maxSpeed),
                 .minSpeed = static_cast<float>(params.minSpeed), .earlyExitRange = 6.0f});
    turnToPoint(x, y, 600,
                {.forwards = params.forwards, .maxSpeed = params.maxSpeed, .minSpeed = params.minSpeed,
                 .earlyExitRange = 2.0f});
    moveToPoint(x, y, params.finalTimeout,
                {.forwards = params.forwards, .maxSpeed = static_cast<float>(params.finalMaxSpeed)});
}

bool Chassis::setPoseFromGoal(float goalX, float goalY, GoalPoseParams params) {
    const lemlib::Pose now = getPose();
    const float heading = lemlib::degToRad(now.theta);
    const float side = params.forwards ? -1.0f : 1.0f;
    const float distance =
        params.distance > 0.0f ? params.distance : (params.forwards ? kGoalResetFrontIn : kGoalResetBackIn);
    // Facing heading h in the field frame points along (cos h, -sin h).
    const float x = goalX + side * distance * std::cos(heading);
    const float y = goalY - side * distance * std::sin(heading);
    if (std::hypot(x - now.x, y - now.y) > params.maxCorrection) {
        return false;
    }
    setPose(x, y, now.theta);
    return true;
}

bool Chassis::resetFromGoalFront(float goalX, float goalY) {
    return setPoseFromGoal(goalX, goalY, {.forwards = true, .maxCorrection = 1000});
}

bool Chassis::resetFromGoalBack(float goalX, float goalY) {
    return setPoseFromGoal(goalX, goalY, {.forwards = false, .maxCorrection = 1000});
}

}
