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
// The donut ends within this of its heading, or once it passes it (turns keep LemLib's exits).
constexpr float kDonutExitDeg = 1.5f;
constexpr float kDonutMinSpin = 17.0f; // the PID's spin never drops below this, so the donut never stalls short

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

// Unit normal to the line through (line_x, line_y) along heading_deg, on the side of (side_x, side_y): the inside of a
// turn onto that line from there.
LemlibPoint inside_of(float line_x, float line_y, float heading_deg, float side_x, float side_y) {
    const LemlibPoint along = travel(heading_deg, 1.0f);
    const LemlibPoint normal{-along.y, along.x};
    if (normal.x * (side_x - line_x) + normal.y * (side_y - line_y) < 0.0f) {
        return {-normal.x, -normal.y};
    }
    return normal;
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

    // The goal this donut pivots on, from where the robot starts: goal distance behind it (in front when forwards).
    const lemlib::Pose start_pose = getPose();
    const float side = params.forwards ? -1.0f : 1.0f; // as in setPoseFromGoal: robot = goal + side * d * facing
    const float goal_distance =
        params.goalDistance > 0.0f ? params.goalDistance : (params.forwards ? kGoalResetFrontIn : kGoalResetBackIn);
    const auto goal_from = [side, goal_distance](const lemlib::Pose& pose) {
        const float h = lemlib::degToRad(pose.theta); // facing h in the field frame points along (cos h, -sin h)
        return lemlib::Pose(pose.x - side * goal_distance * std::cos(h), pose.y + side * goal_distance * std::sin(h));
    };
    const lemlib::Pose goal = goal_from(start_pose);

    const float start_heading = start_pose.theta;
    const lemlib::AngularDirection direction = lemlib::angleError(theta, start_heading, false, params.direction) >= 0
                                                   ? lemlib::AngularDirection::CW_CLOCKWISE
                                                   : lemlib::AngularDirection::CCW_COUNTERCLOCKWISE;

    const float travel = params.forwards ? 1.0f : -1.0f;
    const float push = travel * static_cast<float>(params.fastSpeed - params.slowSpeed) / 2.0f;
    const float max_spin = static_cast<float>(params.fastSpeed + params.slowSpeed) / 2.0f;

    angularPID.reset();

    const std::uint32_t start_time = pros::millis();
    std::optional<float> previous_raw_error;
    bool settling = false;
    distTraveled = 0;

    // Spin from the PID, capped at max_spin and never under kDonutMinSpin; the donut ends within kDonutExitDeg of
    // the heading or as soon as it passes it. The timeout is the cap when the heading never gets there.
    while (motionRunning && pros::millis() - start_time < static_cast<std::uint32_t>(timeout)) {
        const float heading = getPose().theta;
        distTraveled = std::fabs(heading - start_heading);

        const float raw_error = lemlib::angleError(theta, heading, false);
        if (previous_raw_error && std::fabs(raw_error) < kSettleWindowDeg &&
            (raw_error > 0) != (*previous_raw_error > 0)) {
            settling = true;
        }
        previous_raw_error = raw_error;

        const float error = settling ? raw_error : lemlib::angleError(theta, heading, false, direction);
        if (settling || std::fabs(error) <= std::max(kDonutExitDeg, params.earlyExitRange)) {
            break;
        }

        const float spin_size = std::clamp(std::fabs(angularPID.update(error)), kDonutMinSpin, max_spin);
        const float spin = error > 0 ? spin_size : -spin_size;
        drivetrain.leftMotors->move(static_cast<std::int32_t>(std::lround(push + spin)));
        drivetrain.rightMotors->move(static_cast<std::int32_t>(std::lround(push - spin)));
        pros::delay(kPeriodMs);
    }

    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);

    if (params.reanchor) {
        // Back onto the goal it started on, if it stayed on it: ended facing about where it should (the IMU), and
        // odometry still puts the goal about where it was.
        const lemlib::Pose end_pose = getPose();
        const lemlib::Pose goal_now = goal_from(end_pose);
        const bool on_heading = std::fabs(lemlib::angleError(theta, end_pose.theta, false)) <= params.reanchorMaxDeg;
        const bool on_goal = std::hypot(goal_now.x - goal.x, goal_now.y - goal.y) <= params.reanchorMaxDriftIn;
        if (on_heading && on_goal) {
            const float h = lemlib::degToRad(end_pose.theta);
            setPose(goal.x + side * goal_distance * std::cos(h), goal.y - side * goal_distance * std::sin(h),
                    end_pose.theta);
        }
    }

    distTraveled = -1;
    endMotion();
}

namespace {

// curve()'s two carrots for the path from start to (x, y).
struct CurveCarrots {
    LemlibPoint exit;
    LemlibPoint entry;
};

CurveCarrots curve_carrots(LemlibPoint start, float x, float y, float exitHeading, float entryHeading,
                           float exitLength, float entryLength, const CurveParams& params) {
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
    return {exit, {x - entryDir.x * entryLength, y - entryDir.y * entryLength}};
}

}

void Chassis::curve(float x, float y, float exitHeading, float entryHeading, float exitLength, float entryLength,
                    int timeout, CurveParams params) {
    // The exit point hangs off where the robot actually stops, not where an async motion still has it. Only wait while
    // one is running: LemLib's waitUntilDone() never returns before the first motion of the program.
    if (isInMotion()) {
        waitUntilDone();
    }
    const lemlib::Pose start = getPose();
    const CurveCarrots carrots =
        curve_carrots({start.x, start.y}, x, y, exitHeading, entryHeading, exitLength, entryLength, params);
    // lead moves the entry carrot off the approach line toward the inside of the turn.
    const LemlibPoint inside = inside_of(x, y, entryHeading, start.x, start.y);
    const LemlibPoint entry{carrots.entry.x + inside.x * params.lead, carrots.entry.y + inside.y * params.lead};

    moveToPoint(carrots.exit.x, carrots.exit.y, 800,
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

void Chassis::curveReverse(float fromX, float fromY, float x, float y, float exitHeading, float entryHeading,
                           float exitLength, float entryLength, const CurveParams& path, int timeout,
                           CurveParams drive) {
    if (isInMotion()) {
        waitUntilDone();
    }
    const CurveCarrots carrots =
        curve_carrots({fromX, fromY}, x, y, exitHeading, entryHeading, exitLength, entryLength, path);
    const bool forwards = !path.forwards;

    // lead moves the last carrot off the approach line (through (fromX, fromY) along exitHeading) toward the side the
    // path comes from, the inside of the turn.
    const LemlibPoint inside = inside_of(fromX, fromY, exitHeading, x, y);
    const LemlibPoint last{carrots.exit.x + inside.x * drive.lead, carrots.exit.y + inside.y * drive.lead};

    moveToPoint(carrots.entry.x, carrots.entry.y, 800,
                {.forwards = forwards, .maxSpeed = static_cast<float>(drive.maxSpeed),
                 .minSpeed = static_cast<float>(drive.minSpeed), .earlyExitRange = 2.0f});
    moveToPoint(last.x, last.y, timeout,
                {.forwards = forwards, .maxSpeed = static_cast<float>(drive.maxSpeed),
                 .minSpeed = static_cast<float>(drive.minSpeed), .earlyExitRange = 2.0f});
    // The forward curve leaves (fromX, fromY) at exitHeading, so end there squared up rather than at the angle of the
    // last leg.
    moveToPose(fromX, fromY, exitHeading, drive.finalTimeout,
               {.forwards = forwards, .maxSpeed = static_cast<float>(drive.finalMaxSpeed)});
    if (drive.deeper > 0.0f) {
        // Then straight on in along exitHeading.
        const LemlibPoint in = travel(exitHeading, forwards ? 1.0f : -1.0f);
        moveToPoint(fromX + in.x * drive.deeper, fromY + in.y * drive.deeper, 500,
                    {.forwards = forwards, .maxSpeed = static_cast<float>(drive.finalMaxSpeed)});
    }
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
