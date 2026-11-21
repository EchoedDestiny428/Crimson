#include "chaos/chaos.hpp"
#include "pros/rtos.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator/claw.hpp"
#include "subsystems/elevator/constants.hpp"
#include "subsystems/elevator/macros.hpp"
#include "util/angle.hpp"
#include <algorithm>
#include <cmath>

crimson::Chaos chaos;

namespace crimson {

namespace {

namespace claw = subsystems::elevator::claw;
namespace constants = subsystems::elevator::constants;
namespace macros = subsystems::elevator::macros;

constexpr std::uint32_t kPollMs = 10;

struct ArmRange {
    double min_deg;
    double max_deg;
};

struct Point {
    float x;
    float y;
};

std::size_t index(field::GoalId goal) {
    return static_cast<std::size_t>(goal);
}

std::size_t index(field::Wall wall) {
    return static_cast<std::size_t>(wall);
}

Point position_in(field::GoalId goal) {
    const field::Goal& g = field::goal(goal);
    return {static_cast<float>(g.pos.x / field::kMmPerInch), static_cast<float>(g.pos.y / field::kMmPerInch)};
}

constants::Goal height_kind(field::GoalId goal) {
    if (field::goal(goal).type != field::GoalType::Neutral) {
        return constants::Goal::Alliance;
    }
    return goal == field::GoalId::Center ? constants::Goal::Center : constants::Goal::Neutral;
}

ArmRange arm_range(Side side) {
    switch (side) {
    case Side::Front:
        return {macros::kArmFrontMinDeg, macros::kArmFrontMaxDeg};
    case Side::Behind:
        return {macros::kArmBehindMinDeg, macros::kArmBehindMaxDeg};
    case Side::Auto:
        break;
    }
    return {macros::kArmFrontMinDeg, macros::kArmBehindMaxDeg};
}

bool solve_for(double tip_cm, Side side, macros::Pose& out) {
    const ArmRange range = arm_range(side);
    return macros::solve(tip_cm, out, range.min_deg, range.max_deg);
}

double heading_toward(const lemlib::Pose& robot, Point target) {
    return util::rad_to_deg(-std::atan2(target.y - robot.y, target.x - robot.x));
}

double distance_to(const lemlib::Pose& robot, Point target) {
    return std::hypot(target.x - robot.x, target.y - robot.y);
}

double travel_heading(const lemlib::Pose& robot, Point target, bool forwards) {
    return heading_toward(robot, target) + (forwards ? 0.0 : 180.0);
}

double turn_seconds(double from_deg, double to_deg) {
    return std::fabs(util::wrap_signed_degrees(to_deg - from_deg)) / kTurnDegPerSec;
}

double drive_seconds(const lemlib::Pose& robot, Point target, bool forwards) {
    const double distance = distance_to(robot, target);
    if (distance <= 1e-6) {
        return 0.0;
    }
    return distance / kDriveInPerSec + turn_seconds(robot.theta, travel_heading(robot, target, forwards));
}

double pose_seconds(const lemlib::Pose& robot, Point target, double heading, bool forwards) {
    const double arrive_heading =
        distance_to(robot, target) <= 1e-6 ? robot.theta : travel_heading(robot, target, forwards);
    return drive_seconds(robot, target, forwards) + turn_seconds(arrive_heading, heading);
}

int timeout_for(double seconds) {
    return static_cast<int>(seconds * 1000.0 * kTimeoutScale) + kTimeoutPadMs;
}

field::Color opponent_color() {
    return kOurColor == field::Color::Blue ? field::Color::Red : field::Color::Blue;
}

field::Color after_ram(field::Color color) {
    return color == field::Color::Yellow ? opponent_color() : kOurColor;
}

template <typename Done>
bool wait_until(Done done, int timeout_ms) {
    const std::uint32_t start = pros::millis();
    while (!done()) {
        if (pros::millis() - start >= static_cast<std::uint32_t>(timeout_ms)) {
            return false;
        }
        pros::delay(kPollMs);
    }
    return true;
}

}

Chaos::Chaos() {
    reset();
}

void Chaos::reset() {
    for (std::size_t i = 0; i < stacks.size(); ++i) {
        stacks[i] = {};
        const int yellows = field::starting_yellow_pins(field::kGoals[i]);
        for (int pin = 0; pin < yellows; ++pin) {
            stacks[i].colors[stacks[i].count++] = field::Color::Yellow;
        }
    }
    toggles.fill(field::Color::Yellow);
    held.reset();
}

int Chaos::pins(field::GoalId goal) const {
    return static_cast<int>(stacks[index(goal)].count);
}

std::optional<field::Color> Chaos::top(field::GoalId goal) const {
    const Stack& stack = stacks[index(goal)];
    if (stack.count == 0) {
        return std::nullopt;
    }
    return stack.colors[stack.count - 1];
}

void Chaos::setPins(field::GoalId goal, std::initializer_list<field::Color> colors) {
    Stack& stack = stacks[index(goal)];
    stack = {};
    for (const field::Color color : colors) {
        if (stack.count == kMaxPins) {
            break;
        }
        stack.colors[stack.count++] = color;
    }
}

double Chaos::stackTopCm(field::GoalId goal) const {
    return constants::goal_height_cm(height_kind(goal)) + pins(goal) * constants::kPinHeightCm;
}

double Chaos::releaseCm(field::GoalId goal) const {
    return constants::release_cm(height_kind(goal), pins(goal) + 1);
}

field::Color Chaos::toggleColor(field::Wall wall) const {
    return toggles[index(wall)];
}

void Chaos::setToggle(field::Wall wall, field::Color color) {
    toggles[index(wall)] = color;
}

std::optional<field::Color> Chaos::holding() const {
    return held;
}

void Chaos::start(float x, float y, float heading) {
    chassis.setPose(x, y, heading);
}

void Chaos::grab(field::Color color) {
    claw::grab();
    held = color;
}

void Chaos::release() {
    claw::release();
    held.reset();
}

bool Chaos::pickup(field::Color color, std::uint32_t timeoutMs) {
    macros::Pose pose{};
    if (!solve_for(kPickupTipCm, Side::Front, pose)) {
        return false;
    }
    const int swing_timeout = timeout_for(macros::travel_seconds(pose));
    macros::aim(pose);
    if (!wait_until(macros::settled, swing_timeout)) {
        return false;
    }
    if (!macros::pickup(timeoutMs)) {
        return false;
    }
    held = color;
    return true;
}

bool Chaos::home() {
    return macros::home();
}

bool Chaos::moveTo(field::GoalId goal, Side side) {
    macros::Pose pose{};
    if (full(goal) || !solve_for(releaseCm(goal), side, pose)) {
        return false;
    }
    const Point point = position_in(goal);
    double seconds = macros::travel_seconds(pose);
    if (const auto stop = macros::approach(pose, point.x, point.y)) {
        seconds = std::max(seconds, drive_seconds(chassis.getPose(), {stop->x, stop->y}, stop->forwards));
    }
    return macros::reach(pose, point.x, point.y, static_cast<std::uint32_t>(timeout_for(seconds)));
}

bool Chaos::score(field::GoalId goal, Side side) {
    if (!moveTo(goal, side)) {
        return false;
    }
    macros::drop();
    place(goal);
    return true;
}

bool Chaos::donutWhileScoring(field::GoalId goal, Side side, DonutParams params) {
    macros::Pose pose{};
    if (full(goal) || !solve_for(releaseCm(goal), side, pose)) {
        return false;
    }

    const lemlib::Pose robot = chassis.getPose();
    const double toward = heading_toward(robot, position_in(goal));
    const bool claw_in_front = macros::claw_offset_in(pose) >= 0.0;
    const auto heading = static_cast<float>(util::wrap_signed_degrees(claw_in_front ? toward : toward + 180.0));
    const double turn = std::fabs(lemlib::angleError(heading, robot.theta, false, params.direction));
    const int timeout = timeout_for(std::max(turn / kDonutDegPerSec, macros::travel_seconds(pose)));

    chassis.donut(heading, timeout, params, true);
    macros::aim(pose);
    if (!wait_until([] { return !chassis.isInMotion() && macros::settled(); }, timeout)) {
        return false;
    }
    macros::drop();
    place(goal);
    return true;
}

void Chaos::toggle(field::Wall wall, bool forwards) {
    while (toggles[index(wall)] != kOurColor) {
        ram(wall, forwards);
        toggles[index(wall)] = after_ram(toggles[index(wall)]);
    }
}

bool Chaos::resetFrom(field::GoalId goal, Side side) {
    const Point point = position_in(goal);
    bool front = side == Side::Front;
    if (side == Side::Auto) {
        const lemlib::Pose robot = chassis.getPose();
        front = std::fabs(util::wrap_signed_degrees(heading_toward(robot, point) - robot.theta)) < 90.0;
    }
    return chassis.setPoseFromGoal(point.x, point.y, {.forwards = front, .maxCorrection = kResetMaxCorrectionIn});
}

bool Chaos::full(field::GoalId goal) const {
    return stacks[index(goal)].count == kMaxPins;
}

void Chaos::place(field::GoalId goal) {
    Stack& stack = stacks[index(goal)];
    if (stack.count < kMaxPins) {
        stack.colors[stack.count++] = held.value_or(kOurColor);
    }
    held.reset();
}

void Chaos::ram(field::Wall wall, bool forwards) {
    const field::ToggleBar& bar = field::toggle_bar(wall);
    const double facing = util::deg_to_rad(bar.ram_heading_deg);
    const Point approach{static_cast<float>(bar.center_in.x - std::cos(facing) * kToggleApproachIn),
                         static_cast<float>(bar.center_in.y + std::sin(facing) * kToggleApproachIn)};
    const Point center{static_cast<float>(bar.center_in.x), static_cast<float>(bar.center_in.y)};
    const auto heading =
        static_cast<float>(util::wrap_signed_degrees(forwards ? bar.ram_heading_deg : bar.ram_heading_deg + 180.0));

    chassis.moveToPose(approach.x, approach.y, heading,
                       timeout_for(pose_seconds(chassis.getPose(), approach, heading, forwards)),
                       {.forwards = forwards}, false);
    chassis.moveToPoint(center.x, center.y, kRamPushMs, {.forwards = forwards, .minSpeed = kRamMinSpeed}, false);
    chassis.moveToPoint(approach.x, approach.y, timeout_for(drive_seconds(chassis.getPose(), approach, !forwards)),
                        {.forwards = !forwards}, false);
}

}
