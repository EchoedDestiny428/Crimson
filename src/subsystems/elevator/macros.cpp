#include "subsystems/elevator/macros.hpp"
#include "config/controls.hpp"
#include "pros/rtos.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator/arm_pivot.hpp"
#include "subsystems/elevator/claw.hpp"
#include "subsystems/elevator/constants.hpp"
#include "subsystems/elevator/extension.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>

namespace subsystems::elevator::macros {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kFullPower = 127;

int stage = -1;           // stage last commanded: 0 home, 1-7 scoring, -1 none yet
int goal_tier = 0;        // 0 alliance, 1 neutral, 2 center
bool lift_manual = false; // Up/Down driving the lift by hand
int selected = 1;         // stage R1/R2 have selected (1-7); L1 goes there
bool score_held = false;  // L1 was held on the last tick

// Scoring after L1 is released: the claw opens, kOuttakeRaiseDelayMs later the lift goes up
// kOuttakeRaiseCm, and once it is up (or kRaiseTimeoutMs passes) everything goes home.
enum class Score { Idle, Opened, Raising };
constexpr double kRaiseDoneCm = 1.5;
constexpr std::uint32_t kRaiseTimeoutMs = 1000;
Score score_state = Score::Idle;
std::uint32_t score_at_ms = 0;

// B: go home, then open the claw once the arm is on the hardstop and the lift is within
// kRaiseDoneCm of home (or kHomeReleaseTimeoutMs passes). L1 cancels it.
constexpr std::uint32_t kHomeReleaseTimeoutMs = 2000;
bool release_at_home = false;
std::uint32_t home_at_ms = 0;

// Bumped by every lift/arm command, so a staged aim still running in its task gives up once something else takes over.
std::atomic<std::uint32_t> command_id{0};

double pending_arm_motor_deg = 0.0; // the arm half of the last aim_lift(), for swing_arm()

double offset_cm() {
    return goal_tier * constants::kGoalTierStepCm;
}

Pose current_pose() {
    return {constants::extension_motor_deg_to_cm(extension::height()),
            constants::arm_motor_deg_to_deg(arm_pivot::get_pos())};
}

// Lift and arm to a stage's setpoint. Scoring stages get the goal offset on the lift and
// the slack adjustment on the arm (toward 180 deg); home gets neither. Going to L1 or home,
// the arm's last 30 deg runs at 35%.
void go_to_stage(int next) {
    ++command_id;
    stage = std::clamp(next, 0, constants::kTopStage);
    const constants::Setpoint& setpoint = constants::kStageSetpoints[stage];
    const bool scoring = stage > 0;
    arm_pivot::set_slow_zone(stage <= 1);
    extension::set_target(constants::extension_cm_to_motor_deg(setpoint.lift_cm + (scoring ? offset_cm() : 0.0)));
    if (scoring) {
        arm_pivot::move_to(constants::arm_deg_to_motor_deg(setpoint.arm_deg - constants::kArmSlackAdjustmentDeg));
    } else {
        arm_pivot::home(constants::arm_deg_to_motor_deg(setpoint.arm_deg)); // then into the hardstop
    }
}

}

// Fastest-pose solver, kept for autons (the driver stages don't use it: they are hard
// setpoints). tip height = pivot height + lift - arm length * cos(arm). Tries every arm
// angle in [arm_min_deg, arm_max_deg], solves the lift that reaches tip_cm, and keeps the
// pose both reach soonest from `from` (lift at kExtensionMaxRpm, arm at 100 rpm); a coarse
// 0.5 deg sweep, then a 0.01 deg one around the best. Behind with the release point behind
// the pivot is arm 194.0 deg up to the 315 deg soft limit.
double move_seconds(const Pose& from, const Pose& to) {
    constexpr double kLiftCmPerS = constants::kExtensionMaxRpm / 60.0 / constants::kExtensionMotorRotationsPerCm;
    constexpr double kArmDegPerS = 100 * 360.0 / 60.0 / constants::kArmPivotMotorRotationsPerArmRotation;
    return std::max(std::fabs(to.lift_cm - from.lift_cm) / kLiftCmPerS,
                    std::fabs(to.arm_deg - from.arm_deg) / kArmDegPerS);
}

bool solve_from(const Pose& from, double tip_cm, Pose& out, double arm_min_deg, double arm_max_deg) {
    constexpr double kSolveStepDeg = 0.5;
    constexpr double kSolveFineStepDeg = 0.01;
    bool found = false;
    double best_s = 0.0;
    const auto consider = [&](double arm) {
        const double lift =
            tip_cm - constants::kPivotStartHeightCm + constants::kArmLengthCm * std::cos(arm * kPi / 180.0);
        if (lift < 0.0) {
            return;
        }
        const Pose pose{lift, arm};
        const double s = move_seconds(from, pose);
        if (!found || s < best_s) {
            out = pose;
            best_s = s;
            found = true;
        }
    };
    for (double arm = arm_min_deg; arm <= arm_max_deg; arm += kSolveStepDeg) {
        consider(arm);
    }
    if (found) {
        const double center = out.arm_deg;
        const double lo = std::max(arm_min_deg, center - kSolveStepDeg);
        const double hi = std::min(arm_max_deg, center + kSolveStepDeg);
        for (double arm = lo; arm <= hi; arm += kSolveFineStepDeg) {
            consider(arm);
        }
    }
    return found;
}

void update() {
    // R1/R2 pick the stage (shown on the controller); nothing moves until L1.
    if (controller.get_digital_new_press(controls::kStageSelectUp)) {
        selected = std::min(selected + 1, constants::kTopStage);
    }
    if (controller.get_digital_new_press(controls::kStageSelectDown)) {
        selected = std::max(selected - 1, 1);
    }

    const std::uint32_t now = pros::millis();
    const bool go = controller.get_digital(controls::kScore);
    if (go) {
        // Held: go to the selected stage (and follow it if it changes), cancelling any score.
        score_state = Score::Idle;
        release_at_home = false;
        if (stage != selected) {
            go_to_stage(selected);
        }
    } else if (score_held) {
        // Released: score. Hard-set the claw open (an outtake, not a toggle), then the
        // sequence below raises the lift and homes.
        claw::open();
        score_state = Score::Opened;
        score_at_ms = now;
    }
    score_held = go;

    if (score_state == Score::Opened && now - score_at_ms >= static_cast<std::uint32_t>(constants::kOuttakeRaiseDelayMs)) {
        extension::set_target(extension::target() + constants::extension_cm_to_motor_deg(constants::kOuttakeRaiseCm));
        score_state = Score::Raising;
        score_at_ms = now;
    } else if (score_state == Score::Raising) {
        const double gap_cm = std::fabs(constants::extension_motor_deg_to_cm(extension::target() - extension::height()));
        if (gap_cm < kRaiseDoneCm || now - score_at_ms >= kRaiseTimeoutMs) {
            score_state = Score::Idle;
            go_to_stage(0);
        }
    }

    if (controller.get_digital_new_press(controls::kHome)) {
        score_state = Score::Idle;
        go_to_stage(0);
        release_at_home = true;
        home_at_ms = now;
    }
    if (release_at_home) {
        const double gap_cm = std::fabs(constants::extension_motor_deg_to_cm(extension::target() - extension::height()));
        if ((!arm_pivot::homing() && gap_cm < kRaiseDoneCm) || now - home_at_ms >= kHomeReleaseTimeoutMs) {
            release_at_home = false;
            claw::open();
        }
    }

    // Up/Down: the lift by hand; it holds where it is let go.
    const bool lift_up = controller.get_digital(controls::kLiftManualUp);
    const bool lift_down = controller.get_digital(controls::kLiftManualDown) && !lift_up;
    if (lift_up || lift_down) {
        score_state = Score::Idle; // the driver has the lift
        lift_manual = true;
        extension::set_manual(lift_up ? kFullPower : -constants::kExtensionDownPower);
    } else if (lift_manual) {
        lift_manual = false;
        extension::hold();
    }

    // Right cycles the goal (alliance -> neutral -> center): only the lift moves, right
    // away. At a scoring stage it goes to that stage plus the new offset; anywhere else it
    // shifts by the change in offset.
    if (controller.get_digital_new_press(controls::kGoalOffset)) {
        const double old_offset = offset_cm();
        goal_tier = (goal_tier + 1) % 3;
        if (!lift_manual) {
            const double lift = stage > 0 ? constants::kStageSetpoints[stage].lift_cm + offset_cm()
                                          : current_pose().lift_cm + offset_cm() - old_offset;
            extension::set_target(constants::extension_cm_to_motor_deg(lift));
        }
    }
}

namespace {

constexpr std::uint32_t kPollMs = 10;
constexpr std::uint32_t kGripMs = 250;
constexpr std::uint32_t kHomeTimeoutMs = 2000;

// Each blocking auton motion takes a generation; stop() bumps it, which ends every wait.
std::atomic<std::uint32_t> generation{0};

template <typename Done>
bool wait_for(Done done, std::uint32_t timeout_ms) {
    const std::uint32_t mine = generation;
    const std::uint32_t start = pros::millis();
    while (generation == mine && !done()) {
        if (pros::millis() - start >= timeout_ms) {
            return false;
        }
        pros::delay(kPollMs);
    }
    return generation == mine;
}

// Waits ms; false only if stop() cancelled it.
bool pause(std::uint32_t ms) {
    const std::uint32_t mine = generation;
    const std::uint32_t start = pros::millis();
    while (generation == mine && pros::millis() - start < ms) {
        pros::delay(kPollMs);
    }
    return generation == mine;
}

// Same slack adjustment as the driver stages, toward straight up (180) on either side.
double slack_adjusted(double arm_deg) {
    if (arm_deg > 180.0) {
        return arm_deg - constants::kArmSlackAdjustmentDeg;
    }
    if (arm_deg < 180.0) {
        return arm_deg + constants::kArmSlackAdjustmentDeg;
    }
    return arm_deg;
}

bool score_in_range(constants::Goal goal, int level, float goal_x_in, float goal_y_in, std::uint32_t timeout_ms,
                    double arm_min_deg, double arm_max_deg) {
    Pose pose{};
    if (level < 1 || !solve(constants::release_cm(goal, level), pose, arm_min_deg, arm_max_deg)) {
        return false;
    }
    return reach(pose, goal_x_in, goal_y_in, timeout_ms) && drop();
}

}

bool solve(double tip_cm, Pose& out, double arm_min_deg, double arm_max_deg) {
    return solve_from(current_pose(), tip_cm, out, arm_min_deg, arm_max_deg);
}

void aim(Pose pose) {
    ++command_id;
    stage = -1; // off the driver stages
    arm_pivot::set_slow_zone(false);
    extension::set_target(constants::extension_cm_to_motor_deg(pose.lift_cm));
    arm_pivot::move_to(constants::arm_deg_to_motor_deg(slack_adjusted(pose.arm_deg)));
}

void aim_lift(Pose pose) {
    ++command_id;
    stage = -1;
    arm_pivot::set_slow_zone(false);
    extension::set_target(constants::extension_cm_to_motor_deg(pose.lift_cm));
    pending_arm_motor_deg = constants::arm_deg_to_motor_deg(slack_adjusted(pose.arm_deg));
}

bool wait_settled(std::uint32_t timeout_ms) {
    return wait_for(settled, timeout_ms);
}

bool swing_arm(std::uint32_t timeout_ms) {
    arm_pivot::move_to(pending_arm_motor_deg);
    return wait_for(settled, timeout_ms);
}

void aim_staged(Pose pose, std::function<bool()> clear, std::uint32_t timeout_ms) {
    const std::uint32_t mine = ++command_id;
    stage = -1;
    arm_pivot::set_slow_zone(false);
    const double lift_target = constants::extension_cm_to_motor_deg(pose.lift_cm);
    const double arm_target = constants::arm_deg_to_motor_deg(slack_adjusted(pose.arm_deg));
    extension::set_target(lift_target);

    pros::Task task([=] {
        // False only once another lift/arm command has taken over; a timeout just moves on to the next step.
        const auto wait = [mine, timeout_ms](const auto& done) {
            const std::uint32_t start = pros::millis();
            while (command_id == mine && !done() && pros::millis() - start < timeout_ms) {
                pros::delay(kPollMs);
            }
            return command_id == mine;
        };
        if (!wait(clear)) {
            return;
        }
        extension::set_target(extension::height()); // hold the lift while the arm swings
        arm_pivot::move_to(arm_target);
        if (!wait([] { return arm_pivot::is_settled(); })) {
            return;
        }
        extension::set_target(lift_target);
    });
}

bool settled() {
    return extension::is_settled() && arm_pivot::is_settled();
}

bool drop() {
    claw::release();
    return pause(kGripMs);
}

void home_async() {
    go_to_stage(0);
}

bool home() {
    home_async();
    return wait_for(settled, kHomeTimeoutMs);
}

bool pickup() {
    claw::grab();
    return pause(kGripMs);
}

double travel_seconds(const Pose& to) {
    return move_seconds(current_pose(), to);
}

double claw_offset_in(const Pose& pose) {
    const double reach_cm = constants::kArmLengthCm * std::sin(pose.arm_deg * kPi / 180.0);
    return (reach_cm - constants::kPivotBehindCenterCm + constants::kClawReleaseOffsetCm) / constants::kCmPerInch;
}

std::optional<Approach> approach(const Pose& pose, float goal_x_in, float goal_y_in) {
    const lemlib::Pose robot = chassis.getPose();
    const double dx = goal_x_in - robot.x;
    const double dy = goal_y_in - robot.y;
    const double distance = std::hypot(dx, dy);
    if (distance <= 1e-6) {
        return std::nullopt;
    }
    const double offset = claw_offset_in(pose);
    const double standoff = std::fabs(offset);
    const bool claw_in_front = offset >= 0.0;
    const bool toward_goal = distance >= standoff;
    return Approach{static_cast<float>(goal_x_in - dx / distance * standoff),
                    static_cast<float>(goal_y_in - dy / distance * standoff), claw_in_front == toward_goal};
}

bool reach(const Pose& pose, float goal_x_in, float goal_y_in, std::uint32_t timeout_ms) {
    if (const auto stop = approach(pose, goal_x_in, goal_y_in)) {
        chassis.moveToPoint(stop->x, stop->y, static_cast<int>(timeout_ms), {.forwards = stop->forwards});
    }
    aim(pose);
    return wait_for([] { return !chassis.isInMotion() && settled(); }, timeout_ms);
}

bool score(constants::Goal goal, int level, float goal_x_in, float goal_y_in, std::uint32_t timeout_ms) {
    return score_in_range(goal, level, goal_x_in, goal_y_in, timeout_ms, kArmFrontMinDeg, kArmBehindMaxDeg);
}

bool score_front(constants::Goal goal, int level, float goal_x_in, float goal_y_in, std::uint32_t timeout_ms) {
    return score_in_range(goal, level, goal_x_in, goal_y_in, timeout_ms, kArmFrontMinDeg, kArmFrontMaxDeg);
}

bool score_behind(constants::Goal goal, int level, float goal_x_in, float goal_y_in, std::uint32_t timeout_ms) {
    return score_in_range(goal, level, goal_x_in, goal_y_in, timeout_ms, kArmBehindMinDeg, kArmBehindMaxDeg);
}

void stop() {
    ++generation;
    score_state = Score::Idle;
    release_at_home = false;
    score_held = false;
    stage = -1;
    lift_manual = false;
    extension::hold();
    arm_pivot::stop();
}

Pose pose_now() {
    return current_pose();
}

double tip_now() {
    const Pose pose = current_pose();
    return constants::kPivotStartHeightCm + pose.lift_cm -
           constants::kArmLengthCm * std::cos(pose.arm_deg * kPi / 180.0);
}

int selected_stage_now() {
    return selected;
}

int goal_tier_now() {
    return goal_tier;
}

Pose target_now() {
    return {constants::extension_motor_deg_to_cm(extension::target()), constants::arm_motor_deg_to_deg(arm_pivot::target())};
}

int target_stage() {
    return stage;
}

}
