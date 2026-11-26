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

namespace subsystems::elevator::macros {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kFullPower = 127;

int stage = -1;           // stage last commanded: 0 home, 1-7 scoring, -1 none yet
int claw_seen = -1;       // claw state on the last tick: 1 closed, 0 open, -1 not looked yet
std::uint32_t carry_at_ms = 0; // a carry raise waiting for its delay: when to send it (0: none)
int goal_tier = 0;        // 0 alliance, 1 neutral, 2 center
bool lift_manual = false; // R1/R2 driving the lift by hand
int selected = 1;         // stage Up/Down have selected (1-7); L1 goes there
bool score_held = false;  // L1 was held on the last tick

// Scoring after L1 is released: the claw opens, kOuttakeRaiseDelayMs later the lift goes up
// kOuttakeRaiseCm, and once it is up (or kRaiseTimeoutMs passes) everything goes home.
enum class Score { Idle, Opened, Raising };
constexpr double kRaiseDoneCm = 1.5;
constexpr std::uint32_t kRaiseTimeoutMs = 1000;
Score score = Score::Idle;
std::uint32_t score_at_ms = 0;

// Bumped by every lift/arm command, so an aim_swing_first() still running in its task gives up once something else
// takes over.
std::atomic<std::uint32_t> command_id{0};
// Each blocking auton motion takes a generation; stop() bumps it, which ends every wait.
std::atomic<std::uint32_t> generation{0};

double pending_arm_motor_deg = 0.0; // the arm half of the last aim_lift(), for swing_arm()

double offset_cm() {
    return goal_tier * constants::kGoalTierStepCm;
}

Pose current_pose() {
    return {constants::extension_motor_deg_to_cm(extension::height()),
            constants::arm_motor_deg_to_deg(arm_pivot::get_pos())};
}

// Lift and arm to a stage's setpoint. Scoring stages get the goal offset on the lift and
// the slack adjustment on the arm (toward 180 deg); home gets neither, and its lift is
// home_lift_cm() (up a bit while holding a piece). Going to L1 or home, the arm's last 30 deg
// runs at 35%.
void go_to_stage(int next) {
    ++command_id;
    stage = std::clamp(next, 0, constants::kTopStage);
    carry_at_ms = 0; // this sets the lift itself
    const constants::Setpoint& setpoint = constants::kStageSetpoints[stage];
    const bool scoring = stage > 0;
    arm_pivot::set_slow_zone(stage <= 1);
    extension::set_target(
        constants::extension_cm_to_motor_deg(scoring ? setpoint.lift_cm + offset_cm() : home_lift_cm()));
    if (scoring) {
        arm_pivot::move_to(constants::arm_deg_to_motor_deg(setpoint.arm_deg - constants::kArmSlackAdjustmentDeg));
    } else {
        arm_pivot::home(constants::arm_deg_to_motor_deg(setpoint.arm_deg)); // then into the hardstop
    }
}

// The claw opened or closed outside a score: closing at home raises the lift to the carry height
// constants::kCarryDelayMs later (never lowers it), opening at home brings it straight back to 0. at_home: the arm is
// home (or homing).
void claw_changed(bool at_home) {
    carry_at_ms = 0;
    if (!at_home) {
        return;
    }
    if (claw::is_closed()) {
        const std::uint32_t at = pros::millis() + static_cast<std::uint32_t>(constants::kCarryDelayMs);
        carry_at_ms = at == 0 ? 1 : at; // grab first; update_carry lifts once the delay is up
    } else {
        extension::set_target(constants::extension_cm_to_motor_deg(0.0));
    }
}

// Sends a waiting carry raise once its delay is up, if the claw is still closed and the arm still home.
void update_carry() {
    if (carry_at_ms == 0 || static_cast<std::int32_t>(pros::millis() - carry_at_ms) < 0) {
        return;
    }
    carry_at_ms = 0;
    const bool home = constants::arm_motor_deg_to_deg(arm_pivot::target()) < 90.0; // home is in front
    if (!claw::is_closed() || !home) {
        return;
    }
    const double lift_cm = constants::extension_motor_deg_to_cm(extension::target());
    if (!(lift_cm >= constants::kCarryLiftCm)) { // also catches a NaN target
        extension::set_target(constants::extension_cm_to_motor_deg(constants::kCarryLiftCm));
    }
}

}

double home_lift_cm() {
    return claw::is_closed() ? constants::kCarryLiftCm : 0.0;
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
    // Up/Down pick the stage (shown on the controller); nothing moves until L1.
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
        score = Score::Idle;
        if (stage != selected) {
            go_to_stage(selected);
        }
    } else if (score_held) {
        // Released: score. Hard-set the claw open (an outtake, not a toggle), then the
        // sequence below raises the lift and homes.
        claw::open();
        score = Score::Opened;
        score_at_ms = now;
    }
    score_held = go;

    if (score == Score::Opened && now - score_at_ms >= static_cast<std::uint32_t>(constants::kOuttakeRaiseDelayMs)) {
        extension::set_target(extension::target() + constants::extension_cm_to_motor_deg(constants::kOuttakeRaiseCm));
        score = Score::Raising;
        score_at_ms = now;
    } else if (score == Score::Raising) {
        const double gap_cm = std::fabs(constants::extension_motor_deg_to_cm(extension::target() - extension::height()));
        if (gap_cm < kRaiseDoneCm || now - score_at_ms >= kRaiseTimeoutMs) {
            score = Score::Idle;
            go_to_stage(0);
        }
    }

    if (controller.get_digital_new_press(controls::kHome)) {
        score = Score::Idle;
        go_to_stage(0);
    }

    // R1/R2: the lift by hand; it holds where it is let go.
    const bool lift_up = controller.get_digital(controls::kLiftManualUp);
    const bool lift_down = controller.get_digital(controls::kLiftManualDown) && !lift_up;
    if (lift_up || lift_down) {
        score = Score::Idle; // the driver has the lift
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

    // The claw (A / X, its own task) opened or closed: at home, the lift follows it up to the carry height or back
    // down. Not during a score (that opens the claw itself) or while the driver has the lift. Closing counts as home
    // before the first stage press too (holding a preload at the start); opening only after a real home.
    const int closed = claw::is_closed() ? 1 : 0;
    if (closed != claw_seen) {
        const bool first = claw_seen < 0;
        claw_seen = closed;
        if (score == Score::Idle && !lift_manual && (!first || closed == 1)) {
            claw_changed(closed == 1 ? stage <= 0 : stage == 0);
        }
    }
    if (lift_manual) {
        carry_at_ms = 0; // the driver has the lift
    }
    update_carry();
}

namespace {

constexpr std::uint32_t kPollMs = 10;
constexpr std::uint32_t kGripMs = 250; // new-robot's, tuned in its autons
constexpr std::uint32_t kHomeTimeoutMs = 2000;
// drop(): with the lift above kSteadyLiftCm, wait for the robot to stop rocking first: its tilt rate (the IMU's x and
// y, not yaw) under kSteadyDegPerS for kSteadyHoldMs straight, at most kSteadyTimeoutMs. At L7 the rock is about once a
// second; at the top of each sway the rate is near 0 only briefly (a 2.3 deg sway stays under 2 deg/s for about 40 ms),
// so holding 100 ms only passes once the sway is about 1 deg or less (~2 cm at an L7 claw). With the lift down it
// barely rocks, so the check is skipped there.
constexpr double kSteadyLiftCm = 15.0;
constexpr double kSteadyDegPerS = 2.0;
constexpr std::uint32_t kSteadyHoldMs = 100;
constexpr std::uint32_t kSteadyTimeoutMs = 1000;

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

// Blocks until the robot has stopped rocking (see kSteadyLiftCm). False only if stop() cancelled it; a timeout, or no
// IMU reading, returns true so the score still happens.
bool wait_steady() {
    if (!(current_pose().lift_cm >= kSteadyLiftCm)) {
        return true;
    }
    const std::uint32_t mine = generation;
    const std::uint32_t start = pros::millis();
    std::uint32_t calm_since = 0;
    while (generation == mine) {
        const std::uint32_t now = pros::millis();
        const pros::imu_gyro_s_t rate = imu.get_gyro_rate();
        const double tilt_rate = std::hypot(rate.x, rate.y);
        if (!std::isfinite(tilt_rate) || now - start >= kSteadyTimeoutMs) {
            return true;
        }
        if (tilt_rate < kSteadyDegPerS) {
            if (calm_since == 0) {
                calm_since = now == 0 ? 1 : now;
            }
            if (now - calm_since >= kSteadyHoldMs) {
                return true;
            }
        } else {
            calm_since = 0;
        }
        pros::delay(kPollMs);
    }
    return false;
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

// Off the driver stages, for an auton motion.
void start_auton_command() {
    ++command_id;
    stage = -1;
    carry_at_ms = 0;
    arm_pivot::set_slow_zone(false);
}

}

void aim(Pose pose) {
    start_auton_command();
    extension::set_target(constants::extension_cm_to_motor_deg(pose.lift_cm));
    arm_pivot::move_to(constants::arm_deg_to_motor_deg(slack_adjusted(pose.arm_deg)));
}

void aim_lift(Pose pose) {
    start_auton_command();
    extension::set_target(constants::extension_cm_to_motor_deg(pose.lift_cm));
    pending_arm_motor_deg = constants::arm_deg_to_motor_deg(slack_adjusted(pose.arm_deg));
}

bool swing_arm(std::uint32_t timeout_ms) {
    ++command_id;
    arm_pivot::move_to(pending_arm_motor_deg);
    return wait_for(settled, timeout_ms);
}

void aim_swing_first(Pose pose, std::function<bool()> clear, std::uint32_t timeout_ms) {
    start_auton_command();
    const std::uint32_t mine = command_id;
    const double lift_target = constants::extension_cm_to_motor_deg(pose.lift_cm);
    const double arm_target = constants::arm_deg_to_motor_deg(slack_adjusted(pose.arm_deg));
    if (!(extension::target() <= lift_target)) {
        extension::set_target(lift_target); // never up before the swing; down right away
    }

    pros::Task task([=] {
        // False only once another lift/arm command has taken over; a timeout just moves on to the next step.
        const auto wait = [mine, timeout_ms](const auto& done) {
            const std::uint32_t start = pros::millis();
            while (command_id == mine && !done() && pros::millis() - start < timeout_ms) {
                pros::delay(kPollMs);
            }
            return command_id == mine;
        };
        if (clear && !wait(clear)) {
            return;
        }
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

bool wait_settled(std::uint32_t timeout_ms) {
    return wait_for(settled, timeout_ms);
}

bool drop() {
    if (!wait_steady()) {
        return false;
    }
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

void stop() {
    ++generation;
    ++command_id;
    carry_at_ms = 0;
    score = Score::Idle;
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
