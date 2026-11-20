#include "subsystems/elevator/macros.hpp"
#include "config/controls.hpp"
#include "pros/rtos.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator/arm_pivot.hpp"
#include "subsystems/elevator/claw.hpp"
#include "subsystems/elevator/constants.hpp"
#include "subsystems/elevator/extension.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace subsystems::elevator::macros {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kFullPower = 127;

int stage = -1;           // stage last commanded: 0 home, 1-7 scoring, -1 none yet
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
}

void stop() {
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
