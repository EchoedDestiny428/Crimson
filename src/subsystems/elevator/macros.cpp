#include "subsystems/elevator/macros.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator/arm_pivot.hpp"
#include "subsystems/elevator/constants.hpp"
#include "subsystems/elevator/extension.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace subsystems::elevator::macros {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSolveStepDeg = 0.5;
constexpr double kSolveFineStepDeg = 0.01;

constexpr int kFullPower = 127;
constexpr int kArmUpPower = 89;   // 70% of full, arm only, while L1 is held
constexpr int kArmDownPower = 44; // 35% of full, arm only, while L2 is held

int dir = 0;          // +1 while L1 is held, -1 while L2 is held
bool lift_manual = false; // R1/R2 driving the lift by hand
int goal_tier = 0;    // 0 alliance, 1 neutral, 2 center: lifts the elevator kGoalTierStepCm per tier
int snapped_stage = -1; // stage of the last snap, so the offset can move the lift there
bool behind = false;    // side swap (Y): snaps score behind (reverse dump) and L1/L2 flip the arm

double lift_cm() {
    return constants::extension_motor_deg_to_cm(extension::height());
}

void set_lift(double cm) {
    extension::set_target(constants::extension_cm_to_motor_deg(cm));
}

// Every arm setpoint goes through here, slack adjustment included.
void set_arm(double deg) {
    arm_pivot::move_to(constants::arm_deg_to_motor_deg(deg + constants::kArmSlackAdjustmentDeg));
}

double lift_cm_per_s() {
    return constants::kExtensionMaxRpm / 60.0 / constants::kExtensionMotorRotationsPerCm;
}

double arm_deg_per_s() {
    return constants::kArmPivotMoveRpm * 360.0 / 60.0 / constants::kArmPivotMotorRotationsPerArmRotation;
}

Pose current_pose() {
    return {lift_cm(), constants::arm_motor_deg_to_deg(arm_pivot::get_pos())};
}

// Both mechanisms move at once, so a move takes as long as the slower one.
double move_seconds(const Pose& from, const Pose& to) {
    return std::max(std::fabs(to.lift_cm - from.lift_cm) / lift_cm_per_s(),
                    std::fabs(to.arm_deg - from.arm_deg) / arm_deg_per_s());
}

// tip height = pivot start height + lift - arm length * cos(arm angle from straight down).
// Try every arm angle in range, solve the lift that reaches tip_cm, and keep the
// pose both mechanisms reach soonest. Lifts below the start are skipped. A coarse
// sweep finds the best region, then a fine sweep around it finds the fastest pose.
bool solve_from(const Pose& from, double tip_cm, Pose& out, double arm_min_deg, double arm_max_deg) {
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

double tip_cm(const Pose& pose) {
    return constants::kPivotStartHeightCm + pose.lift_cm - constants::kArmLengthCm * std::cos(pose.arm_deg * kPi / 180.0);
}

double stage_tip_cm(int stage) {
    return constants::release_cm(kStageGoal, stage);
}

// Precomputed stage setpoints, one table per side: stage 0 is home; each stage after it
// is the solver's fastest pose from the stage before, claw in front of the pivot, or
// behind it (reverse dump: the arm goes up and over the top and drops the piece behind).
std::array<Pose, kTopStage + 1> solve_stages(double arm_min_deg, double arm_max_deg) {
    std::array<Pose, kTopStage + 1> out{};
    out[0] = {0.0, constants::kArmStartDeg};
    for (int stage = 1; stage <= kTopStage; ++stage) {
        if (!solve_from(out[stage - 1], stage_tip_cm(stage), out[stage], arm_min_deg, arm_max_deg)) {
            out[stage] = out[stage - 1];
        }
    }
    return out;
}

const std::array<Pose, kTopStage + 1>& stage_poses() {
    static const auto front = solve_stages(kArmFrontMinDeg, kArmFrontMaxDeg);
    static const auto back = solve_stages(kArmBehindMinDeg, kArmBehindMaxDeg);
    return behind ? back : front;
}

double offset_cm() {
    return goal_tier * constants::kGoalTierStepCm;
}

// The stage whose release height on the selected goal (the offset included) is closest
// to where the piece actually is.
int closest_stage(double tip) {
    int best = 1;
    for (int stage = 2; stage <= kTopStage; ++stage) {
        if (std::fabs(stage_tip_cm(stage) + offset_cm() - tip) < std::fabs(stage_tip_cm(best) + offset_cm() - tip)) {
            best = stage;
        }
    }
    return best;
}

// Release: the lift to the stage's height (slowing over its last 5 cm) and the arm to
// the stage's angle on its SmartMotor PID.
void snap(int stage) {
    snapped_stage = std::clamp(stage, 0, kTopStage);
    const Pose pose = stage_poses()[snapped_stage];
    // Stage 1 or home: the arm's last 30 deg (falling, on either side) at 35%.
    arm_pivot::set_slow_zone(snapped_stage <= 1);
    set_lift(pose.lift_cm + (snapped_stage > 0 ? offset_cm() : 0.0));
    set_arm(pose.arm_deg);
}

}

// Hold L1/L2: 100% to the lift and the arm, up/down, no control loop. Release: snap to the
// stage whose release height is closest to the piece's actual height (from the lift and
// arm), lift and arm each to that stage's setpoint. B goes home.
void update() {
    const bool up = controller.get_digital(controls::kExtensionUp);
    const bool down = controller.get_digital(controls::kExtensionDown) && !up;
    const int held = up ? 1 : down ? -1 : 0;

    // R1/R2: the lift by hand, no snap; it holds where it is let go. L1/L2 take priority.
    const bool lift_up = controller.get_digital(controls::kLiftManualUp);
    const bool lift_down = controller.get_digital(controls::kLiftManualDown) && !lift_up;
    if (held == 0 && dir == 0 && (lift_up || lift_down)) {
        lift_manual = true;
        extension::set_manual(lift_up ? kFullPower : -constants::kExtensionDownPower);
    } else if (lift_manual) {
        lift_manual = false;
        extension::hold();
    }

    if (held != 0) {
        // Hold: lift 100% up / 35% down, arm 70% up / 35% down, no control loop. Behind,
        // the arm's directions flip.
        extension::set_manual(held > 0 ? kFullPower : -constants::kExtensionDownPower);
        const int arm_power = held > 0 ? kArmUpPower : -kArmDownPower;
        arm_pivot::set_power(behind ? -arm_power : arm_power);
    } else if (dir != 0) {
        // Release: the only place motor control runs.
        snap(closest_stage(tip_cm(current_pose())));
    }
    dir = held;

    if (controller.get_digital_new_press(controls::kHome)) {
        snap(0);
    }

    // Y swaps front <-> back: the arm goes to the same stage on the other side (stage 1
    // from home or between stages). Swapping to the front at home stays home.
    if (controller.get_digital_new_press(controls::kSideSwap)) {
        behind = !behind;
        if (held == 0 && (behind || snapped_stage != 0)) {
            snap(snapped_stage > 0 ? snapped_stage : closest_stage(tip_cm(current_pose())));
        }
    }

    // Right cycles the goal (alliance -> neutral -> center): only the elevator moves, right
    // away. At a stage it goes to that stage plus the new offset; anywhere else it shifts by
    // the change in offset. While L1/L2 is held the release snap picks the offset up.
    if (controller.get_digital_new_press(controls::kGoalOffset)) {
        const double old_offset = offset_cm();
        goal_tier = (goal_tier + 1) % 3;
        if (held == 0) {
            if (snapped_stage > 0) {
                set_lift(stage_poses()[snapped_stage].lift_cm + offset_cm());
            } else {
                set_lift(current_pose().lift_cm + offset_cm() - old_offset);
            }
        }
    }
}

void stop() {
    dir = 0;
    lift_manual = false;
    snapped_stage = -1;
    extension::hold();
    arm_pivot::stop();
}

Pose pose_now() {
    return current_pose();
}

double tip_now() {
    return tip_cm(current_pose());
}

int snap_stage_now() {
    return closest_stage(tip_now());
}

bool behind_now() {
    return behind;
}

int goal_tier_now() {
    return goal_tier;
}

Pose target_now() {
    return {constants::extension_motor_deg_to_cm(extension::target()), constants::arm_motor_deg_to_deg(arm_pivot::target())};
}

int target_stage() {
    return snapped_stage;
}

}
