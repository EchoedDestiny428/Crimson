#include "subsystems/elevator/macros.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator/arm_pivot.hpp"
#include "subsystems/elevator/constants.hpp"
#include "subsystems/elevator/extension.hpp"
#include <algorithm>
#include <cmath>

namespace subsystems::elevator::macros {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kFullPower = 127;

int stage = -1;           // stage last commanded: 0 home, 1-7 scoring, -1 none yet
int goal_tier = 0;        // 0 alliance, 1 neutral, 2 center
bool lift_manual = false; // R1/R2 driving the lift by hand

double offset_cm() {
    return goal_tier * constants::kGoalTierStepCm;
}

Pose current_pose() {
    return {constants::extension_motor_deg_to_cm(extension::height()),
            constants::arm_motor_deg_to_deg(arm_pivot::get_pos())};
}

// Lift and arm to a stage's setpoint. Scoring stages get the goal offset on the lift and
// the slack adjustment on the arm; home gets neither. Going to L1 or home,
// the arm's last 30 deg runs at 35%.
void go_to_stage(int next) {
    stage = std::clamp(next, 0, constants::kTopStage);
    const constants::Setpoint& setpoint = constants::kStageSetpoints[stage];
    const bool scoring = stage > 0;
    arm_pivot::set_slow_zone(stage <= 1);
    extension::set_target(constants::extension_cm_to_motor_deg(setpoint.lift_cm + (scoring ? offset_cm() : 0.0)));
    arm_pivot::move_to(constants::arm_deg_to_motor_deg(
        setpoint.arm_deg + (scoring ? constants::kArmSlackAdjustmentDeg : 0.0)));
}

}

// Fastest-pose solver, kept for autons (commented out: the driver stages are a hard arm
// flip plus lift). tip height = pivot height + lift - arm length * cos(arm). Tries every
// arm angle in [arm_min_deg, arm_max_deg], solves the lift that reaches tip_cm, and keeps
// the pose both reach soonest from `from` (lift 60 cm/s, arm 600 deg/s); a coarse 0.5 deg
// sweep, then a 0.01 deg one around the best. Behind with the release point behind the
// pivot is arm 194.0-340 deg.
//
// double move_seconds(const Pose& from, const Pose& to) {
//     constexpr double kLiftCmPerS = constants::kExtensionMaxRpm / 60.0 / constants::kExtensionMotorRotationsPerCm;
//     constexpr double kArmDegPerS = 100 * 360.0 / 60.0 / constants::kArmPivotMotorRotationsPerArmRotation;
//     return std::max(std::fabs(to.lift_cm - from.lift_cm) / kLiftCmPerS,
//                     std::fabs(to.arm_deg - from.arm_deg) / kArmDegPerS);
// }
//
// bool solve_from(const Pose& from, double tip_cm, Pose& out, double arm_min_deg, double arm_max_deg) {
//     constexpr double kSolveStepDeg = 0.5;
//     constexpr double kSolveFineStepDeg = 0.01;
//     bool found = false;
//     double best_s = 0.0;
//     const auto consider = [&](double arm) {
//         const double lift =
//             tip_cm - constants::kPivotStartHeightCm + constants::kArmLengthCm * std::cos(arm * kPi / 180.0);
//         if (lift < 0.0) {
//             return;
//         }
//         const Pose pose{lift, arm};
//         const double s = move_seconds(from, pose);
//         if (!found || s < best_s) {
//             out = pose;
//             best_s = s;
//             found = true;
//         }
//     };
//     for (double arm = arm_min_deg; arm <= arm_max_deg; arm += kSolveStepDeg) {
//         consider(arm);
//     }
//     if (found) {
//         const double center = out.arm_deg;
//         const double lo = std::max(arm_min_deg, center - kSolveStepDeg);
//         const double hi = std::min(arm_max_deg, center + kSolveStepDeg);
//         for (double arm = lo; arm <= hi; arm += kSolveFineStepDeg) {
//             consider(arm);
//         }
//     }
//     return found;
// }

void update() {
    if (controller.get_digital_new_press(controls::kStageDown)) {
        go_to_stage(stage < 1 ? 1 : std::max(1, stage - 1));
    }
    if (controller.get_digital_new_press(controls::kStageUp)) {
        go_to_stage(stage < 1 ? 1 : stage + 1);
    }
    if (controller.get_digital_new_press(controls::kHome)) {
        go_to_stage(0);
    }

    // R1/R2: the lift by hand; it holds where it is let go.
    const bool lift_up = controller.get_digital(controls::kLiftManualUp);
    const bool lift_down = controller.get_digital(controls::kLiftManualDown) && !lift_up;
    if (lift_up || lift_down) {
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
