#include "subsystems/elevator/macros.hpp"
#include "config/controls.hpp"
#include "pros/rtos.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator/arm_pivot.hpp"
#include "subsystems/elevator/claw.hpp"
#include "subsystems/elevator/constants.hpp"
#include "subsystems/elevator/extension.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace subsystems::elevator::macros {

namespace {

enum class Mode { Stowed, Deployed };

constexpr double kTopArrivalToleranceCm = 2.0; // guess; the old robot used 50 motor deg

constexpr double kPi = 3.14159265358979323846;
constexpr double kSolveStepDeg = 0.5;
constexpr double kSolveFineStepDeg = 0.01;
constexpr double kMoveTimeoutScale = 1.5;
constexpr std::uint32_t kMoveTimeoutPadMs = 300;

constexpr std::uint32_t kMoveTimeoutMs = 2000;
constexpr std::uint32_t kExtensionTimeoutMs = 1000;
constexpr std::uint32_t kArmTimeoutMs = 600;
constexpr std::uint32_t kDualArmEndTimeoutMs = 500;
constexpr std::uint32_t kDualPickupDwellMs = 250;
constexpr std::uint32_t kGripMs = 250;
constexpr std::uint32_t kPollMs = 10;

constexpr auto kStagesCm = [] {
    std::array stages{constants::kExtensionFlipOutCm, constants::kExtensionStage1Cm, constants::kExtensionStage2Cm,
                      constants::kExtensionStage3Cm,  constants::kExtensionStage4Cm, constants::kExtensionStage5Cm};
    std::sort(stages.begin(), stages.end());
    return stages;
}();

std::atomic<Mode> mode{Mode::Stowed};
std::atomic<bool> tilted{false};
std::atomic<std::uint32_t> generation{0};

// A macro remembers the generation it started in; cancel() or a newer macro
// bumps the generation, and the older one stops at its next wait.
class Run {
public:
    Run() : generation_(++generation) {}

    bool active() const {
        return generation == generation_;
    }

    template <typename Done>
    bool wait_for(Done done, std::uint32_t timeout_ms) const {
        const std::uint32_t start = pros::millis();
        while (active() && !done()) {
            if (pros::millis() - start >= timeout_ms) {
                return false;
            }
            pros::delay(kPollMs);
        }
        return active();
    }

    bool pause(std::uint32_t ms) const {
        const std::uint32_t start = pros::millis();
        while (active() && pros::millis() - start < ms) {
            pros::delay(kPollMs);
        }
        return active();
    }

private:
    std::uint32_t generation_;
};

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

bool ratios_known() {
    return constants::kExtensionMotorRotationsPerCm > 0.0 && constants::kArmPivotMotorRotationsPerArmRotation > 0.0;
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

double top_stage_cm() {
    return kStagesCm.back();
}

void tilt_if_at_top() {
    if (!tilted && lift_cm() >= top_stage_cm() - kTopArrivalToleranceCm) {
        tilted = true;
        set_arm(constants::kArmTopDeg);
    }
}

void untilt_arm() {
    if (tilted) {
        tilted = false;
        set_arm(constants::kArmFlippedDeg);
    }
}

void start_flip_out(double lift, double arm) {
    mode = Mode::Deployed;
    tilted = false;
    set_arm(arm);
    set_lift(std::max(lift, constants::kExtensionFlipOutMinCm));
}

void start_home() {
    ++generation;
    mode = Mode::Stowed;
    tilted = false;
    set_arm(constants::kArmHomeDeg);
    set_lift(constants::kExtensionHomeCm);
}

bool finish_motion(const Run& run, bool auto_tilt = true) {
    if (!run.wait_for(extension::is_settled, kExtensionTimeoutMs)) {
        return false;
    }
    if (auto_tilt && mode == Mode::Deployed) {
        tilt_if_at_top();
    }
    return run.wait_for(arm_pivot::is_settled, kArmTimeoutMs);
}

bool release_if(bool reached) {
    if (!reached) {
        return false;
    }
    const Run run;
    claw::release();
    return run.pause(kGripMs);
}

}

bool go_to_pose(Pose pose) {
    const Run run;
    extension::set_target(constants::extension_cm_to_motor_deg(pose.lift_cm));
    set_arm(pose.arm_deg);
    return run.wait_for([] { return extension::is_settled() && arm_pivot::is_settled(); }, kMoveTimeoutMs);
}

// tip height = pivot start height + lift - arm length * cos(arm angle from straight down).
// Try every arm angle in range, solve the lift that reaches tip_cm, and keep the
// pose both mechanisms reach soonest. Lifts below the start are skipped. A coarse
// sweep finds the best region, then a fine sweep around it finds the fastest pose.
namespace {

bool solve_from(const Pose& from, double tip_cm, Pose& out, double arm_min_deg, double arm_max_deg) {
    if (!ratios_known()) {
        return false;
    }
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

}

bool solve(double tip_cm, Pose& out, double arm_min_deg, double arm_max_deg) {
    return solve_from(current_pose(), tip_cm, out, arm_min_deg, arm_max_deg);
}

bool move_to(double tip_cm) {
    Pose pose{};
    if (!solve(tip_cm, pose)) {
        return false;
    }
    const Run run;
    mode = Mode::Deployed;
    tilted = false;
    const double seconds = move_seconds(current_pose(), pose);
    set_lift(pose.lift_cm);
    set_arm(pose.arm_deg);
    const auto timeout_ms = static_cast<std::uint32_t>(seconds * 1000.0 * kMoveTimeoutScale) + kMoveTimeoutPadMs;
    return run.wait_for([] { return extension::is_settled() && arm_pivot::is_settled(); }, timeout_ms);
}

bool flip_out(double lift_cm, double arm_deg) {
    const Run run;
    start_flip_out(lift_cm, arm_deg);
    return finish_motion(run, arm_deg == constants::kArmFlippedDeg);
}

bool go_to_lift(double lift_cm) {
    if (lift_cm <= constants::kExtensionHomeCm) {
        return home();
    }
    if (mode == Mode::Stowed) {
        return flip_out(lift_cm);
    }
    const Run run;
    if (lift_cm < top_stage_cm()) {
        untilt_arm();
    }
    set_lift(lift_cm);
    return finish_motion(run);
}

bool go_to_stage(int stage) {
    constexpr std::array kStageLiftCm{constants::kExtensionStage1Cm, constants::kExtensionStage2Cm,
                                      constants::kExtensionStage3Cm, constants::kExtensionStage4Cm,
                                      constants::kExtensionStage5Cm};
    if (stage < 1 || stage > static_cast<int>(kStageLiftCm.size())) {
        return false;
    }
    return go_to_lift(kStageLiftCm[stage - 1]);
}

bool home() {
    start_home();
    const Run run;
    return finish_motion(run);
}

bool dual_setup() {
    const Run run;
    mode = Mode::Deployed;
    tilted = false;
    set_lift(constants::kExtensionDualSetupCm);
    if (!run.wait_for(extension::is_settled, kExtensionTimeoutMs)) {
        return false;
    }
    set_arm(constants::kArmDualSetupDeg);
    return run.wait_for(arm_pivot::is_settled, kArmTimeoutMs);
}

bool dual_pickup() {
    const Run run;
    set_arm(constants::kArmDualPickupDeg);
    if (!run.wait_for(arm_pivot::is_settled, kArmTimeoutMs) || !run.pause(kDualPickupDwellMs)) {
        return false;
    }
    set_arm(constants::kArmFlippedDeg);
    return run.wait_for(arm_pivot::is_settled, kDualArmEndTimeoutMs);
}

bool dual_pickup_height(double lift_cm) {
    const Run run;
    set_arm(constants::kArmDualPickupDeg);
    if (!run.wait_for(arm_pivot::is_settled, kArmTimeoutMs) || !run.pause(kDualPickupDwellMs)) {
        return false;
    }
    start_flip_out(lift_cm, constants::kArmFlippedDeg);
    return finish_motion(run);
}

// Prime the claw (open), wait until the distance sensor sees a piece closer than
// constants::kPickupDistanceMm, then close on it. The claw ends Grabbed; if it times
// out or is cancelled first, it stays Primed (open).
bool pickup(std::uint32_t timeout_ms) {
    const Run run;
    claw::prime();
    if (!run.wait_for([] { return claw::distance_mm() < constants::kPickupDistanceMm; }, timeout_ms)) {
        return false;
    }
    claw::grab();
    return run.pause(kGripMs);
}

double claw_offset_in(const Pose& pose) {
    const double reach_cm = constants::kArmLengthCm * std::sin(pose.arm_deg * kPi / 180.0);
    return (reach_cm - constants::kPivotBehindCenterCm + constants::kClawReleaseOffsetCm) / constants::kCmPerInch;
}

namespace {

bool score_in_range(constants::Goal goal, int level, float goal_x_in, float goal_y_in, std::uint32_t timeout_ms,
                    double arm_min_deg, double arm_max_deg) {
    if (level < 1) {
        return false;
    }
    Pose pose{};
    if (!solve(constants::release_cm(goal, level), pose, arm_min_deg, arm_max_deg)) {
        return false;
    }

    const lemlib::Pose robot = chassis.getPose();
    const double dx = goal_x_in - robot.x;
    const double dy = goal_y_in - robot.y;
    const double distance = std::hypot(dx, dy);
    const double offset = claw_offset_in(pose);
    const double standoff = std::fabs(offset);
    if (distance > 1e-6) {
        // The center stops `standoff` short of the goal on the robot-to-goal line, facing
        // the goal if the claw is in front or facing away if it is behind. Driving toward
        // the goal is forwards when facing it; backing off when too close is the reverse.
        const bool claw_in_front = offset >= 0.0;
        const bool toward_goal = distance >= standoff;
        const float stop_x = static_cast<float>(goal_x_in - dx / distance * standoff);
        const float stop_y = static_cast<float>(goal_y_in - dy / distance * standoff);
        chassis.moveToPoint(stop_x, stop_y, static_cast<int>(timeout_ms), {.forwards = claw_in_front == toward_goal});
    }

    const Run run;
    mode = Mode::Deployed;
    tilted = false;
    set_lift(pose.lift_cm);
    set_arm(pose.arm_deg);
    const bool reached = run.wait_for(
        [] { return !chassis.isInMotion() && extension::is_settled() && arm_pivot::is_settled(); }, timeout_ms);
    return release_if(reached);
}

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

bool score(constants::Goal goal, int level) {
    if (level < 1) {
        return false;
    }
    return release_if(move_to(constants::release_cm(goal, level)));
}

void cancel() {
    ++generation;
}

namespace {

constexpr int kFullPower = 127;
constexpr int kArmUpPower = 89; // 70% of full, arm only, while L1 is held

int dir = 0;          // +1 while L1 is held, -1 while L2 is held
bool lift_manual = false; // R1/R2 driving the lift by hand
int goal_tier = 0;    // 0 alliance, 1 neutral, 2 center: lifts the elevator kGoalTierStepCm per tier
int snapped_stage = -1; // stage of the last snap, so the offset can move the lift there

double tip_cm(const Pose& pose) {
    return constants::kPivotStartHeightCm + pose.lift_cm - constants::kArmLengthCm * std::cos(pose.arm_deg * kPi / 180.0);
}

double stage_tip_cm(int stage) {
    return constants::release_cm(kStageGoal, stage);
}

// Precomputed stage setpoints: stage 0 is home; each stage after it is the solver's
// fastest pose from the stage before, claw in front of the pivot.
const std::array<Pose, kTopStage + 1>& stage_poses() {
    static const auto poses = [] {
        std::array<Pose, kTopStage + 1> out{};
        out[0] = {0.0, constants::kArmStartDeg};
        for (int stage = 1; stage <= kTopStage; ++stage) {
            if (!solve_from(out[stage - 1], stage_tip_cm(stage), out[stage], kArmFrontMinDeg, kArmFrontMaxDeg)) {
                out[stage] = out[stage - 1];
            }
        }
        return out;
    }();
    return poses;
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

bool stage_pose(int stage, Pose& pose) {
    pose = stage_poses()[std::clamp(stage, 0, kTopStage)];
    return true;
}

// Release: the lift to the stage's height (slowing over its last 5 cm) and the arm to
// the stage's angle on its SmartMotor PID.
void snap(int stage) {
    Pose pose{};
    stage_pose(stage, pose);
    cancel();
    mode = Mode::Deployed;
    tilted = false;
    snapped_stage = std::clamp(stage, 0, kTopStage);
    set_lift(pose.lift_cm + (snapped_stage > 0 ? offset_cm() : 0.0));
    set_arm(pose.arm_deg);
}

}

bool go_to_level(int stage) {
    Pose pose{};
    stage_pose(stage, pose);
    return go_to_pose(pose);
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
        if (!lift_manual) {
            cancel();
            lift_manual = true;
        }
        extension::set_manual(lift_up ? kFullPower : -constants::kExtensionDownPower);
    } else if (lift_manual) {
        lift_manual = false;
        extension::hold();
    }

    if (held != 0) {
        // Hold: lift 100% up / 35% down, arm 70% up / 100% down, no control loop.
        if (held != dir) {
            cancel();
        }
        extension::set_manual(held > 0 ? kFullPower : -constants::kExtensionDownPower);
        arm_pivot::set_power(held > 0 ? kArmUpPower : -kFullPower);
    } else if (dir != 0) {
        // Release: the only place motor control runs.
        snap(closest_stage(tip_cm(current_pose())));
    }
    dir = held;

    if (controller.get_digital_new_press(controls::kHome)) {
        snap(0);
    }

    // Right cycles the goal (alliance -> neutral -> center): only the elevator moves.
    if (controller.get_digital_new_press(controls::kGoalOffset)) {
        goal_tier = (goal_tier + 1) % 3;
        if (held == 0 && snapped_stage > 0) {
            set_lift(stage_poses()[snapped_stage].lift_cm + offset_cm());
        }
    }
}

void stop() {
    dir = 0;
    lift_manual = false;
    snapped_stage = -1;
    cancel();
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
