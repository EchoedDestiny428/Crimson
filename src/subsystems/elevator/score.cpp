#include "subsystems/elevator/score.hpp"
#include "pros/error.h"
#include "pros/rtos.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator/arm_pivot.hpp"
#include "subsystems/elevator/constants.hpp"
#include "subsystems/elevator/extension.hpp"
#include "subsystems/elevator/ik.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

namespace subsystems::elevator::score {

namespace {

constexpr std::uint32_t kPollMs = 10;
constexpr double kSlowZoneDeg = 30.0; // arm_pivot caps the last 30 deg at 35%
constexpr double kSlackArmEpsDeg = 0.01;

bool pose_now(double& lift_cm, double& arm_deg) {
    lift_cm = constants::extension_motor_deg_to_cm(extension::height());
    arm_deg = constants::arm_motor_deg_to_deg(arm_pivot::get_pos());
    return std::isfinite(lift_cm) && std::isfinite(arm_deg);
}

bool near_stop(double arm_deg) {
    return arm_deg <= constants::kMinArmDeg + kSlowZoneDeg || arm_deg >= constants::kMaxArmDeg - kSlowZoneDeg;
}

void command(const ik::Step& step, bool slack) {
    const double arm = step.arm_deg + (slack ? constants::kArmSlackAdjustmentDeg : 0.0);
    extension::set_target(constants::extension_cm_to_motor_deg(step.lift_cm));
    arm_pivot::move_to(constants::arm_deg_to_motor_deg(arm));
}

bool wait_both(std::uint32_t deadline) {
    while (static_cast<std::int32_t>(deadline - pros::millis()) > 0) {
        if (extension::settled() && arm_pivot::settled()) {
            return true;
        }
        pros::delay(kPollMs);
    }
    return extension::settled() && arm_pivot::settled();
}

std::optional<double> read_range_cm() {
    const std::int32_t mm = score_distance.get();
    // 9999 is the sensor's "nothing there" reading, not a real 10 m.
    if (mm == PROS_ERR || mm <= 0 || mm >= 9999) {
        return std::nullopt;
    }
    return static_cast<double>(mm) / 10.0;
}

}

double stage_z_cm(int stage, int tier) {
    if (stage < 1 || stage > constants::kTopStage || tier < 0 || tier > 2) {
        return std::nan("");
    }
    return constants::kStageReleaseCm[static_cast<std::size_t>(stage)] + tier * constants::kGoalTierStepCm;
}

bool go(double z_cm, double range_cm, std::uint32_t timeout_ms) {
    double lift_cm = 0.0;
    double arm_deg = 0.0;
    if (!pose_now(lift_cm, arm_deg) || !std::isfinite(z_cm) || !std::isfinite(range_cm)) {
        return false;
    }
    const double x_cm = ik::x_from_score_range_cm(range_cm);
    const ik::Motion motion = ik::plan(lift_cm, arm_deg, x_cm, z_cm);
    if (!motion.valid || ik::release_x_cm(motion.goal.arm_deg) >= 0.0) {
        return false;
    }

    arm_pivot::set_slow_zone(near_stop(motion.goal.arm_deg));
    const std::uint32_t deadline = pros::millis() + timeout_ms;
    for (int i = 0; i < motion.count; ++i) {
        const ik::Step& step = motion.steps[static_cast<std::size_t>(i)];
        const bool slack = std::fabs(step.arm_deg - motion.goal.arm_deg) < kSlackArmEpsDeg;
        command(step, slack);
        if (!wait_both(deadline)) {
            return false;
        }
    }
    return true;
}

bool go_measured(double z_cm, std::uint32_t timeout_ms) {
    const std::optional<double> range_cm = read_range_cm();
    if (!range_cm) {
        return false;
    }
    return go(z_cm, *range_cm, timeout_ms);
}

bool go_stage(int stage, int tier, double range_cm, std::uint32_t timeout_ms) {
    const double z_cm = stage_z_cm(stage, tier);
    if (!std::isfinite(z_cm)) {
        return false;
    }
    return go(z_cm, range_cm, timeout_ms);
}

bool go_stage_measured(int stage, int tier, std::uint32_t timeout_ms) {
    const double z_cm = stage_z_cm(stage, tier);
    if (!std::isfinite(z_cm)) {
        return false;
    }
    return go_measured(z_cm, timeout_ms);
}

}
