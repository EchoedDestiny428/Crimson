#include "subsystems/elevator/ik.hpp"
#include <algorithm>
#include <cmath>

namespace subsystems::elevator::ik {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kLiftEpsCm = 0.3;
constexpr double kArmEpsDeg = 0.3;
constexpr double kPlaceClearanceCm = 5.0; // swing this far above the pin, then drop
constexpr double kCrossbarZoneDeg = 30.0; // matches the arm's slow zone at the 315 deg stop

double deg_to_rad(double deg) {
    return deg * kPi / 180.0;
}

Solution solution_at(double arm_deg, double z_cm) {
    Solution out;
    if (arm_deg < constants::kMinArmDeg || arm_deg > constants::kMaxArmDeg) {
        return out;
    }
    const double lift =
        z_cm - constants::kPivotStartHeightCm + constants::kArmLengthCm * std::cos(deg_to_rad(arm_deg));
    if (lift < -1e-6 || lift > constants::kMaxLiftCm + 1e-6) {
        return out;
    }
    out.valid = true;
    out.arm_deg = arm_deg;
    out.lift_cm = std::clamp(lift, 0.0, constants::kMaxLiftCm);
    out.z_cm = z_cm;
    return out;
}

}

double release_x_cm(double arm_deg) {
    return constants::kArmLengthCm * std::sin(deg_to_rad(arm_deg)) + constants::kClawReleaseOffsetCm;
}

double release_z_cm(double lift_cm, double arm_deg) {
    return constants::kPivotStartHeightCm + lift_cm -
           constants::kArmLengthCm * std::cos(deg_to_rad(arm_deg));
}

double x_from_score_range_cm(double range_cm) {
    return release_x_cm(constants::kArmScoreLowDeg) - (range_cm - constants::kScoreRangeAtRammedCm);
}

Pair solve(double x_cm, double z_cm) {
    Pair pair;
    if (!std::isfinite(x_cm) || !std::isfinite(z_cm) || constants::kArmLengthCm <= 0.0) {
        return pair;
    }
    const double s = (x_cm - constants::kClawReleaseOffsetCm) / constants::kArmLengthCm;
    if (s < -1.0 || s > 1.0) {
        return pair;
    }
    const double asin_deg = std::asin(s) * kRadToDeg;
    // asin is in [-90, 90]. Wrapped into [0, 360) that is one root (310° at the
    // design distance). The other root is its mirror through 270°, which is 540° − θ
    // (230°). Same sin, so the same horizontal reach.
    const double theta = asin_deg >= 0.0 ? asin_deg : asin_deg + 360.0;
    const double mirror = 540.0 - theta;
    const auto store = [&](double arm_deg) {
        if (arm_deg > 270.0) {
            pair.low = solution_at(arm_deg, z_cm);
        } else {
            pair.high = solution_at(arm_deg, z_cm);
        }
    };
    store(theta);
    if (std::fabs(mirror - theta) > 0.05) {
        store(mirror);
    }
    return pair;
}

Solution choose(const Pair& pair, double from_lift_cm, double from_arm_deg) {
    Solution best;
    if (!std::isfinite(from_lift_cm) || !std::isfinite(from_arm_deg)) {
        return best;
    }
    double best_travel = 0.0;
    const auto consider = [&](const Solution& candidate) {
        if (!candidate.valid) {
            return;
        }
        const double travel = std::fabs(candidate.lift_cm - from_lift_cm);
        const bool closer_lift = !best.valid || travel < best_travel - 1e-9;
        const bool same_lift = best.valid && std::fabs(travel - best_travel) <= 1e-9 &&
                               std::fabs(candidate.arm_deg - from_arm_deg) < std::fabs(best.arm_deg - from_arm_deg);
        if (closer_lift || same_lift) {
            best = candidate;
            best_travel = travel;
        }
    };
    consider(pair.high);
    consider(pair.low);
    return best;
}

Motion plan(double from_lift_cm, double from_arm_deg, double x_cm, double z_cm) {
    Motion motion;
    if (!std::isfinite(from_lift_cm) || !std::isfinite(from_arm_deg)) {
        return motion;
    }
    motion.goal = choose(solve(x_cm, z_cm), from_lift_cm, from_arm_deg);
    if (!motion.goal.valid) {
        return motion;
    }
    motion.valid = true;

    const bool arm_moves = std::fabs(from_arm_deg - motion.goal.arm_deg) >= kArmEpsDeg;
    const bool at_crossbar = motion.goal.arm_deg >= constants::kMaxArmDeg - kCrossbarZoneDeg;
    const double transit =
        arm_moves && !at_crossbar ? std::max(from_lift_cm, motion.goal.lift_cm + kPlaceClearanceCm)
                                  : std::max(from_lift_cm, motion.goal.lift_cm);
    const auto add = [&](double lift, double arm) {
        if (motion.count >= static_cast<int>(motion.steps.size())) {
            return;
        }
        if (motion.count == 0) {
            if (std::fabs(from_lift_cm - lift) < kLiftEpsCm && std::fabs(from_arm_deg - arm) < kArmEpsDeg) {
                return;
            }
        } else {
            const Step& prev = motion.steps[static_cast<std::size_t>(motion.count - 1)];
            if (std::fabs(prev.lift_cm - lift) < kLiftEpsCm && std::fabs(prev.arm_deg - arm) < kArmEpsDeg) {
                return;
            }
        }
        motion.steps[static_cast<std::size_t>(motion.count++)] = Step{lift, arm};
    };
    add(transit, from_arm_deg);
    add(transit, motion.goal.arm_deg);
    add(motion.goal.lift_cm, motion.goal.arm_deg);
    if (motion.count == 0) {
        motion.steps[0] = Step{motion.goal.lift_cm, motion.goal.arm_deg};
        motion.count = 1;
    }
    return motion;
}

}
