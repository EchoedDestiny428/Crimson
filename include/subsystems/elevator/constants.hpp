#pragma once

#include <array>
#include <cmath>
#include <cstdint>

namespace subsystems::elevator::constants {

inline constexpr double kArmLengthCm = 31.0;
inline constexpr double kArmPivotMotorRotationsPerArmRotation = 1.0;
inline constexpr double kExtensionMotorRotationsPerCm = 1.0 / 6.0;

inline constexpr int kExtensionMaxRpm = 600;
inline constexpr int kExtensionDownPower = 44;
inline constexpr int kArmPivotMoveRpm = 100;

inline constexpr double kPivotStartHeightCm = 45.9;
inline constexpr double kClawStartHeightCm = 15.0;
inline constexpr double kPivotBehindCenterCm = 3.5;
inline constexpr double kClawReleaseOffsetCm = 7.0;
inline constexpr double kCmPerInch = 2.54;
inline constexpr double kArmSlackAdjustmentDeg = 2.0;
inline constexpr double kArmStartDeg =
    std::acos((kPivotStartHeightCm - kClawStartHeightCm) / kArmLengthCm) * 180.0 / 3.14159265358979323846;

inline constexpr double kExtensionBootupCm = 0.0; // placeholder
inline constexpr double kExtensionFlipOutMinCm = 0.0; // placeholder
inline constexpr double kExtensionHomeCm = 0.0; // placeholder
inline constexpr double kExtensionDualSetupCm = 0.0; // placeholder
inline constexpr double kExtensionOffsetCm = 0.0; // placeholder
inline constexpr double kExtensionFlipOutCm = 0.0; // placeholder
inline constexpr double kExtensionStage1Cm = 0.0; // placeholder
inline constexpr double kExtensionStage2Cm = 0.0; // placeholder
inline constexpr double kExtensionStage3Cm = 0.0; // placeholder
inline constexpr double kExtensionStage4Cm = 0.0; // placeholder
inline constexpr double kExtensionStage5Cm = 0.0; // placeholder

inline constexpr double kArmHomeDeg = 0.0; // placeholder
inline constexpr double kArmFlippedDeg = 0.0; // placeholder
inline constexpr double kArmTopDeg = 0.0; // placeholder
inline constexpr double kArmDualSetupDeg = 0.0; // placeholder
inline constexpr double kArmDualPickupDeg = 0.0; // placeholder

inline constexpr double kPinHeightCm = 16.5;
inline constexpr double kCupHeightCm = 16.45;
inline constexpr double kAllianceGoalHeightCm = 8.25;
inline constexpr double kNeutralGoalHeightCm = 14.65;
inline constexpr double kCenterGoalHeightCm = 22.27;

inline constexpr double kClawGripAboveCm = 2.0;
inline constexpr double kClawGripBelowCm = 8.0;
inline constexpr double kClawGripCenterBelowTipCm = (kClawGripBelowCm - kClawGripAboveCm) / 2.0;
inline constexpr double kClawLowestCm = kPivotStartHeightCm - kArmLengthCm;

inline constexpr std::array kAllianceReleaseCm{17.5, 34.5, 51.5, 68.5};
inline constexpr double kGoalTierStepCm = 7.0;

enum class Goal { Alliance, Neutral, Center };

constexpr double goal_height_cm(Goal goal) {
    switch (goal) {
    case Goal::Alliance:
        return kAllianceGoalHeightCm;
    case Goal::Neutral:
        return kNeutralGoalHeightCm;
    case Goal::Center:
        return kCenterGoalHeightCm;
    }
    return kAllianceGoalHeightCm;
}

constexpr int goal_tier(Goal goal) {
    switch (goal) {
    case Goal::Alliance:
        return 0;
    case Goal::Neutral:
        return 1;
    case Goal::Center:
        return 2;
    }
    return 0;
}

constexpr double release_cm(Goal goal, int level) {
    const int measured = static_cast<int>(kAllianceReleaseCm.size());
    if (level < 1) {
        level = 1;
    }
    double alliance = 0.0;
    if (level <= measured) {
        alliance = kAllianceReleaseCm[level - 1];
    } else {
        const double step = kAllianceReleaseCm[measured - 1] - kAllianceReleaseCm[measured - 2]; // unsure
        alliance = kAllianceReleaseCm[measured - 1] + (level - measured) * step;
    }
    return alliance + goal_tier(goal) * kGoalTierStepCm;
}

inline constexpr std::int32_t kPickupDistanceMm = 50; // placeholder


constexpr double extension_cm_to_motor_deg(double cm) {
    return cm * kExtensionMotorRotationsPerCm * 360.0;
}

constexpr double extension_motor_deg_to_cm(double motor_deg) {
    return motor_deg / 360.0 / kExtensionMotorRotationsPerCm;
}

constexpr double arm_deg_to_motor_deg(double arm_deg) {
    return (arm_deg - kArmStartDeg) * kArmPivotMotorRotationsPerArmRotation;
}

constexpr double arm_motor_deg_to_deg(double motor_deg) {
    return motor_deg / kArmPivotMotorRotationsPerArmRotation + kArmStartDeg;
}

}
