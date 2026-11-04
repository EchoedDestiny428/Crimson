#pragma once

// Shared by the elevator extension and arm pivot.
// Every value marked TODO still has to be measured on the new robot.

#include <array>
#include <cstdint>

namespace subsystems::elevator::constants {

// Geometry and gearing.
inline constexpr double kArmLengthCm = 32.0;                         // arm pivot axis to claw
inline constexpr double kArmPivotMotorRotationsPerArmRotation = 1.0; // direct drive
inline constexpr double kExtensionMotorRotationsPerCm = 1.0 / 6.0;   // string lift: one motor rotation raises it 6 cm

// Commanded speeds (motor rpm).
inline constexpr int kExtensionMaxRpm = 600;
inline constexpr int kArmPivotMoveRpm = 100; // red cartridge

// Starting pose (robot at power-on, before anything moves).
inline constexpr double kPivotStartHeightCm = 43.0; // arm pivot height above the ground
inline constexpr double kClawStartHeightCm = 18.0;  // claw height above the ground
inline constexpr double kPivotBehindCenterCm = 3.5; // elevator pivot shaft, behind the robot's center
// The piece releases this far toward the robot's front from the arm tip: 7 cm farther
// out when the claw is in front, 7 cm closer when it is behind.
inline constexpr double kClawReleaseOffsetCm = 7.0;
inline constexpr double kCmPerInch = 2.54;
// Measured on the robot: the arm starts 32 deg off straight down. (acos((43 - 18) / 32)
// gives 38.6, so one of those three lengths is a little off.)
inline constexpr double kArmStartDeg = 32.0;

// Elevator extension setpoints, in cm of lift.
inline constexpr double kExtensionBootupCm = 0.0;     // TODO
inline constexpr double kExtensionFlipOutMinCm = 0.0; // TODO
inline constexpr double kExtensionHomeCm = 0.0;       // TODO
inline constexpr double kExtensionDualSetupCm = 0.0;  // TODO
inline constexpr double kExtensionOffsetCm = 0.0;     // TODO
inline constexpr double kExtensionFlipOutCm = 0.0;    // TODO
inline constexpr double kExtensionStage1Cm = 0.0;     // TODO
inline constexpr double kExtensionStage2Cm = 0.0;     // TODO
inline constexpr double kExtensionStage3Cm = 0.0;     // TODO
inline constexpr double kExtensionStage4Cm = 0.0;     // TODO
inline constexpr double kExtensionStage5Cm = 0.0;     // TODO

// Arm pivot setpoints, in degrees from straight down (positive toward the side the arm starts on).
inline constexpr double kArmHomeDeg = 0.0;       // TODO
inline constexpr double kArmFlippedDeg = 0.0;    // TODO
inline constexpr double kArmTopDeg = 0.0;        // TODO
inline constexpr double kArmDualSetupDeg = 0.0;  // TODO
inline constexpr double kArmDualPickupDeg = 0.0; // TODO

// Override (V5RC 2026-27) field elements, from game manual v2.1, in cm.
inline constexpr double kPinHeightCm = 16.5;
inline constexpr double kCupHeightCm = 16.45;
inline constexpr double kAllianceGoalHeightCm = 8.25;
inline constexpr double kNeutralGoalHeightCm = 14.65; // neutral quadrant goals ("short" in the manual)
inline constexpr double kCenterGoalHeightCm = 22.27;  // center goal ("tall" in the manual)

// Claw grip window relative to the arm tip (measured): it grabs from 8 cm below to 2 cm
// above the tip, tuned so at its lowest it grabs a cup/pin combo off the floor.
inline constexpr double kClawGripAboveCm = 2.0;
inline constexpr double kClawGripBelowCm = 8.0;
inline constexpr double kClawGripCenterBelowTipCm = (kClawGripBelowCm - kClawGripAboveCm) / 2.0; // 3 cm
inline constexpr double kClawLowestCm = kPivotStartHeightCm - kArmLengthCm; // arm tip, arm straight down, no lift

// Arm-tip heights above the ground for releasing onto a goal.
// Level 1 is the first pin, scored alone into the goal. Every level after that is a
// cup/pin combo scored at once.
// Measured on the alliance goal: 17.5, 34.5, 51.5, 68.5 cm for levels 1-4. Levels past
// 4 repeat the last measured step (+17 cm), which is not measured.
// Each taller goal tier adds kGoalTierStepCm: neutral = alliance + 7, center = alliance + 14.
inline constexpr std::array kAllianceReleaseCm{17.5, 34.5, 51.5, 68.5};
inline constexpr double kGoalTierStepCm = 7.0;

enum class Goal { Alliance, Neutral, Center }; // 8.25, 14.65 and 22.27 cm tall

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
        const double step = kAllianceReleaseCm[measured - 1] - kAllianceReleaseCm[measured - 2];
        alliance = kAllianceReleaseCm[measured - 1] + (level - measured) * step;
    }
    return alliance + goal_tier(goal) * kGoalTierStepCm;
}
// Alliance: 17.5, 34.5, 51.5, 68.5 cm  Neutral: 24.5, 41.5, 58.5, 75.5  Center: 31.5, 48.5, 65.5, 82.5

// Pickup: the claw closes once the distance sensor reads closer than this.
inline constexpr std::int32_t kPickupDistanceMm = 50; // TODO: guess, tune on the robot


// Unit conversions. These divide by the ratios above, so fill those in first.
constexpr double extension_cm_to_motor_deg(double cm) {
    return cm * kExtensionMotorRotationsPerCm * 360.0;
}

constexpr double extension_motor_deg_to_cm(double motor_deg) {
    return motor_deg / 360.0 / kExtensionMotorRotationsPerCm;
}

// The arm pivot motor is tared at startup, so motor 0 is the starting angle (kArmStartDeg).
constexpr double arm_deg_to_motor_deg(double arm_deg) {
    return (arm_deg - kArmStartDeg) * kArmPivotMotorRotationsPerArmRotation;
}

constexpr double arm_motor_deg_to_deg(double motor_deg) {
    return motor_deg / kArmPivotMotorRotationsPerArmRotation + kArmStartDeg;
}

}
