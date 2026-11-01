#pragma once

// Shared by the elevator extension, arm pivot and claw pivot.
// Every value marked TODO still has to be measured on the new robot.

namespace subsystems::elevator::constants {

// Geometry and gearing.
inline constexpr double kArmLengthCm = 32.0;                         // arm pivot axis to claw
inline constexpr double kArmPivotMotorRotationsPerArmRotation = 1.0; // direct drive
inline constexpr double kExtensionMotorRotationsPerCm = 0.0;         // TODO: extension motor rotations per cm of lift

// Starting pose (robot at power-on, before anything moves).
inline constexpr double kPivotStartHeightCm = 43.0; // arm pivot height above the ground
inline constexpr double kClawStartHeightCm = 14.0;  // claw height above the ground
// acos((43 - 14) / 32): the arm starts 25.0 deg off straight down, claw 13.5 cm out from the pivot.
inline constexpr double kArmStartDeg = 25.0078;

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
inline constexpr double kShortGoalHeightCm = 14.65; // neutral quadrant goals
inline constexpr double kTallGoalHeightCm = 22.27;  // center goal

// Claw grip window relative to the arm tip (measured): it grabs from 8 cm below to 2 cm
// above the tip, tuned so at its lowest it grabs a cup/pin combo off the floor.
inline constexpr double kClawGripAboveCm = 2.0;
inline constexpr double kClawGripBelowCm = 8.0;
inline constexpr double kClawGripCenterBelowTipCm = (kClawGripBelowCm - kClawGripAboveCm) / 2.0; // 3 cm
inline constexpr double kClawLowestCm = kPivotStartHeightCm - kArmLengthCm; // arm tip, arm straight down, no lift

// THEORETICAL arm-tip heights above the ground for releasing onto a goal. Calculated
// from the manual, not measured on the robot.
// Level 1 is the first pin, scored alone into the goal. Every level after that is a
// cup/pin combo scored at once: the cup slides over the pin below it and carries the
// next pin, so each combo raises the stack by one cup height.
// A held piece sits with its bottom kClawLowestCm below the arm tip: at its lowest the
// arm tip is that high and grabs a combo standing on the floor.
inline constexpr double kReleaseClearanceCm = 2.0; // guess
inline constexpr double kHeldBottomBelowTipCm = kClawLowestCm;

enum class Goal { Alliance, Short, Tall };

constexpr double goal_height_cm(Goal goal) {
    switch (goal) {
    case Goal::Alliance:
        return kAllianceGoalHeightCm;
    case Goal::Short:
        return kShortGoalHeightCm;
    case Goal::Tall:
        return kTallGoalHeightCm;
    }
    return kAllianceGoalHeightCm;
}

// Height of the top of the stack before scoring `level`.
constexpr double stack_top_cm(double goal_cm, int level) {
    return level <= 1 ? goal_cm : goal_cm + kPinHeightCm / 2.0 + (level - 2) * kCupHeightCm;
}

constexpr double stack_release_cm(double goal_cm, int level) {
    return stack_top_cm(goal_cm, level) + kReleaseClearanceCm + kHeldBottomBelowTipCm;
}
// Alliance: 21.2, 29.5, 46.0, 62.4 cm  Short: 27.6, 35.9, 52.3, 68.8  Tall: 35.3, 43.5, 60.0, 76.4

// Claw pivot, in motor degrees until its gearing is known.
inline constexpr double kClawPivotHomeMotorDeg = 0.0; // TODO

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
