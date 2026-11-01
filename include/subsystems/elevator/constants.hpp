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

// THEORETICAL arm-tip heights above the ground for releasing a piece, with the piece
// held at the middle of the grip window. Calculated from the manual, not measured on
// the robot. Stacks nest goal -> pin -> cup -> pin, each half inside the next piece.
inline constexpr double kReleaseClearanceCm = 2.0; // guess

constexpr double place_pin_on_goal_cm(double goal_cm) {
    return goal_cm + kReleaseClearanceCm + kPinHeightCm / 2.0 + kClawGripCenterBelowTipCm;
}

constexpr double place_cup_on_pin_cm(double goal_cm) {
    return goal_cm + kPinHeightCm / 2.0 + kReleaseClearanceCm + kCupHeightCm / 2.0 + kClawGripCenterBelowTipCm;
}

constexpr double place_pin_on_cup_cm(double goal_cm) {
    return goal_cm + kCupHeightCm + kReleaseClearanceCm + kPinHeightCm / 2.0 + kClawGripCenterBelowTipCm;
}

inline constexpr double kClawPinAllianceCm = place_pin_on_goal_cm(kAllianceGoalHeightCm); // 21.5
inline constexpr double kClawCupAllianceCm = place_cup_on_pin_cm(kAllianceGoalHeightCm);  // 29.7
inline constexpr double kClawPinShortCm = place_pin_on_goal_cm(kShortGoalHeightCm);       // 27.9
inline constexpr double kClawCupShortCm = place_cup_on_pin_cm(kShortGoalHeightCm);        // 36.1
inline constexpr double kClawPinTallCm = place_pin_on_goal_cm(kTallGoalHeightCm);         // 35.5
inline constexpr double kClawCupTallCm = place_cup_on_pin_cm(kTallGoalHeightCm);          // 43.7

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
