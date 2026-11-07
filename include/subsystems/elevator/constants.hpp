#pragma once

// Shared by the elevator extension and arm pivot.

#include <array>
#include <cmath>

namespace subsystems::elevator::constants {

// Geometry and gearing.
inline constexpr double kArmLengthCm = 31.0;                         // arm pivot axis to claw
inline constexpr double kArmPivotMotorRotationsPerArmRotation = 1.0; // direct drive
inline constexpr double kExtensionMotorRotationsPerCm = 1.0 / 6.0;   // string lift: one motor rotation raises it 6 cm

// Commanded speeds (motor rpm).
inline constexpr int kExtensionMaxRpm = 600;
inline constexpr int kExtensionDownPower = 44; // 35% of 127: the lift never comes down faster than this
inline constexpr int kArmPivotMoveRpm = 100; // red cartridge

// Starting pose (robot at power-on, before anything moves).
// Pivot height fitted to measured claw heights (27 / 41.8 / 56.6 cm at arm 68 / 94.2 / 122.2 deg
// on the old 25.4 deg start), within 0.5 cm. Tape measure said 42.
inline constexpr double kPivotStartHeightCm = 45.9; // arm pivot height above the ground
inline constexpr double kClawStartHeightCm = 15.0;  // claw height above the ground
// Slack adjustment: every arm setpoint is commanded this much higher than planned, to take
// up slack in the arm so it ends where it was meant to.
inline constexpr double kArmSlackAdjustmentDeg = 2.0;
// Calculated from the measurements: acos((pivot height - claw height) / arm length),
// acos((45.9 - 15) / 31) = 4.6 deg off straight down. Change a length and this follows.
inline constexpr double kArmStartDeg =
    std::acos((kPivotStartHeightCm - kClawStartHeightCm) / kArmLengthCm) * 180.0 / 3.14159265358979323846;

// Arm-tip heights above the ground for releasing onto a goal.
// Level 1 is the first pin, scored alone into the goal. Every level after that is a
// cup/pin combo scored at once.
// Measured on the alliance goal: 17.5, 34.5, 51.5, 68.5 cm for levels 1-4. Levels past
// 4 repeat the last measured step (+17 cm), which is not measured.
// Each taller goal tier adds kGoalTierStepCm: neutral = alliance + 7, center = alliance + 14.
inline constexpr std::array kAllianceReleaseCm{17.5, 34.5, 51.5, 68.5};
inline constexpr double kGoalTierStepCm = 7.0;

enum class Goal { Alliance, Neutral, Center };

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

// Unit conversions.
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
