#pragma once

// Shared by the elevator extension and arm pivot.

#include <array>
#include <cmath>

namespace subsystems::elevator::constants {

// Geometry and gearing.
inline constexpr double kArmLengthCm = 29.0;                         // arm pivot axis to claw
inline constexpr double kArmPivotMotorRotationsPerArmRotation = 1.0; // direct drive
inline constexpr double kExtensionMotorRotationsPerCm = (1100.0 / 360.0) / 33.0; // measured: 1100 motor deg raises it 33 cm (10.8 cm/rev)

// Commanded speeds (motor rpm).
inline constexpr int kExtensionMaxRpm = 600;
inline constexpr int kExtensionDownPower = 89; // 70% of 127: the lift never comes down faster than this

// Starting pose: the arm is set at 90 deg (horizontal) before every power-on, so the
// encoder's zero is 90 deg. The pivot height and setpoints are still from the 32 deg /
// 29 cm solve.
inline constexpr double kArmStartDeg = 90.0;
inline constexpr double kClawStartHeightCm = 15.0;  // claw height above the ground
inline constexpr double kPivotStartHeightCm = 39.6; // arm pivot height above the ground (15 + 29 * cos(32))
// Slack adjustment: every arm setpoint is commanded this much higher than planned, to take
// up slack in the arm so it ends where it was meant to.
inline constexpr double kArmSlackAdjustmentDeg = 2.0;
// The piece releases this far toward the robot's front from the arm tip, so behind the
// pivot only once arm length * sin(arm) < -kClawReleaseOffsetCm: arm >= 180 + asin(7 / 29)
// = 194.0 deg. Straight up (180) would drop it 7 cm in front of the pivot.
inline constexpr double kClawReleaseOffsetCm = 7.0;
// Soft limits, degrees from straight down. 315 is the rear crossbar.
inline constexpr double kMinArmDeg = 35.0;
inline constexpr double kMaxArmDeg = 315.0;
// Scoring-side distance sensor, facing back. Not installed. A reading equal to this is
// the rammed pose (release at the 230/310 point). 0 until the mount is measured, so a
// 0 cm reading is treated as flush and any larger reading as that many cm short of the goal.
inline constexpr double kScoreRangeAtRammedCm = 0.0;

// Driver scoring stages, all behind (reverse scoring): {lift cm, arm deg from straight down}.
// The arm has two hard scoring angles, mirrored about horizontal behind (270 deg), so the
// claw is the same distance behind the pivot at both (29 * sin(40) - 7 = 15.2 cm) and the
// piece releases in the same spot; the lift makes up the rest of the height.
//   L1-L3 at kArmScoreLowDeg (310): tip 39.6 - 29 * cos(310) = 21.0 cm with the lift down.
//   L4+ at kArmScoreHighDeg (230): tip 39.6 - 29 * cos(230) = 58.2 cm with the lift down.
// Lift = release height - that tip height. L1 can't go lower than 21 cm.
// Release heights (alliance goal): L1 17.5 cm (the first pin alone), L2 36 cm, then
// +17 cm per stage. L3 (53 cm) is below the high angle's 58.2 cm, so it stays low; the
// arm flips to the high angle from L4.
inline constexpr double kArmScoreLowDeg = 310.0;
inline constexpr double kArmScoreHighDeg = 230.0;
struct Setpoint {
    double lift_cm;
    double arm_deg;
};
inline constexpr std::array<Setpoint, 8> kStageSetpoints{{
    {0.0, 35.0},               // home / intake: the 35 deg soft limit
    {0.0, kArmScoreLowDeg},    // L1  17.5 cm (ends ~21 cm: lift already down)
    {15.0, kArmScoreLowDeg},   // L2  36.0 cm
    {32.0, kArmScoreLowDeg},   // L3  53.0 cm
    {11.8, kArmScoreHighDeg},  // L4  70.0 cm
    {28.8, kArmScoreHighDeg},  // L5  87.0 cm
    {45.8, kArmScoreHighDeg},  // L6 104.0 cm
    {62.8, kArmScoreHighDeg},  // L7 121.0 cm
}};
inline constexpr int kTopStage = static_cast<int>(kStageSetpoints.size()) - 1;
// Pin height above the ground on an alliance goal. Index matches kStageSetpoints.
// L1 (17.5) is below the arm at the design distance (the lift would have to go
// negative), so the IK refuses it. Neutral and center add kGoalTierStepCm on top.
inline constexpr std::array<double, 8> kStageReleaseCm{{
    0.0, 17.5, 36.0, 53.0, 70.0, 87.0, 104.0, 121.0,
}};
// Right cycles the goal: neutral = alliance + 7 cm of lift, center = alliance + 14.
inline constexpr double kGoalTierStepCm = 7.0;
// Highest lift the IK will command: L7 on the center goal. The extension motor still
// has no software stop; this only rejects a solved pose.
inline constexpr double kMaxLiftCm = kStageSetpoints[kTopStage].lift_cm + 2.0 * kGoalTierStepCm;

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
