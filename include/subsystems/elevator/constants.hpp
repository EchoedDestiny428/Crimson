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

// Lower hardstop: the arm rests on it at every power-on, so the encoder's zero is this angle,
// and homing drives into it. TODO: 25 is a placeholder until the real angle is measured.
// The pivot height and setpoints are still from the 32 deg / 29 cm solve.
inline constexpr double kArmHardstopDeg = 25.0;
inline constexpr double kArmStartDeg = kArmHardstopDeg;
inline constexpr double kClawStartHeightCm = 15.0;  // claw height above the ground
inline constexpr double kPivotStartHeightCm = 39.6; // arm pivot height above the ground (15 + 29 * cos(32))
// Slack adjustment: every scoring arm setpoint is commanded this many degrees closer to
// straight up (180) than planned: behind the pivot (angles over 180) the slack lets the claw
// sag toward more angle, so it is commanded to less angle to end where it was meant to.
inline constexpr double kArmSlackAdjustmentDeg = 3.5;
// The piece releases this far toward the robot's front from the arm tip, so behind the
// pivot only once arm length * sin(arm) < -kClawReleaseOffsetCm: arm >= 180 + asin(7 / 29)
// = 194.0 deg. Straight up (180) would drop it 7 cm in front of the pivot.
inline constexpr double kClawReleaseOffsetCm = 7.0;

// Driver scoring stages, all behind (reverse scoring): {lift cm, arm deg from straight down}.
// The arm has two hard scoring angles, mirrored about horizontal behind (270 deg), so the
// claw is the same distance behind the pivot at both (29 * sin(40) - 7 = 15.2 cm) and the
// piece releases in the same spot; the lift makes up the rest of the height.
//   L1-L2 at kArmScoreLowDeg (312): claw tip about 20 cm up with the lift down.
//   L3+ at kArmScoreHighDeg (228): claw tip about 59 cm up with the lift down.
// Lift = release height - that tip height, plus about 5 cm found on the robot (the lifts below
// are the measured ones). L1 can't go lower than the low angle's ~20 cm.
// Release heights (alliance goal): L1 17.5 cm (the first pin alone), L2 36 cm, then
// +17 cm per stage. L3 (53 cm) is about what the high angle already reaches with the lift
// down, so it uses the high angle with no lift.
inline constexpr double kArmScoreLowDeg = 312.0;
inline constexpr double kArmScoreHighDeg = 228.0;
struct Setpoint {
    double lift_cm;
    double arm_deg;
};
inline constexpr std::array<Setpoint, 8> kStageSetpoints{{
    {0.0, 30.0},               // home / intake: PID to 30 deg, then pushed onto the hardstop
    {0.0, kArmScoreLowDeg},    // L1  17.5 cm (ends ~21 cm: lift already down)
    {20.0, kArmScoreLowDeg},   // L2  36.0 cm
    {0.0, kArmScoreHighDeg},   // L3  53.0 cm (the high angle already reaches it, lift stays down)
    {16.8, kArmScoreHighDeg},  // L4  70.0 cm
    {33.8, kArmScoreHighDeg},  // L5  87.0 cm
    {50.8, kArmScoreHighDeg},  // L6 104.0 cm
    {67.8, kArmScoreHighDeg},  // L7 121.0 cm
}};
inline constexpr int kTopStage = static_cast<int>(kStageSetpoints.size()) - 1;
// Scoring (L1 release): after the claw opens, wait this long and raise the lift this far.
inline constexpr int kOuttakeRaiseDelayMs = 300;
inline constexpr double kOuttakeRaiseCm = 11.0;
// Right cycles the goal: neutral = alliance + 7 cm of lift, center = alliance + 14.
inline constexpr double kGoalTierStepCm = 7.0;

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
