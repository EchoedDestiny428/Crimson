#pragma once

// Shared by the elevator extension and arm pivot.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

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
    {3.0, kArmScoreLowDeg},    // L1  20.5 cm (lift 3 cm up; was 0 / 17.5 cm, too low)
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
// Clamping the claw (L2 or X) raises the lift to at least this height, kClampLiftDelayMs after
// the clamp, so the cup doesn't drag on the floor. It doesn't change the selected stage.
inline constexpr double kClampLiftCm = 9.0;
inline constexpr int kClampLiftDelayMs = 200;
// Right cycles the goal: neutral = alliance + 7 cm of lift, center = alliance + 14.
inline constexpr double kGoalTierStepCm = 7.0;

// Auton scoring (chaos planner and the score_* macros). Those solve a lift + arm pose for a
// release height instead of using the driver's hard setpoints.
inline constexpr double kCmPerInch = 2.54;
inline constexpr double kPivotBehindCenterCm = 3.5; // elevator pivot shaft, behind the robot's center
inline constexpr double kPinHeightCm = 16.5;        // one pin added to a stack
inline constexpr double kAllianceGoalHeightCm = 8.25;
inline constexpr double kNeutralGoalHeightCm = 14.65; // neutral quadrant goals ("short" in the manual)
inline constexpr double kCenterGoalHeightCm = 22.27;  // center goal ("tall" in the manual)
// Alliance goal release heights per level, the same ones the driver stages above aim for.
inline constexpr std::array kAllianceReleaseCm{17.5, 36.0, 53.0, 70.0, 87.0, 104.0, 121.0};

enum class Goal { Alliance, Neutral, Center }; // 8.25, 14.65 and 22.27 cm tall

constexpr double goal_height_cm(Goal goal) {
    switch (goal) {
    case Goal::Neutral:
        return kNeutralGoalHeightCm;
    case Goal::Center:
        return kCenterGoalHeightCm;
    default:
        return kAllianceGoalHeightCm;
    }
}

// Release height for the level-th piece on a goal: the alliance table plus kGoalTierStepCm
// per tier, extrapolated past the last measured level.
constexpr double release_cm(Goal goal, int level) {
    const int measured = static_cast<int>(kAllianceReleaseCm.size());
    level = std::max(level, 1);
    const double step = kAllianceReleaseCm[measured - 1] - kAllianceReleaseCm[measured - 2];
    const double alliance = level <= measured ? kAllianceReleaseCm[level - 1]
                                              : kAllianceReleaseCm[measured - 1] + (level - measured) * step;
    return alliance + static_cast<int>(goal) * kGoalTierStepCm;
}

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
