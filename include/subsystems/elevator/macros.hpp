#pragma once

#include "subsystems/elevator/constants.hpp"

namespace subsystems::elevator::macros {

// Elevator lift (cm above the start) and arm angle (degrees from straight down).
struct Pose {
    double lift_cm;
    double arm_deg;
};

// Arm angles are degrees from straight down, counted continuously as the arm swings
// up: 0 to 180 puts the claw in front of the pivot (180 is straight up), and 180 to
// 360 carries it over the top to behind. The stage solver keeps the arm at least
// kArmMinFromDownDeg from straight down. Manual control is not limited by this.
// 20 so stage 1 (17.5 cm) is reachable with the lift down: it needs 23.6 deg off down.
inline constexpr double kArmMinFromDownDeg = 20.0;
inline constexpr double kArmFrontMinDeg = kArmMinFromDownDeg;
inline constexpr double kArmFrontMaxDeg = 180.0;
inline constexpr double kArmBehindMinDeg = 180.0;
inline constexpr double kArmBehindMaxDeg = 360.0 - kArmMinFromDownDeg;

// Stages are the scoring levels on kStageGoal: stage n puts the arm tip at
// release_cm(kStageGoal, n) using the solver; stage 0 is the starting pose.
inline constexpr constants::Goal kStageGoal = constants::Goal::Alliance;
inline constexpr int kTopStage = static_cast<int>(constants::kAllianceReleaseCm.size()); // measured levels

// Driver control: hold L1/L2 to run the lift and arm up/down (no control loop); release
// to snap both to the stage closest to the piece's actual height. B goes home. Y swaps
// front/back scoring. Right cycles the goal (alliance, neutral, center), moving only the
// elevator 7 cm per goal. R1/R2 drive the lift by hand (100% up, 35% down) and hold where
// let go, no snap.
void update();
void stop();

// Live readings for the controller readout: lift (cm), arm (deg from straight down),
// piece height (cm) from those, and the stage a release would snap to.
Pose pose_now();
double tip_now();
int snap_stage_now();
int goal_tier_now(); // 0 alliance, 1 neutral, 2 center
bool behind_now();   // side swap: true when snaps score behind
Pose target_now();   // lift and arm setpoints currently commanded
int target_stage();  // stage of the last snap, -1 before the first

}
