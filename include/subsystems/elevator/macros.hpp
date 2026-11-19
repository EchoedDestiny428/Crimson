#pragma once

#include "subsystems/elevator/constants.hpp"

namespace subsystems::elevator::macros {

// Elevator lift (cm above the start) and arm angle (degrees from straight down).
struct Pose {
    double lift_cm;
    double arm_deg;
};

// Driver control, always scoring behind. Up/Down select a stage (1-7, printed on the
// controller). Hold L1 to send the lift and arm to it (hard setpoints,
// constants::kStageSetpoints); release L1 to score: the claw opens, 300 ms later the lift
// goes up 11 cm, then everything goes home. B goes home (the intake pose) and cancels a
// score. Right cycles the goal (alliance, neutral, center), moving only the lift 7 cm per
// goal. R1/R2 drive the lift by hand (100% up, 70% down) and hold where let go.
void update();
void stop();

// Fastest-pose solver for autons (not used by driver control): the lift + arm angle in
// [arm_min_deg, arm_max_deg] that puts the arm tip at tip_cm and is reached soonest from
// `from`. False if no pose in range reaches it with the lift at or above 0.
bool solve_from(const Pose& from, double tip_cm, Pose& out, double arm_min_deg, double arm_max_deg);
double move_seconds(const Pose& from, const Pose& to); // the slower of the lift and arm moves

// Live readings for the controller readout.
Pose pose_now();
double tip_now();    // piece height (cm) from the lift and arm
int selected_stage_now(); // the stage Up/Down have selected, 1-7
int goal_tier_now(); // 0 alliance, 1 neutral, 2 center
Pose target_now();   // lift and arm setpoints currently commanded
int target_stage();  // 0 home, 1-7 scoring stage, -1 before the first press

}
