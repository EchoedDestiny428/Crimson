#pragma once

#include "subsystems/elevator/constants.hpp"

namespace subsystems::elevator::macros {

// Elevator lift (cm above the start) and arm angle (degrees from straight down).
struct Pose {
    double lift_cm;
    double arm_deg;
};

// Driver control, always scoring behind. Each press moves the lift and arm to a hard
// setpoint (constants::kStageSetpoints) on their PID: L1 goes down a scoring stage
// (minimum L1, never home), L2 goes up one (maximum L7); from home either goes to L1.
// B goes home (the intake pose). Right cycles the goal (alliance, neutral, center),
// moving only the lift 7 cm per goal. R1/R2 drive the lift by hand (100% up, 70% down)
// and hold where let go.
void update();
void stop();

// Live readings for the controller readout.
Pose pose_now();
double tip_now();    // piece height (cm) from the lift and arm
int goal_tier_now(); // 0 alliance, 1 neutral, 2 center
Pose target_now();   // lift and arm setpoints currently commanded
int target_stage();  // 0 home, 1-7 scoring stage, -1 before the first press

}
