#pragma once

#include "subsystems/elevator/constants.hpp"
#include <cstdint>
#include <optional>

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

// Auton motions, used by the chaos planner. Arm ranges for the solver, degrees from
// straight down: front up to straight up, behind from straight up to the 315 soft limit.
inline constexpr double kArmFrontMinDeg = constants::kArmHardstopDeg;
inline constexpr double kArmFrontMaxDeg = 180.0;
inline constexpr double kArmBehindMinDeg = 180.0;
inline constexpr double kArmBehindMaxDeg = 315.0;

// The blocking ones return false on a timeout or when stop() cancels them.
bool solve(double tip_cm, Pose& out, double arm_min_deg = kArmFrontMinDeg, double arm_max_deg = kArmBehindMaxDeg);
void aim(Pose pose);  // start the lift and arm toward pose (slack adjustment on the arm) and return
bool settled();       // lift and arm both settled
bool drop();          // open the claw, then wait for the piece to fall
bool home();          // the driver's home stage, blocking until there
bool pickup(std::uint32_t timeout_ms = 5000); // prime the claw, grab once the sensor sees a piece
double travel_seconds(const Pose& to);        // from the current pose
double claw_offset_in(const Pose& pose);      // release point ahead of robot center (negative: behind)

// Where to stop the chassis so the release point at `pose` lands on the goal, on the line
// from the robot to the goal, and which way to drive there. Nothing when already on it.
struct Approach {
    float x;
    float y;
    bool forwards;
};
std::optional<Approach> approach(const Pose& pose, float goal_x_in, float goal_y_in);
// Drive to approach() while aiming; true once the chassis stops and the elevator settles.
bool reach(const Pose& pose, float goal_x_in, float goal_y_in, std::uint32_t timeout_ms);

bool score(constants::Goal goal, int level, float goal_x_in, float goal_y_in, std::uint32_t timeout_ms = 3000);
bool score_front(constants::Goal goal, int level, float goal_x_in, float goal_y_in, std::uint32_t timeout_ms = 3000);
bool score_behind(constants::Goal goal, int level, float goal_x_in, float goal_y_in, std::uint32_t timeout_ms = 3000);

// Live readings for the controller readout.
Pose pose_now();
double tip_now();    // piece height (cm) from the lift and arm
int selected_stage_now(); // the stage Up/Down have selected, 1-7
int goal_tier_now(); // 0 alliance, 1 neutral, 2 center
Pose target_now();   // lift and arm setpoints currently commanded
int target_stage();  // 0 home, 1-7 scoring stage, -1 before the first press

}
