#pragma once

#include "subsystems/elevator/constants.hpp"
#include <cstdint>

namespace subsystems::elevator::macros {

// Elevator lift (cm above the start) and arm angle (degrees from straight down).
struct Pose {
    double lift_cm;
    double arm_deg;
};

// Moves straight to a lift and arm angle. True once both settle; false if
// cancelled by a newer macro or cancel(), or timed out.
bool go_to_pose(Pose pose);

// Arm angles are degrees from straight down, counted continuously as the arm swings
// up: 0 to 180 puts the claw in front of the pivot (180 is straight up), and 180 to
// 360 carries it over the top to behind. The solver keeps the arm at least
// kArmMinFromDownDeg from straight down, so it reaches the back by swinging up and
// over, never down through 0. Manual control is not limited by this.
inline constexpr double kArmMinFromDownDeg = 25.0;
inline constexpr double kArmFrontMinDeg = kArmMinFromDownDeg;
inline constexpr double kArmFrontMaxDeg = 180.0;
inline constexpr double kArmBehindMinDeg = 180.0;
inline constexpr double kArmBehindMaxDeg = 360.0 - kArmMinFromDownDeg;

// Fastest lift + arm angle within [arm_min_deg, arm_max_deg] that puts the arm tip at
// tip_cm above the ground, from where the elevator and arm are now, given their
// speeds. False if no pose in range reaches it or the ratios are not filled in yet.
bool solve(double tip_cm, Pose& out, double arm_min_deg = kArmFrontMinDeg, double arm_max_deg = kArmBehindMaxDeg);

// Moves the arm tip to tip_cm using solve(). True once both mechanisms settle.
bool move_to(double tip_cm);

// Carried over from the old robot's macros, using the setpoints in constants.hpp.
// Each returns true once the move settles, false if cancelled or timed out.
bool flip_out(double lift_cm = constants::kExtensionFlipOutCm, double arm_deg = constants::kArmFlippedDeg);
bool go_to_lift(double lift_cm);
bool go_to_stage(int stage); // 1-5, constants::kExtensionStage1Cm ... kExtensionStage5Cm
bool home();
bool dual_setup();
bool dual_pickup();
bool dual_pickup_height(double lift_cm);

// Primes the claw (open), waits for the distance sensor to read closer than
// constants::kPickupDistanceMm, then closes: claw::state() becomes Grabbed.
bool pickup(std::uint32_t timeout_ms = 5000);

// Moves the held piece to stack `level` on `goal` with move_to() and releases it.
// Level 1 is the first pin, scored alone; every later level is a cup/pin combo.
bool score(constants::Goal goal, int level);

// Scores on a goal at field point (goal_x_in, goal_y_in), LemLib inches. Solves the
// fastest lift + arm pose for the level, then drives the robot along the line to the
// goal so the claw ends over it (facing the goal if the claw is in front, backing in
// if it is behind), moving the elevator and arm at the same time, and releases.
// score() picks front or behind, whichever is faster.
bool score(constants::Goal goal, int level, float goal_x_in, float goal_y_in, std::uint32_t timeout_ms = 3000);
bool score_front(constants::Goal goal, int level, float goal_x_in, float goal_y_in, std::uint32_t timeout_ms = 3000);
bool score_behind(constants::Goal goal, int level, float goal_x_in, float goal_y_in, std::uint32_t timeout_ms = 3000);

// Where the piece releases along the robot's heading from its center for a pose, in
// inches: arm_length * sin(arm) - kPivotBehindCenterCm + kClawReleaseOffsetCm.
// Negative means behind.
double claw_offset_in(const Pose& pose);

void cancel();

// Stages are the scoring levels on kStageGoal: stage n puts the arm tip at
// release_cm(kStageGoal, n) using the solver; stage 0 is the starting pose.
inline constexpr constants::Goal kStageGoal = constants::Goal::Alliance;
inline constexpr int kTopStage = static_cast<int>(constants::kAllianceReleaseCm.size()); // measured levels
bool go_to_level(int stage);
void stage_up();   // one stage above the tip (or above the stage already being moved to)
void stage_down(); // one stage below

// Driver control: tap L1/L2 to step a stage up/down, hold either (250 ms) to drive the
// lift by hand. B goes to stage 0.
void update();
void stop();

}
