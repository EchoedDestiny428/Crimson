#pragma once

#include "subsystems/elevator/constants.hpp"
#include <cstdint>
#include <functional>

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
// Holding a piece at home: closing the claw raises the lift to constants::kCarryLiftCm 0.5 s
// later, and home never brings it lower while the claw stays closed.
void update();
void stop();

// Lift height (cm) for home: constants::kCarryLiftCm while the claw is closed, so a held piece stays off the floor;
// 0 while it's open.
double home_lift_cm();

// ---- Auton motions (from new-robot, retuned from the arm-swing tests) ----
// Every arm swing eases in and out, gentler the higher the lift (arm_pivot.cpp): 3000 deg/s^2 with the lift down,
// about 1100 at L7. The blocking ones return false on a timeout or when stop() cancels them.
void aim(Pose pose); // start the lift and arm toward pose together (slack adjustment on the arm) and return
// aim() split in two, so only the lift moves while driving: aim_lift() starts the lift and remembers the arm angle;
// swing_arm() (once at the goal) swings the arm there and waits for the lift and arm to settle.
void aim_lift(Pose pose);
bool swing_arm(std::uint32_t timeout_ms = 1500);
// The order the swing tests point to: swing low, then lift. The lift stays where it is (or goes down to pose, if
// that's lower) while the arm waits for clear() and swings, fast and calm that low; once the arm settles, the lift goes
// up to pose. Use it where the claw has room to swing low (not over a goal's stack), so the lift can finish before the
// claw reaches the goal. Returns at once; each step moves on after timeout_ms, and any later lift/arm command cancels
// the rest.
void aim_swing_first(Pose pose, std::function<bool()> clear = [] { return true; }, std::uint32_t timeout_ms = 3000);
bool settled();                                     // lift and arm both settled
bool wait_settled(std::uint32_t timeout_ms = 2000); // block until settled (a home counts once it's on the hardstop)
// Open the claw and wait for the piece to fall. With the lift up, it first waits (up to 1 s) for the robot to stop
// rocking: at L7 the rock peaks ~280 ms after the arm stops, and 2 deg there is ~4 cm at the claw.
bool drop();
bool home();       // the driver's home stage, blocking until there
void home_async(); // start the driver's home stage (arm onto the hardstop) and return
bool pickup();     // close the claw on the piece in front of it, then wait for the grip

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
