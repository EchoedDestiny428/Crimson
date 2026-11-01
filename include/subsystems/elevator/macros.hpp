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

// Fastest lift + arm angle that puts the arm tip at tip_cm above the ground, from
// where the elevator and arm are now, given their speeds. False if no pose reaches
// it or the extension/arm ratios in constants.hpp are not filled in yet.
bool solve(double tip_cm, Pose& out);

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

void cancel();

// Driver control: L1/L2 move the lift (snapping to a stage on release), Y toggles the
// scoring offset, B goes home. Not wired into main.cpp yet (see update()).
void press(bool up, bool down, bool down_pressed, bool offset_button);
void update();
void stop();

}
