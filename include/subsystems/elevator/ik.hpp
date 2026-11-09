#pragma once

#include "subsystems/elevator/constants.hpp"
#include <array>

namespace subsystems::elevator::ik {

// Release point of the piece, cm. x is forward-positive from the arm pivot (scoring
// behind the robot is negative). z is height above the ground.
//
// The pivot motor is direct drive. The chain only keeps the wrist level, so the 7 cm
// release offset stays horizontal, toward the front, at every arm angle:
//   x = L sin(arm) + 7
//   z = pivot_height + lift - L cos(arm)
// Arm degrees are from straight down, same as the rest of the elevator. 270 is
// horizontal behind. 230 and 310 are the same x, mirrored about that.

double release_x_cm(double arm_deg);
double release_z_cm(double lift_cm, double arm_deg);

// Scoring-side range (cm) to the forward-positive release x. A reading of
// constants::kScoreRangeAtRammedCm is the rammed pose. Larger means the goal is
// farther behind, so x gets more negative.
double x_from_score_range_cm(double range_cm);

// One pose that hits (x, z) with the lift inside 0..kMaxLiftCm and the arm inside
// 35..315 deg. Anything outside that is not valid. There is no clamped fallback.
struct Solution {
    bool valid = false;
    double lift_cm = 0.0;
    double arm_deg = 0.0;
    double z_cm = 0.0;
};

// θ and 540° − θ for one x. At the design distance those are 310° and 230°, mirrors
// about horizontal behind (270°). Either can be invalid.
struct Pair {
    Solution high; // 540° − θ, above horizontal behind
    Solution low;  // θ, at or below it
};

// Both poses for a release at (x_cm, z_cm). Empty if that x is past the arm's reach
// (|sin| > 1) or either input isn't finite. Does not pick one.
Pair solve(double x_cm, double z_cm);

// Of the poses with 0 ≤ lift ≤ kMaxLiftCm and 35° ≤ arm ≤ 315°, the one whose lift
// is closer to from_lift_cm. Equal lift travel keeps the shorter arm move.
Solution choose(const Pair& pair, double from_lift_cm, double from_arm_deg);

struct Step {
    double lift_cm = 0.0;
    double arm_deg = 0.0;
};

// Ordered macro, 1 to 3 steps. Not valid when choose() has nothing.
// The lift rises before the arm crosses the top. Away from the rear crossbar the arm
// swings 5 cm above the pin and the lift drops last, so the piece settles vertically.
// A low-arm score (within 30 deg of the 315 deg crossbar) skips that extra and swings
// at the final height, which is the move the arm's slow zone was tuned for.
struct Motion {
    bool valid = false;
    Solution goal;
    std::array<Step, 3> steps{};
    int count = 0;
};

Motion plan(double from_lift_cm, double from_arm_deg, double x_cm, double z_cm);

}
