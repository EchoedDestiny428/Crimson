#pragma once

#include <cstdint>

namespace subsystems::elevator::score {

// Alliance pin height plus the goal tier (0 alliance, 1 neutral, 2 center).
// Not finite for a stage outside 1..7 or a tier outside 0..2.
double stage_z_cm(int stage, int tier);

// Block until the lift and arm have run the IK motion for a piece released at
// z_cm, using range_cm from the scoring side (see ik::x_from_score_range_cm).
// False if no pose keeps the lift inside 0..kMaxLiftCm and the arm inside
// 35..315 deg (L1's 17.5 cm is below the arm, so it is refused), the release
// would be in front of the pivot, or a step doesn't settle before timeout_ms.
//
// Pass range_cm from the distance sensor or, later, from a backward camera.
// The two agree as long as both report centimeters past the rammed pose.
bool go(double z_cm, double range_cm, std::uint32_t timeout_ms = 5000);
bool go_measured(double z_cm, std::uint32_t timeout_ms = 5000); // ports::kScoreDistance

bool go_stage(int stage, int tier, double range_cm, std::uint32_t timeout_ms = 5000);
bool go_stage_measured(int stage, int tier, std::uint32_t timeout_ms = 5000);

}
