#include "autons/routine.hpp"
#include "subsystems/elevator/claw.hpp"
#include "subsystems/elevator/macros.hpp"

namespace autons::match::short_stage1 {

namespace {

namespace claw = subsystems::elevator::claw;
namespace macros = subsystems::elevator::macros;
namespace constants = subsystems::elevator::constants;

// Red alliance goal (the 8.25 cm one, tag 3) the robot starts against and scores on, in the field frame from
// field_constants.hpp.
constexpr float kGoalX = 96.0f;
constexpr float kGoalY = 120.0f;
constexpr float kStartHeading = 180.0f;

}

void setup() {}

// Starts with its back against the alliance goal, facing 180 deg (away from it), holding
// a preload. Scores it as level 3 on that goal from behind.
void run() {
    chassis.setPose(kGoalX, kGoalY, kStartHeading);
    chassis.resetFromGoalBack(kGoalX, kGoalY); // back 22 cm from the goal: (87.3, 120)
    claw::grab();
    // Level 3 (51.5 cm): arm swings up and over the top to 252.2 deg, no lift; with the
    // 7 cm release offset the piece releases 26.0 cm behind center, so the robot drives
    // forward 4.0 cm to (85.8, 120) and releases.
    macros::score_behind(constants::Goal::Alliance, 3, kGoalX, kGoalY);
}

}
