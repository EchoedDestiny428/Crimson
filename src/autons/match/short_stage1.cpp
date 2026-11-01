#include "autons/routine.hpp"
#include "subsystems/elevator/claw.hpp"
#include "subsystems/elevator/macros.hpp"

namespace autons::match::short_stage1 {

namespace {

namespace claw = subsystems::elevator::claw;
namespace macros = subsystems::elevator::macros;
namespace constants = subsystems::elevator::constants;

// Red alliance goal (the 8.25 cm one) the robot starts against and scores on (LemLib inches).
constexpr float kGoalX = -24.0f;
constexpr float kGoalY = -48.0f;
constexpr float kStartHeading = 90.0f;

}

void setup() {}

// Starts with its back against the alliance goal, facing 90 deg (away from it), holding
// a preload. Scores it as level 3 on that goal from behind.
void run() {
    chassis.setPose(kGoalX, kGoalY, kStartHeading);
    chassis.resetFromGoalBack(kGoalX, kGoalY); // back 22 cm from the goal: (-15.3, -48)
    claw::grab();
    // Level 3 (51.5 cm): arm swings up and over the top to 254.6 deg (75 deg past straight
    // up), no lift; with the 7 cm release offset the piece releases 27.4 cm behind
    // center, so the robot drives forward 5.4 cm to (-13.2, -48) and releases.
    macros::score_behind(constants::Goal::Alliance, 3, kGoalX, kGoalY);
}

}
