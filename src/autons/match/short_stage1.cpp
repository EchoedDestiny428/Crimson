#include "autons/routine.hpp"
#include "subsystems/elevator/claw.hpp"

namespace autons::match::short_stage1 {

namespace {

namespace claw = subsystems::elevator::claw;

// Red alliance goal (the 8.25 cm one) the robot starts against and scores on (LemLib inches).
constexpr float kGoalX = -24.0f;
constexpr float kGoalY = -48.0f;
constexpr float kStartHeading = 90.0f;

}

void setup() {}

// Starts with its back against the alliance goal, facing 90 deg (away from it), holding
// a preload.
void run() {
    chassis.setPose(kGoalX, kGoalY, kStartHeading);
    chassis.resetFromGoalBack(kGoalX, kGoalY); // back 22 cm from the goal: (-15.3, -48)
    claw::grab();
    // TODO: score. The auton scoring macro (score_behind) was removed with the rest of
    // the unused macros; this only sets the pose and grabs the preload for now.
}

}
