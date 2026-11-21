#include "autons/routine.hpp"
#include "subsystems/elevator/claw.hpp"

namespace autons::match::short_stage1 {

namespace {

namespace claw = subsystems::elevator::claw;

constexpr float kGoalX = 96.0f;
constexpr float kGoalY = 120.0f;
constexpr float kStartHeading = 180.0f;

}

void setup() {}

// Starts with its back against the alliance goal, facing 180 deg (away from it), holding
// a preload.
void run() {
    chassis.setPose(kGoalX, kGoalY, kStartHeading);
    chassis.resetFromGoalBack(kGoalX, kGoalY);
    claw::grab();
    // TODO: score. The auton scoring macro (score_behind) was removed with the rest of
    // the unused macros; this only sets the pose and grabs the preload for now.
}

}
