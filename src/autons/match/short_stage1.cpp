#include "autons/routine.hpp"
#include "subsystems/elevator/claw.hpp"
#include "subsystems/elevator/macros.hpp"

namespace autons::match::short_stage1 {

namespace {

namespace claw = subsystems::elevator::claw;
namespace macros = subsystems::elevator::macros;
namespace constants = subsystems::elevator::constants;

constexpr float kGoalX = 96.0f;
constexpr float kGoalY = 120.0f;
constexpr float kStartHeading = 180.0f;

}

void setup() {}

void run() {
    chassis.setPose(kGoalX, kGoalY, kStartHeading);
    chassis.resetFromGoalBack(kGoalX, kGoalY);
    claw::grab();
    macros::score_behind(constants::Goal::Alliance, 3, kGoalX, kGoalY);
}

}
