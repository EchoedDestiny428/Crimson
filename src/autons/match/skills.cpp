#include "autons/routine.hpp"
#include "pros/rtos.hpp"
#include "subsystems/side_toggle.hpp"

namespace autons::skills {

namespace {

constexpr float kStartX = 62.75f;
constexpr float kStartY = 0.0f;
constexpr float kStartHeading = 90.0f;
constexpr float kTurnHeading = 80.0f;
constexpr double kToggleRotations = 3.0;

}

void setup() {}

void run() {
    chassis.setPose(kStartX, kStartY, kStartHeading);
    side_toggle_motors.move_relative(kToggleRotations * 360.0, subsystems::side_toggle::kForwardPower);
    pros::delay(1800);
    side_toggle_motors.brake();
    chassis.moveToPoint(-16, -56, 800, {.forwards=false});
}

}
