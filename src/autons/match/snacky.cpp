#include "autons/routine.hpp"

namespace autons::match::snacky {

void setup() {
    intake::lift();
}

void run() {
    chassis.setPose(7.8, -63.4, 180);

    parallel({
        [] { macros::flip_out(10, 800); },
        [] {
            chassis.moveToPoint(7.8, -62, 500, {.forwards = false, .minSpeed = 127});
            chassis.turnToPoint(24, -48, 500, {.forwards = false});
            chassis.moveToPoint(24, -48, 800, {.forwards = false, .maxSpeed = 60});
            chassis.waitUntilDone();
        },
    });

    claw::spin(-127);
    pros::delay(400);

    parallel({
        [] { macros::flip_out(elevator::kFlipOut, pivot::kFlippedMotorDeg); },
        [] {
            chassis.donut(-180, 5000, {.forwards = false, .fastSpeed = 127, .slowSpeed = 60});
            chassis.waitUntilDone();
        },
    });

}

}
