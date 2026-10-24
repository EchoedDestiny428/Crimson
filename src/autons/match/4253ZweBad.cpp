#include "autons/routine.hpp"

namespace autons::match::zwe_bad_4253 {

void setup() {
    pull_toggle.retract();
}

void run() {
    chassis.setPose(7.8, -63.4, 180);
    pull_toggle.extend();
    chassis.tank(127, 127, true);
    pros::delay(150);

    parallel({
        [] { macros::flip_out(10, 900); },
        [] {
            chassis.moveToPoint(7.8, -62.0, 500, {.forwards = false, .minSpeed = 127});
            chassis.turnToPoint(24, -48, 400, {.forwards = false});
            chassis.moveToPoint(24, -48, 400, {.forwards = false, .minSpeed = 60});
            chassis.moveToPoint(24, -48, 200, {.forwards = false, .maxSpeed = 30});
            chassis.waitUntilDone();
        },
    });

    chassis.setPoseFromGoal(24, -48, {.maxCorrection = 1000});
    claw::spin(-127);
    pros::delay(600);
    chassis.moveToPoint(-24,-32, 1900);
}

}

namespace autons::match::zwe_bad_4253_cw {

void setup() {
    pull_toggle.retract();
}

void run() {
    chassis.setPose(-7.8, -63.4, 180);
    pull_toggle.extend();
    chassis.tank(45, 45, true);
    pros::delay(300);
    chassis.tank(0, 0);

    parallel({
        [] { macros::flip_out(10, 900); },
        [] {
            chassis.moveToPoint(-7.8, -62.0, 500, {.forwards = false, .minSpeed = 127});
            chassis.turnToPoint(-25, -47, 400, {.forwards = false});
            chassis.moveToPoint(-25, -47, 400, {.forwards = false, .minSpeed = 60});
            chassis.moveToPoint(-25, -47, 350, {.forwards = false, .maxSpeed = 50});
            chassis.waitUntilDone();
        },
    });

    chassis.setPoseFromGoal(-24, -48, {.maxCorrection = 1000});
    claw::spin(-127);
    pros::delay(600);
    chassis.moveToPoint(24,-32, 1900);
}

}