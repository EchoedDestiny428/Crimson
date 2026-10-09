#include "autons/routine.hpp"

namespace autons::skills::v1 {

void setup() {}

void run() {
    chassis.setPose(-31, -48, -90);
    pivot::set_pos(pivot::kFlippedMotorDeg);
    

    intake::spin(127);
    chassis.moveToPoint(-33.5, -48, 800, {.maxSpeed = 60, .minSpeed = 60});

    parallel ({
        [] {
            claw::spin(127);
            macros::home();
        },
        [] {
            chassis.turnToHeading(30, 500);
            chassis.moveToPoint(-25.5, -32, 1000, {.maxSpeed = 80});
            chassis.turnToHeading(100, 500);
        },
    });
    
    chassis.moveToPoint(0, -48, 1200);
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(0, -72, 800);

    parallel ({
        [] {
            macros::flip_out(10, 950);
        },
        [] {
            chassis.moveToPoint(0, -60, 500, {.minSpeed = 127});
            chassis.moveToPoint(0, -72, 500, {.minSpeed = 127});
            chassis.turnToPoint(24, -48, 500, {.forwards = false});
            chassis.moveToPoint(24, -48, 1100, {.forwards = false, .maxSpeed = 80});
        },
    });

    chassis.waitUntilDone();

    claw::spin(-127);
    pros::delay(500);

    
    
    parallel ({
        [] {
            macros::dual_setup();
            claw::spin(127);
        },
        [] {
            chassis.moveToPoint(0, -48, 1000);
            chassis.turnToPoint(24, -24, 500, {.forwards = false});
            chassis.moveToPoint(13, -35, 1000, {.forwards = false});
            chassis.waitUntilDone();
        }
    });

    macros::dual_pickup_height(700);

}
}
