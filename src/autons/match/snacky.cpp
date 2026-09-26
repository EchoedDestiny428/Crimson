#include "autons/routine.hpp"

namespace autons::match::snacky {

void setup() {

}

void run() {
    chassis.setPose(7.8, -63.4, 180);

    parallel({
        [] { macros::flip_out(10, 950); },
        [] {
            chassis.moveToPoint(7.8, -62.0, 500, {.forwards = false, .minSpeed = 127});
            chassis.turnToPoint(24, -48, 500, {.forwards = false});
            chassis.moveToPoint(24, -48, 400, {.forwards = false, .minSpeed = 60});
            chassis.moveToPoint(24, -48, 200, {.forwards = false, .maxSpeed = 30});
            chassis.waitUntilDone();
        },
    });

    
    parallel({
        [] { 
            pros::delay(500);
            claw::spin(-127);
             
        },
        [] {
            chassis.donut(-180, 1500, {.forwards = false, .fastSpeed = 127, .slowSpeed = 30});
            chassis.waitUntilDone();
        },
    });

    //at this point the bot should be at 24, -54 ish
    chassis.setPose(24, -54, 180);

    parallel({
        [] { 
            pros::delay(200);
            macros::home();
        },
        [] {
            intake::lift();
            intake::spin(127);
            claw::spin(127);
            chassis.moveToPoint(24, -62, 800);
            chassis.moveToPoint(24, -58, 400, {.forwards = false});
            pros::delay(150);
            intake::drop();
            chassis.waitUntilDone();
        },
    });

    pros::delay(500);

    chassis.moveToPoint(24, -60, 500, {.minSpeed = 50});
    
    chassis.waitUntilDone();
    pros::delay(500);

    parallel({
        [] {
            macros::flip_out(560, pivot::kFlippedMotorDeg);
        },
        [] {
            pros::delay(300);
            chassis.moveToPoint(24, -54, 500, {.forwards = false, .maxSpeed = 50});
            chassis.donut(135, 1200, {.forwards = false, .fastSpeed = 127, .slowSpeed = 30});
            pros::delay(200);
            claw::spin(-127);
            chassis.waitUntilDone();
        },
    });

    parallel({
        [] {  
            macros::flip_out(700, pivot::kFlippedMotorDeg + 200);
            macros::dual_setup();
        },
        [] {
            chassis.moveToPoint(35, -59, 700);
            chassis.turnToHeading(-135, 450);
            chassis.waitUntilDone();
        },
    });

    parallel({
        [] {
            claw::spin(127);
            pros::delay(500);
            macros::dual_pickup();
        },
        [] {
            chassis.moveToPoint(42.6, -52.5, 800, {.forwards = false});
            chassis.waitUntilDone();
        },
    });

    parallel({
        [] {
            macros::flip_out(600, pivot::kFlippedMotorDeg);
        },
        [] {
            chassis.moveToPoint(48.5, -24, 450, {.forwards = false});
            chassis.moveToPoint(48.5, -24, 450, {.forwards = false, .maxSpeed = 40});
            chassis.waitUntilDone();
        },
    });

    pros::delay(400);
    claw::spin(-127);
    pros::delay(400);
    macros::flip_out(800, pivot::kFlippedMotorDeg + 200);
}

}
