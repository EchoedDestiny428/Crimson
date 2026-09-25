#include "autons/routine.hpp"

namespace autons::match::snacky {

void setup() {

}

void run() {
    // chassis.setPose(7.8, -63.4, 180);

    // parallel({
    //     [] { macros::flip_out(10, 950); },
    //     [] {
    //         chassis.moveToPoint(7.8, -62.5, 500, {.forwards = false, .minSpeed = 127});
    //         chassis.turnToPoint(24, -48, 500, {.forwards = false});
    //         chassis.moveToPoint(24, -48, 400, {.forwards = false, .minSpeed = 60});
    //         chassis.moveToPoint(24, -48, 200, {.forwards = false, .maxSpeed = 30});
    //         chassis.waitUntilDone();
    //     },
    // });

    
    // parallel({
    //     [] { 
    //         pros::delay(500);
    //         claw::spin(-127);
             
    //     },
    //     [] {
    //         chassis.donut(-180, 1000, {.forwards = false, .fastSpeed = 127, .slowSpeed = 30});
    //         chassis.waitUntilDone();
    //     },
    // });

    //at this point the bot should be at 24, -54 ish
    chassis.setPose(24, -54, 180);
    pivot::set_pos(950);

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
            chassis.moveToPoint(24, -54, 400, {.forwards = false});
            intake::drop();
            chassis.waitUntilDone();
        },
    });

    chassis.moveToPoint(24, -62, 500);
    chassis.waitUntilDone();
    pros::delay(500);

    parallel({
        [] {
            macros::flip_out(500, pivot::kFlippedMotorDeg);
        },
        [] {
            pros::delay(100);
            chassis.moveToPoint(24, -54, 500, {.forwards = false, .maxSpeed = 50});
            chassis.donut(135, 700, {.forwards = false, .fastSpeed = 127, .slowSpeed = 30});
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
            chassis.moveToPoint(36, -60, 700);
            chassis.turnToHeading(-135, 450);
            chassis.waitUntilDone();
        },
    });

    parallel({
        [] {
            pros::delay(200);
            macros::dual_pickup();
        },
        [] {
            chassis.moveToPoint(42.5, -52.9, 800, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
}

}
