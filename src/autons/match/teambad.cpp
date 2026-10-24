#include "autons/routine.hpp"
#include "subsystems/pivot.hpp"

namespace autons::match::teambad {

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

    chassis.setPoseFromGoal(24, -48, {.maxCorrection = 1000}); // always reset, even when odom drifted far

    claw::spin(-127);
    pros::delay(600);

    // parallel({
    //     [] {
    //         pros::delay(400);
    //         claw::spin(-127);
    //     },
    //     [] {
    //         chassis.donut(-183, 1300, {.forwards = false, .fastSpeed = 127, .slowSpeed = 30});
    //         pros::delay(400);
    //         intake::lift();
    //         chassis.waitUntilDone();
    //     },
    // });

    // chassis.setPoseFromGoal(24, -48);

    // parallel({
    //     [] {
    //         pros::delay(200);
    //         macros::home();
    //     },
    //     [] {
    //         intake::spin(50);
    //         claw::spin(127);
    //         chassis.moveToPoint(24, -64, 600);
    //         chassis.moveToPoint(24, -56, 400, {.forwards = false, .maxSpeed = 50});
    //         intake::drop();
    //         intake::spin(127);
    //         pros::delay(50);
    //         intake::lift();
    //         chassis.moveToPoint(24, -60, 600, {.maxSpeed = 80, .minSpeed = 50});
    //         pros::delay(150);
    //         intake::drop();
    //         intake::spin(127);
    //         chassis.waitUntilDone();
    //     },
    // });

    // pros::delay(400);

    // parallel({
    //     [] { macros::flip_out(elevator::kStage1, pivot::kFlippedMotorDeg); },
    //     [] {
    //         pros::delay(100);
    //         // chassis.moveToPoint(24, -48, 800, {.forwards = false, .maxSpeed = 50});
    //         pros::delay(200);
    //         claw::spin(-127);
    //         chassis.waitUntilDone();
    //         chassis.setPoseFromGoal(24, -48);
    //     },
    // });
    
    chassis.donut(135, 2000, {.forwards = false, .fastSpeed = 127, .slowSpeed = 30});
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(24, -48, {.maxCorrection = 1000}); // always reset, even when odom drifted far

    parallel({
        [] {
            pivot::move_to(pivot::kScoredMotorDeg); // change it to the correct angle
            pros::delay(100);
            macros::dual_setup();
        },
        [] {
            chassis.moveToPoint(33, -59, 700);
            chassis.turnToPoint(44.0, -50.5, 600, {.forwards = false}); 
            chassis.waitUntilDone();
        },
    });


    parallel({
        [] {
            claw::spin(127);
            pros::delay(500);
            macros::dual_pickup_height(elevator::kStage1); // changed the height to the correct height
        },
        [] {
            chassis.moveToPoint(44.0, -51.125, 1200, {.forwards = false});
            chassis.turnToPoint(24, -46, 800, {.forwards = false}); // needs tuning.
            chassis.moveToPoint(24, -46, 1200, {.forwards = false, .maxSpeed = 50});
            chassis.waitUntilDone();
        },
    });

    chassis.setPoseFromGoal(24, -48, {.maxCorrection = 1000}); // always reset, even when odom drifted far
    claw::spin(-127);
    pros::delay(200);

    parallel({
        [] {
            pivot::move_to(pivot::kScoredMotorDeg);
            pros::delay(200);
            macros::dual_setup();
            claw::spin(127);
        },
        [] {
            chassis.moveToPoint(48, -48, 800);
            chassis.turnToPoint(28.5, -26.5, 500, {.forwards = false});
            chassis.moveToPoint(28.5, -26.5, 800, {.forwards = false});
            chassis.waitUntilDone();
        },
    });

    parallel ({
        [] {
            macros::dual_pickup_height(elevator::kStage1+elevator::kOffset+70); // changed the height to the correct height
        },
        [] {
            pros::delay(400);
            chassis.turnToPoint(-24, -46, 600, {.forwards = false}); //just needs to turn slightly first
            chassis.moveToPoint(-24, -46, 1000, {.forwards = false, .minSpeed = 127});        
            chassis.moveToPoint(-24, -46, 1000, {.forwards = false, .maxSpeed = 50});
            chassis.waitUntilDone();
            claw::spin(-127);
        },
    });

    pros::delay(300);

    parallel ({
        [] {
            pivot::move_to(pivot::kFlippedMotorDeg + 500);
            pros::delay(800);
            macros::home();
            claw::stop();
        },
        [] {
            pros::delay(400);
            chassis.moveToPoint(0, -36, 400, {.minSpeed = 127});
        },
    });
}

}

namespace autons::match::teambad_cw {

void setup() {
    pull_toggle.retract();
}

// X mirrored from TeamBad. First two scores are the bottom red goal. Last score is the neutral at (24, -48).
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

    chassis.donut(225, 2000, {.forwards = false, .fastSpeed = 127, .slowSpeed = 30});
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(-24, -48, {.maxCorrection = 1000});

    parallel({
        [] {
            macros::dual_setup();
        },
        [] {
            chassis.moveToPoint(-33, -59, 700);
            chassis.turnToPoint(-42.0, -52, 600, {.forwards = false});
            chassis.waitUntilDone();
        },
    });

    parallel({
        [] {
            claw::spin(127);
            pros::delay(500);
            macros::dual_pickup_height(elevator::kStage1);
        },
        [] {
            chassis.moveToPoint(-42.0, -52, 1200, {.forwards = false});
            chassis.turnToPoint(-24, -48, 900, {.forwards = false});
            chassis.moveToPoint(-24, -48, 1200, {.forwards = false, .maxSpeed = 50});
            chassis.waitUntilDone();
        },
    });
    chassis.setPoseFromGoal(-24, -48, {.maxCorrection = 1000});
    claw::spin(-127);
    pros::delay(200);

    parallel({
        [] {
            pivot::move_to(pivot::kScoredMotorDeg+80);
            pros::delay(900);
            macros::dual_setup();
            claw::spin(127);
        },
        [] {
            chassis.moveToPoint(-48, -48, 800);
            chassis.turnToPoint(-26, -26.5, 500, {.forwards = false});
            chassis.moveToPoint(-26, -26.5, 800, {.forwards = false,.maxSpeed = 80});
            chassis.waitUntilDone();
        },
    });

    parallel({
        [] {
            macros::dual_pickup_height(elevator::kStage1 + elevator::kOffset + 70);
        },
        [] {
            pros::delay(400);
            chassis.turnToPoint(24, -48, 600, {.forwards = false});
            chassis.moveToPoint(24, -48, 900, {.forwards = false, .minSpeed = 127});
            chassis.moveToPoint(24, -48, 1000, {.forwards = false, .maxSpeed = 40});
            chassis.waitUntilDone();
            claw::spin(-127);
        },
    });

    pros::delay(350);

    parallel({
        [] {
            pivot::move_to(pivot::kFlippedMotorDeg + 550);
            pros::delay(800);
            macros::home();
            claw::stop();
        },
        [] {
            chassis.moveToPoint(0, -36, 400, {.minSpeed = 90});
        },
    });
}

}
