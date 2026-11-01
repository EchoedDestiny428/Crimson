#include "autons/routine.hpp"
#include "subsystems/pivot.hpp"

namespace autons::match::teambad {

void setup() {
    pull_toggle.retract();
}

void run() {
    chassis.setPose(7.8, -63.4, 180);

    parallel ({
        [] {
            pull_toggle.extend();
        },
        [] {
            chassis.moveToPoint(7.8, -70.0, 400, {.minSpeed = 127});
        }
    });

    parallel({
        [] { macros::flip_out(10, 900); },
        [] {
            chassis.moveToPoint(7.8, -62.0, 8000, {.forwards = false, .minSpeed = 127});
            chassis.turnToPoint(24, -48, 500, {.forwards = false});
            chassis.moveToPoint(24, -48, 600, {.forwards = false, .minSpeed = 60});
            chassis.moveToPoint(24, -48, 500, {.forwards = false, .maxSpeed = 30});
            chassis.waitUntilDone();
        },
    });

    chassis.setPoseFromGoal(24, -48); // always reset, even when odom drifted far

    claw::spin(-127);
    pros::delay(600);

    
    chassis.donut(135, 2000, {.forwards = false, .fastSpeed = 127, .slowSpeed = 30});
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(24, -48); // always reset, even when odom drifted far


    parallel({
        [] {
            pivot::move_to(pivot::kScoredMotorDeg); // change it to the correct angle
            pros::delay(100);
            macros::dual_setup();
        },
        [] {
            chassis.moveToPoint(40, -58, 900);
            chassis.turnToPoint(46.8, -49.2, 600, {.forwards = false}); 
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
            chassis.moveToPoint(46.8, -49.2, 1000, {.forwards = false});
            chassis.turnToPoint(24, -46, 800, {.forwards = false}); // needs tuning.
            chassis.moveToPoint(24, -46, 1000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
        },
    });

    chassis.setPoseFromGoal(24, -48); // always reset, even when odom drifted far
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
            chassis.turnToPoint(29.5, -27.5, 500, {.forwards = false});
            chassis.moveToPoint(29.5, -27.5, 900, {.forwards = false});
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
    chassis.moveToPoint(0, -36, 400, {.minSpeed = 127});

    pivot::move_to(pivot::kFlippedMotorDeg + 500);
    pros::delay(800);
    macros::home();
    claw::stop();
}

}

namespace autons::match::teambad_cw {

void setup() {
    pull_toggle.retract();
}

// X mirrored from TeamBad.
void run() {
    chassis.setPose(-7.8, -63.4, 180);

    parallel ({
        [] {
            pull_toggle.extend();
        },
        [] {
            chassis.moveToPoint(-7.8, -70.0, 400, {.minSpeed = 127});
        }
    });

    parallel({
        [] { macros::flip_out(10, 900); },
        [] {
            chassis.moveToPoint(-7.8, -62.0, 8000, {.forwards = false, .minSpeed = 127});
            chassis.turnToPoint(-24, -48, 500, {.forwards = false});
            chassis.moveToPoint(-24, -48, 600, {.forwards = false, .minSpeed = 60});
            chassis.moveToPoint(-24, -48, 500, {.forwards = false, .maxSpeed = 30});
            chassis.waitUntilDone();
        },
    });

    chassis.setPoseFromGoal(-24, -48); // always reset, even when odom drifted far

    claw::spin(-127);
    pros::delay(600);

    chassis.donut(225, 2000, {.forwards = false, .fastSpeed = 127, .slowSpeed = 30});
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(-24, -48); // always reset, even when odom drifted far

    parallel({
        [] {
            pivot::move_to(pivot::kScoredMotorDeg); // change it to the correct angle
            pros::delay(100);
            macros::dual_setup();
        },
        [] {
            chassis.moveToPoint(-40, -58, 900);
            chassis.turnToPoint(-46.8, -49.2, 600, {.forwards = false});
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
            chassis.moveToPoint(-46.8, -49.2, 1000, {.forwards = false});
            chassis.turnToPoint(-24, -46, 800, {.forwards = false}); // needs tuning.
            chassis.moveToPoint(-24, -46, 1000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
        },
    });

    chassis.setPoseFromGoal(-24, -48); // always reset, even when odom drifted far
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
            chassis.moveToPoint(-48, -48, 800);
            chassis.turnToPoint(-29.5, -27.5, 500, {.forwards = false});
            chassis.moveToPoint(-29.5, -27.5, 900, {.forwards = false});
            chassis.waitUntilDone();
        },
    });

    parallel ({
        [] {
            macros::dual_pickup_height(elevator::kStage1+elevator::kOffset+70); // changed the height to the correct height
        },
        [] {
            pros::delay(400);
            chassis.turnToPoint(24, -46, 600, {.forwards = false}); //just needs to turn slightly first
            chassis.moveToPoint(24, -46, 1000, {.forwards = false, .minSpeed = 127});
            chassis.moveToPoint(24, -46, 1000, {.forwards = false, .maxSpeed = 50});
            chassis.waitUntilDone();
            claw::spin(-127);
        },
    });

    pros::delay(300);
    chassis.moveToPoint(0, -36, 400, {.minSpeed = 127});

    pivot::move_to(pivot::kFlippedMotorDeg + 500);
    pros::delay(800);
    macros::home();
    claw::stop();
}

}
