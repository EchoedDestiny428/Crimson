#include "autons/routine.hpp"
#include "subsystems/pivot.hpp"

namespace autons::match::teambad {

void setup() {
    pull_toggle.retract();
}

void run() {
    chassis.setPose(7.8, -63.4, 180);
    pull_toggle.extend();
    pros::delay(250);

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

    chassis.setPoseFromGoal(24, -48);

    parallel({
        [] {
            pros::delay(400);
            claw::spin(-127);
        },
        [] {
            chassis.donut(-180, 1200, {.forwards = false, .fastSpeed = 127, .slowSpeed = 30});
            pros::delay(400);
            intake::lift();
            chassis.waitUntilDone();
        },
    });

    chassis.setPoseFromGoal(24, -48);

    parallel({
        [] {
            pros::delay(200);
            macros::home();
        },
        [] {
            intake::spin(127);
            claw::spin(127);
            chassis.moveToPoint(24, -64, 500, {.minSpeed = 50});
            chassis.moveToPoint(24, -56, 500, {.forwards = false, .minSpeed = 50});
            intake::stop();
            pros::delay(150);
            intake::drop();
            intake::spin(127);
            chassis.moveToPoint(24, -60, 500, {.maxSpeed = 80, .minSpeed = 50});
            chassis.waitUntilDone();
        },
    });


    pros::delay(600);

    parallel({
        [] { macros::flip_out(elevator::kStage1, pivot::kFlippedMotorDeg); },
        [] {
            pros::delay(100);
            chassis.moveToPoint(24, -48, 800, {.forwards = false, .maxSpeed = 50});
            chassis.donut(135, 800, {.forwards = false, .fastSpeed = 127, .slowSpeed = 30});
            pros::delay(200);
            claw::spin(-127);
            chassis.waitUntilDone();
            chassis.setPoseFromGoal(24, -48);
        },
    });


    parallel({
        [] {
            pivot::move_to(pivot::kScoredMotorDeg); // change it to the correct angle
            pros::delay(100);
            macros::dual_setup();
        },
        [] {
            chassis.moveToPoint(33, -59, 700);
            chassis.turnToPoint(43.6, -52.5, 500, {.forwards = false}); 
            chassis.waitUntilDone();
        },
    });


    parallel({
        [] {
            claw::spin(127);
            pros::delay(600);
            macros::dual_pickup_height(elevator::kStage2 + 200); // changed the height to the correct height
        },
        [] {
            chassis.moveToPoint(43.6, -52.5, 1000, {.forwards = false}); // goes to alliance goal
            chassis.turnToPoint(24, -48, 500, {.forwards = false}); // needs tuning.
            chassis.moveToPoint(24, -48, 1000, {.forwards = false, .maxSpeed = 50}); // goes to alliance goal
            chassis.waitUntilDone();
        },
    });

    chassis.setPoseFromGoal(24, -48);
    claw::spin(-127);
    pros::delay(300);

    parallel({
        [] {
            pivot::move_to(pivot::kScoredMotorDeg);
            pros::delay(200);
            macros::dual_setup();
            claw::spin(127);
            pros::delay(800);
            macros::dual_pickup_height(elevator::kStage1 + 200);
        },
        [] {
            chassis.moveToPoint(48, -48, 650);
            chassis.turnToPoint(24, -24, 500, {.forwards = false});
            chassis.moveToPoint(29, -29, 1000, {.forwards = false});
            chassis.turnToPoint(-24, -48, 500, {.forwards = false}); //just needs to turn slightly first
            chassis.moveToPoint(-24, -48, 1000, {.forwards = false});
            pros::delay(200);
            chassis.waitUntilDone();
        },
    });

    chassis.moveToPoint(-24, -48, 600, {.forwards = false, .maxSpeed = 40});
    chassis.waitUntilDone();

    claw::spin(-127);
    pros::delay(300);

    parallel ({
        [] {
            pivot::move_to(pivot::kFlippedMotorDeg + 450);
            macros::home();
        },
        [] {
            chassis.moveToPoint(0, -36, 400, {.minSpeed = 127});
        },
    });
}

}
