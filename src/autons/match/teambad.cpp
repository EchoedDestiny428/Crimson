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
        [] { macros::flip_out(10, 950); },
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
            chassis.donut(-180, 1500, {.forwards = false, .fastSpeed = 127, .slowSpeed = 30});
            chassis.waitUntilDone();
        },
    });

    chassis.setPose(24, -54, chassis.getPose().theta);

    parallel({
        [] {
            pros::delay(200);
            macros::home();
        },
        [] {
            intake::lift();
            intake::spin(127);
            claw::spin(127);
            chassis.moveToPoint(24, -62, 800, {.maxSpeed = 80});
            chassis.moveToPoint(24, -55, 800, {.forwards = false});
            intake::stop();
            pros::delay(150);
            intake::drop();
            pros::delay(100);
            intake::spin(127);
            chassis.waitUntilDone();
        },
    });

    pros::delay(200);

    chassis.moveToPoint(24, -60, 800, {.maxSpeed = 80, .minSpeed = 50});

    chassis.waitUntilDone();
    pros::delay(500);

    parallel({
        [] { macros::flip_out(560, pivot::kFlippedMotorDeg); },
        [] {
            pros::delay(100);
            chassis.moveToPoint(24, -48, 1000, {.forwards = false, .maxSpeed = 50});
            chassis.donut(135, 1000, {.forwards = false, .fastSpeed = 127, .slowSpeed = 30});
            pros::delay(200);
            claw::spin(-127);
            chassis.waitUntilDone();
            chassis.setPoseFromGoal(24, -48);
        },
    });

    parallel({
        [] {
            pivot::move_to(pivot::kFlippedMotorDeg + 450);
            pros::delay(400);
            macros::dual_setup();
        },
        [] {
            chassis.moveToPoint(34, -58, 1000);
            chassis.waitUntilDone();
        },
    });

    parallel({
        [] {
            claw::spin(127);
            pros::delay(500);
            macros::dual_pickup_height(600); // changed the height to the correct height
        },
        [] {
            pros::delay(300);
            chassis.turnToPoint(43, -53, 700, {.forwards = false}); // needs tuning.
            chassis.moveToPoint(43, -53, 1200, {.forwards = false}); // goes to alliance goal
            chassis.waitUntilDone();
            pros::delay(300);
            chassis.waitUntilDone();
        },
    });


    claw::spin(-127);
    pros::delay(100);

    parallel({
        [] {
            pivot::move_to(pivot::kFlippedMotorDeg + 450); // change it to the correct angle
            pros::delay(200);
            macros::dual_setup();
        },
        [] {
            chassis.moveToPoint(35, -59, 550, {.forwards = true}); // needs tuning
            chassis.turnToPoint(28, -26, 350, {.forwards = false}); //needs tuning
            chassis.moveToPoint(28, -26, 350, {.forwards = false}); //needs tuning
            chassis.waitUntilDone();
        },
    });

    // third pin pickup
    claw::spin(127);

    parallel ({
        [] {
            pros::delay(700);
            macros::dual_pickup_height(elevator::kStage1);
            pros::delay(200);
            claw::spin(-127);
            pros::delay(300);
        },
        [] {
            chassis.moveToPoint(30.2, -27.2, 800, {.forwards = false});
            chassis.waitUntilDone();
            pros::delay(300);
            chassis.moveToPoint(-24, -51, 1200, {.forwards = false});
            chassis.moveToPoint(-24, -51, 600, {.forwards = false, .maxSpeed = 40});
            chassis.waitUntilDone();
        }
    });

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
