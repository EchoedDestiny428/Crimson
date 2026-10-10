#include "autons/routine.hpp"

namespace autons::skills::v1 {

void setup() {}

void run() {
    chassis.setPose(-39, -63.4, 180);
    parallel({
        [] { macros::flip_out(10, 950); },
        [] {
            chassis.moveToPoint(-39, -48, 1400, {.forwards = false});
            chassis.turnToHeading(-90, 1200);
            chassis.moveToPoint(-30, -48, 1400, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    parallel({
        [] {
            chassis.moveToPoint(-55.5, -48, 1400);
            chassis.turnToHeading(180, 1200);
            chassis.moveToPoint(-56.5, -63.4, 1400);
            chassis.waitUntilDone();
        },
        [] {
            pivot::move_to(pivot::kScoredMotorDeg);
            elevator::set_target(elevator::height() + elevator::kOffset);
            pros::delay(500);
            macros::go_to(subsystems::elevator::kHome);
        },
    });
    pros::delay(150);
    claw::spin(127);
    intake::spin(127);
    chassis.setPose(-56.5, -61.9, chassis.getPose().theta);
    pros::delay(1800);
    parallel({
        [] {
            intake::stop();
            macros::go_to(subsystems::elevator::kStage1 + subsystems::elevator::kOffset);
        },
        [] {
            chassis.moveToPoint(-58, -47.75, 1400, {.forwards = false});
            chassis.turnToHeading(-90, 1200);
            chassis.moveToPoint(-30, -48, 1400, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    parallel({
        [] {
            chassis.moveToPoint(-55.5, -48, 1400);
            chassis.turnToHeading(180, 1200);
            chassis.moveToPoint(-56.5, -63.4, 1400);
            chassis.waitUntilDone();
        },
        [] {
            pivot::move_to(pivot::kScoredMotorDeg);
            elevator::set_target(elevator::height() + elevator::kOffset);
            pros::delay(500);
            macros::go_to(subsystems::elevator::kHome);
        },
    });
    pros::delay(150);
    claw::spin(127);
    intake::spin(127);
    chassis.setPose(-56.5, -61.9, chassis.getPose().theta);
    pros::delay(1800);
    parallel({
        [] {
            intake::stop();
            macros::go_to(subsystems::elevator::kStage2 + subsystems::elevator::kOffset);
        },
        [] {
            chassis.moveToPoint(-58, -47.75, 1400, {.forwards = false});
            chassis.turnToHeading(-90, 1200);
            chassis.moveToPoint(-30, -48, 1400, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    parallel({
        [] {
            chassis.moveToPoint(-55.5, -48, 1400);
            chassis.turnToHeading(180, 1200);
            chassis.moveToPoint(-56.5, -63.4, 1400);
            chassis.waitUntilDone();
        },
        [] {
            pivot::move_to(pivot::kScoredMotorDeg);
            elevator::set_target(elevator::height() + elevator::kOffset);
            pros::delay(500);
            macros::go_to(subsystems::elevator::kHome);
        },
    });
    pros::delay(150);
    claw::spin(127);
    intake::spin(127);
    chassis.setPose(-56.5, -61.9, chassis.getPose().theta);
    pros::delay(1800);
    parallel({
        [] {
            intake::stop();
            macros::go_to(subsystems::elevator::kStage3 + subsystems::elevator::kOffset);
        },
        [] {
            chassis.moveToPoint(-58, -47.75, 1400, {.forwards = false});
            chassis.turnToHeading(-90, 1200);
            chassis.moveToPoint(-30, -48, 1400, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    parallel({
        [] {
            chassis.moveToPoint(-55.5, -48, 1400);
            chassis.turnToHeading(180, 1200);
            chassis.moveToPoint(-56.5, -63.4, 1400);
            chassis.waitUntilDone();
        },
        [] {
            pivot::move_to(pivot::kScoredMotorDeg);
            elevator::set_target(elevator::height() + elevator::kOffset);
            pros::delay(500);
            macros::go_to(subsystems::elevator::kHome);
        },
    });
    pros::delay(150);
    claw::spin(127);
    intake::spin(127);
    chassis.setPose(-56.5, -61.9, chassis.getPose().theta);
    pros::delay(1800);
    parallel({
        [] {
            intake::stop();
            macros::go_to(subsystems::elevator::kStage4 + subsystems::elevator::kOffset);
        },
        [] {
            chassis.moveToPoint(-58, -48, 1400, {.forwards = false});
            chassis.turnToHeading(-90, 1200);
            chassis.moveToPoint(-30, -48, 1400, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    parallel({
        [] {
            chassis.moveToPoint(-55.5, -48, 1400);
            chassis.turnToHeading(180, 1200);
            chassis.moveToPoint(-56.5, -63.4, 1400);
            chassis.waitUntilDone();
        },
        [] {
            pivot::move_to(pivot::kScoredMotorDeg);
            elevator::set_target(elevator::height() + elevator::kOffset);
            pros::delay(500);
            macros::go_to(subsystems::elevator::kHome);
        },
    });
    parallel({
        [] {
            intake::stop();
            macros::go_to(subsystems::elevator::kStage5 + subsystems::elevator::kOffset);
        },
        [] {
            chassis.moveToPoint(-58, -48, 1400, {.forwards = false});
            chassis.turnToHeading(-90, 1200);
            chassis.moveToPoint(-30, -48, 1400, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    // ends on intake cycle for some reason should be score last one i think, but elevator kept breaking so I left
    // stages for now middle intake
    chassis.setPose(-31, -48, -90);

    intake::spin(127);
    chassis.moveToPoint(-33.5, -48, 800, {.maxSpeed = 60, .minSpeed = 60});

    parallel({
        [] {
            claw::spin(127);
            macros::home();
        },
        [] {
            chassis.turnToHeading(30, 500);
            chassis.moveToPoint(-25.5, -33, 1400, {.maxSpeed = 60});
            chassis.turnToHeading(100, 500);
        },
    });

    chassis.moveToPoint(0, -48, 1200);
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(0, -72, 800);

    parallel({
        [] { macros::flip_out(10, 950); },
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

    parallel({[] {
                  macros::dual_setup();
                  claw::spin(127);
              },
              [] {
                  chassis.moveToPoint(0, -48, 1000);
                  chassis.turnToPoint(24, -24, 500, {.forwards = false});
                  chassis.moveToPoint(16.5, -31.0, 1000, {.forwards = false});
                  chassis.waitUntilDone();
              }});

    parallel({
        [] { macros::dual_pickup_height(700); },
        [] {
            pros::delay(800);
            chassis.turnToPoint(24, -48, 800, {.forwards = false});
        },
    });

    chassis.moveToPoint(24, -48, 800, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();

    claw::spin(-127);
    pros::delay(300);
    pivot::move_to(pivot::kFlippedMotorDeg + 450);
    pros::delay(400);

    chassis.moveToPoint(24, -27, 1000);
    claw::spin(127);

    parallel({[] { macros::dual_setup(); },
              [] {
                  chassis.turnToPoint(49, -51, 500, {.forwards = false});
                  chassis.moveToPoint(43.5, -45.5, 1200, {.forwards = false});
              }});

    chassis.waitUntilDone();

    parallel({
        [] { macros::dual_pickup_height(1300); },
        [] {
            pros::delay(800);
            chassis.turnToPoint(24, -50, 800, {.forwards = false});
            chassis.moveToPoint(24, -50, 1000, {.forwards = false, .maxSpeed = 60});
        },
    });

    chassis.waitUntilDone();
    claw::spin(-127);
    pros::delay(300);
    pivot::move_to(pivot::kFlippedMotorDeg + 450);
    pros::delay(400);
    chassis.moveToPoint(58, -52, 1000);
    parallel({[] { macros::home(); },
              [] {
                  chassis.turnToPoint(58, -72, 500);
                  chassis.moveToPoint(58, -72, 1000);
              }});

    chassis.waitUntilDone();
    pros::delay(150);
    claw::spin(127);
    intake::spin(127);
    chassis.setPose(59, -61.9, 0);
    pros::delay(1800);
    parallel({
        [] {
            intake::stop();
            macros::go_to(subsystems::elevator::kStage3);
        },
        [] {
            chassis.moveToPoint(57.75, -48, 1000, {.forwards = false});
            chassis.turnToHeading(90, 1000);
            chassis.moveToPoint(24, -48, 1000, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    parallel({
        [] {
            chassis.moveToPoint(57.75, -48, 1000);
            chassis.turnToHeading(0, 1000);
            chassis.moveToPoint(57.5, -63.4, 1000);
            chassis.waitUntilDone();
        },
        [] {
            pivot::move_to(pivot::kScoredMotorDeg);
            elevator::set_target(elevator::height() + elevator::kOffset);
            pros::delay(500);
            macros::go_to(subsystems::elevator::kHome);
        },
    });
}
}
