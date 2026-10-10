#include "autons/routine.hpp"

namespace autons::skills::v1 {

void setup() {}

void run() {
    chassis.setPose(-38, -63.4, 180);
    parallel({
        [] { macros::flip_out(10, 950); },
        [] {
            chassis.moveToPoint(-24, -48, 1000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
        },
    });

    claw::spin(-127);
    pros::delay(300);
    chassis.setPoseFromGoal(-24, -48);

    // ----------------------------------

    for (const double stage : {subsystems::elevator::kStage1}) {
        parallel({
            [] {
                chassis.moveToPoint(-56, -52, 1200, {.maxSpeed = 100});
                chassis.turnToPoint(-56, -72, 800);
                chassis.moveToPoint(-56, -72, 400);
                chassis.moveToPoint(-56, -72, 1250, {.maxSpeed = 50}); //  this is the loading timeout
            },
            [] {
                pivot::move_to(pivot::kScoredMotorDeg);
                elevator::set_target(elevator::height() + elevator::kOffset);

                pros::delay(500);
                macros::home();
                claw::spin(127);
                intake::spin(127);
            },
        });

        chassis.waitUntilDone();
        chassis.moveToPoint(-56, -48, 700, {.forwards = false});
        chassis.turnToPoint(-24, -48, 500, {.forwards = false});
        chassis.waitUntilDone();

        parallel({
            [stage] {
                intake::stop();
                macros::flip_out(stage + 150, pivot::kFlippedMotorDeg + 100);
            },
            [] {
                chassis.moveToPoint(-24, -48, 600, {.forwards = false});
                chassis.moveToPoint(-24, -48, 900, {.forwards = false, .maxSpeed = 40});
            },
        });

        chassis.waitUntilDone();
        chassis.setPoseFromGoal(-24, -48);

        claw::spin(-127);
        pros::delay(300);

        chassis.setPoseFromGoal(-24, -48);
    }

    controller.clear();
    pros::delay(100);
    controller.print(0, 0, "X: %.2f", chassis.getPose().x);
    pros::delay(100);
    controller.print(1, 0, "Y: %.2f", chassis.getPose().y);
    pros::delay(100);
    controller.print(2, 0, "Theta: %.2f", chassis.getPose().theta);


    // ----------------------------------

    

    

    // ----------------------------------------------------------------------------------------------
    
    /*

    chassis.setPose(-31, -48, -90);
    pivot::set_pos(pivot::kFlippedMotorDeg);

    intake::spin(127);
    chassis.moveToPoint(-33.5, -48, 800, {.maxSpeed = 60, .minSpeed = 60});

    parallel({
        [] {
            claw::spin(127);
            macros::home();
        },
        [] {
            chassis.turnToHeading(30, 500);
            chassis.moveToPoint(-25.5, -33, 1900, {.maxSpeed = 60});
            chassis.turnToHeading(100, 500);
        },
    });

    chassis.moveToPoint(0, -48, 1500);
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
                  chassis.moveToPoint(43.5, -45.5, 1500, {.forwards = false});
              }});

    chassis.waitUntilDone();

    parallel({
        [] { macros::dual_pickup_height(1300); },
        [] {
            pros::delay(800);
            chassis.turnToPoint(22, -48, 800, {.forwards = false});
            chassis.moveToPoint(22, -48, 1000, {.forwards = false, .maxSpeed = 60});
        },
    });

    chassis.waitUntilDone();
    claw::spin(-127);
    pros::delay(300);
    pivot::move_to(pivot::kFlippedMotorDeg + 450);
    pros::delay(400);
    chassis.moveToPoint(55.25, -52, 1000);
    parallel({[] { macros::home(); },
              [] {
                  chassis.turnToPoint(56.5, -72, 500);
                  chassis.moveToPoint(55.25, -72, 1000);
              }});

    chassis.waitUntilDone();
    pros::delay(150);
    claw::spin(127);
    intake::spin(127);
    chassis.setPose(56.5, -61.9, chassis.getPose().theta);
    pros::delay(1800);
    parallel({
        [] {
            intake::stop();
            macros::go_to(subsystems::elevator::kStage3 + subsystems::elevator::kOffset);
        },
        [] {
            chassis.moveToPoint(55.25, -47.75, 1900, {.forwards = false});
            chassis.turnToHeading(90, 1500);
            chassis.moveToPoint(28, -48, 1900, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    parallel({
        [] {
            chassis.moveToPoint(55.25, -48, 1900);
            chassis.turnToHeading(180, 1500);
            chassis.moveToPoint(55.25, -63.4, 1900);
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
    chassis.setPose(56.5, -61.9, chassis.getPose().theta);
    pros::delay(1800);
    parallel({
        [] {
            intake::stop();
            macros::go_to(subsystems::elevator::kStage4 + subsystems::elevator::kOffset);
        },
        [] {
            chassis.moveToPoint(55.25, -48, 1900, {.forwards = false});
            chassis.turnToHeading(90, 1500);
            chassis.moveToPoint(28, -48, 1900, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    parallel({
        [] {
            //middle movement
            chassis.moveToPoint(55.25, -48, 1900);
            chassis.turnToHeading(180, 1500);
            chassis.moveToPoint(55.25, -63.4, 1900);
            chassis.waitUntilDone();
        },
        [] {
            pivot::move_to(pivot::kScoredMotorDeg);
            elevator::set_target(elevator::height() + elevator::kOffset);
            pros::delay(500);
            macros::go_to(subsystems::elevator::kHome);
        },
    });
    chassis.moveToPoint(55.25,-54,800,{.forwards = false,.minSpeed = 60,.earlyExitRange = 3});
    chassis.moveToPoint(0,-24,1900,{.forwards = false,.minSpeed = 60,.earlyExitRange = 3});
    chassis.turnToPoint(0,0,800,{.forwards = false});
    chassis.moveToPoint(0,0,1500,{.forwards=false});
    macros::go_to(subsystems::elevator::kStage1 + subsystems::elevator::kOffset);
    claw::spin(-127);

    */

}
}
