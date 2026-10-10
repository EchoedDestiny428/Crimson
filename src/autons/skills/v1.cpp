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

    for (const double stage : {subsystems::elevator::kStage1, subsystems::elevator::kStage2, subsystems::elevator::kStage3, subsystems::elevator::kStage4}) {
        parallel({
            [] {
                chassis.moveToPoint(-55, -48, 1200, {.maxSpeed = 100});
                chassis.turnToPoint(-58, -72, 600);
                chassis.moveToPoint(-58, -72, 400);
                chassis.moveToPoint(-58, -72, 1000, {.maxSpeed = 50}); //  this is the loading timeout
                chassis.waitUntilDone();
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
        chassis.moveToPoint(-55, -48, 700, {.forwards = false});

        parallel({
            [stage] {
                intake::stop();
                macros::flip_out(stage + 350, pivot::kFlippedMotorDeg + 50);
            },
            [] {
                chassis.turnToPoint(-24, -48, 500, {.forwards = false});
                chassis.moveToPoint(-24, -48, 600, {.forwards = false});
                chassis.moveToPoint(-24, -48, 1000, {.forwards = false, .maxSpeed = 30});
            },
        });

        chassis.waitUntilDone();

        claw::spin(-127);
        pros::delay(300);
        chassis.setPoseFromGoal(-24, -48);
    }

    pivot::move_to(pivot::kScoredMotorDeg);
    elevator::set_target(elevator::height() + elevator::kOffset);



    // ----------------------------------------------------------------------------------------------

    chassis.setPose(-31, -48, -90);
    pivot::set_pos(pivot::kFlippedMotorDeg);
    
    claw::spin(127);
    intake::spin(127);

    parallel({
        [] {
            macros::home();
        },
        [] {
            chassis.moveToPoint(-33.5, -48, 800, {.maxSpeed = 60, .minSpeed = 60});
            chassis.turnToHeading(30, 500);
            chassis.moveToPoint(-25.5, -33, 650, {.maxSpeed = 70});
            chassis.moveToPoint(-25.5, -33, 1000, {.maxSpeed = 20});
            
            chassis.turnToHeading(100, 500);
        },
    });

    chassis.moveToPoint(0, -48, 1500);
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(0, -72, 800);

    parallel({
        [] { macros::flip_out(10, 950); },
        [] {
            chassis.moveToPoint(0, -60, 500, {.forwards = false, .minSpeed = 127});
            chassis.moveToPoint(0, -72, 500, {.minSpeed = 127});
            chassis.turnToPoint(24, -48, 500, {.forwards = false});
            chassis.moveToPoint(24, -48, 1100, {.forwards = false, .maxSpeed = 80});
        },
    });

    chassis.waitUntilDone();
    chassis.setPoseFromGoal(24, -48);

    claw::spin(-127);
    pros::delay(500);

    parallel({[] {
                  macros::dual_setup();
                  claw::spin(127);
              },
              [] {
                  chassis.moveToPoint(0, -48, 1000);
                  chassis.turnToPoint(24, -24, 500, {.forwards = false});
                  chassis.moveToPoint(19.5, -29.5, 1000, {.forwards = false});
                  chassis.waitUntilDone();
              }});

    parallel({
        [] { macros::dual_pickup_height(700); },
        [] {
            pros::delay(800);
            chassis.turnToPoint(25, -48, 800, {.forwards = false});
        },
    });

    chassis.moveToPoint(25, -48, 800, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(24, -48);


    claw::spin(-127);
    pros::delay(300);
    pivot::move_to(pivot::kFlippedMotorDeg + 450);
    pros::delay(400);

    chassis.moveToPoint(24, -24, 1000);
    claw::spin(127);

    parallel({
        [] { 
            macros::dual_setup(); },
        [] {
            chassis.turnToPoint(48, -48, 500, {.forwards = false});
            chassis.moveToPoint(43.5, -43.5, 1500, {.forwards = false});
        }
    });

    chassis.waitUntilDone();

    parallel({
        [] { macros::dual_pickup_height(1400); },
        [] {
            pros::delay(800);
            chassis.turnToPoint(24, -48, 800, {.forwards = false});
            chassis.moveToPoint(24, -48, 1000, {.forwards = false, .maxSpeed = 60});
        }
    });

    chassis.waitUntilDone();
    chassis.setPoseFromGoal(24, -48);

    claw::spin(-127);
    pros::delay(300);
    pivot::move_to(pivot::kFlippedMotorDeg + 450);
    
    
    controller.clear();
    pros::delay(100);
    controller.print(0, 0, "X: %.2f", chassis.getPose().x);
    pros::delay(100);
    controller.print(1, 0, "Y: %.2f", chassis.getPose().y);
    pros::delay(100);
    controller.print(2, 0, "pivot: %.2f", pivot::get_pos());



}
}
