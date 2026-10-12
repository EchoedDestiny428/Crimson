#include "autons/routine.hpp"

namespace autons::skills::v1 {

void setup() {}

void curve(float initialX, float initialY, float finalX, float finalY, float exitAngle, float entryAngle,
           float exitLength, float entryLength,int timeout,int speed=10, int maxSpeed=127, bool forwards=true) {
    constexpr float kPi = 3.14159265358979323846f;
    const float exitRadians = exitAngle * kPi / 180.0f;
    const float entryRadians = entryAngle * kPi / 180.0f;
    const float movementDirection = forwards ? 1.0f : -1.0f;

    chassis.moveToPoint(initialX + movementDirection * sin(exitRadians) * (exitLength+2.0f),
                        initialY + movementDirection * cos(exitRadians) * (exitLength+2.0f), 800,
                        {.forwards = forwards, .maxSpeed = maxSpeed,.minSpeed = speed,.earlyExitRange = 2.0f});
    chassis.moveToPoint(finalX - movementDirection * sin(entryRadians) * entryLength,
                        finalY - movementDirection * cos(entryRadians) * entryLength, timeout,
                        {.forwards = forwards, .maxSpeed = maxSpeed,.minSpeed = speed,.earlyExitRange = 6.0f});
    chassis.turnToPoint(finalX, finalY, 600, {.forwards = forwards,.maxSpeed = maxSpeed,.minSpeed = speed,.earlyExitRange = 2.0f});
    chassis.moveToPoint(finalX, finalY, 800, {.forwards = forwards, .maxSpeed = maxSpeed});
}

void run() {
    chassis.setPose(-39, -63.4, 180);
    parallel({
        [] { macros::flip_out(10, 950); },
        [] {
            chassis.moveToPoint(-39, -48, 1000, {.forwards = false});
            chassis.turnToHeading(-90, 1000);
            chassis.moveToPoint(-30, -48, 1000, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    parallel({
        [] {
            chassis.moveToPoint(-59.25, -48, 1000);
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(-59.5, -63.4, 1000);
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
    chassis.setPose(-60, -61.9, 180);
    pros::delay(1800);
    parallel({
        [] {
            intake::stop();
            macros::go_to(subsystems::elevator::kStage1 + subsystems::elevator::kOffset);
        },
        [] {
            chassis.moveToPoint(-59.5, -47.75, 1000, {.forwards = false});
            chassis.turnToHeading(-90, 1000);
            chassis.moveToPoint(-30, -48, 1000, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    parallel({
        [] {
            chassis.moveToPoint(-59.25, -48, 1000);
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(-59.5, -63.4, 1000);
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
    chassis.setPose(-60, -61.9, 180);
    pros::delay(1800);
    parallel({
        [] {
            intake::stop();
            macros::go_to(subsystems::elevator::kStage2 + subsystems::elevator::kOffset);
        },
        [] {
            chassis.moveToPoint(-59.5, -47.75, 1000, {.forwards = false});
            chassis.turnToHeading(-90, 1000);
            chassis.moveToPoint(-30, -48, 1000, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    parallel({
        [] {
            chassis.moveToPoint(-59.25, -48, 1000);
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(-59.5, -63.4, 1000);
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
    chassis.setPose(-60, -61.9, 180);
    pros::delay(1800);
    parallel({
        [] {
            intake::stop();
            macros::go_to(subsystems::elevator::kStage3 + subsystems::elevator::kOffset);
        },
        [] {
            chassis.moveToPoint(-59.25, -47.75, 1000, {.forwards = false});
            chassis.turnToHeading(-90, 1000);
            chassis.moveToPoint(-30, -48, 1000, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    parallel({
        [] {
            chassis.moveToPoint(-59.25, -48, 1000);
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(-59.5, -63.4, 1000);
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
    chassis.setPose(-60, -61.9, 180);
    pros::delay(1800);
    parallel({
        [] {
            intake::stop();
            macros::go_to(subsystems::elevator::kStage4 + subsystems::elevator::kOffset);
        },
        [] {
            chassis.moveToPoint(-59.75, -48, 1000, {.forwards = false});
            chassis.turnToHeading(-90, 1000);
            chassis.moveToPoint(-30, -48, 1000, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    parallel({
        [] {
            chassis.moveToPoint(-59.25, -48, 1000);
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(-59.5, -63.4, 1000);
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
            chassis.moveToPoint(-59.75, -48, 1000, {.forwards = false});
            chassis.turnToHeading(-90, 1000);
            chassis.moveToPoint(-30, -48, 1000, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    // ends on intake cycle for some reason should be score last one i think, but elevator kept breaking so I left
    // stages for now middle intake
    chassis.setPose(-30, -48, -90);
    // pivot::set_pos(pivot::kFlippedMotorDeg);

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
            // curve(chassis.getPose().x, chassis.getPose().y, -25.5, -33,
            //       -90, 30, 3.0, 5.0, 1000, 20, 60);
            
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

    macros::dual_pickup_height(700);
    chassis.turnToPoint(25, -48, 800, {.forwards = false});
    chassis.moveToPoint(25, -48, 800, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(24, -48);


    claw::spin(-127);
    pros::delay(300);
    pivot::move_to(pivot::kFlippedMotorDeg + 450);
    pros::delay(400);

    chassis.moveToPoint(24, -24, 1000);
    claw::spin(127);

    parallel({[] { macros::dual_setup(); },
              [] {
                  chassis.turnToPoint(49, -51, 500, {.forwards = false});
                  chassis.moveToPoint(42, -44, 1500, {.forwards = false});
              }});

    chassis.waitUntilDone();

    parallel({
        [] { 
            macros::dual_setup(); },
        [] {
            pros::delay(800);
            chassis.turnToPoint(24, -48, 800, {.forwards = false});
            chassis.moveToPoint(24, -48, 1000, {.forwards = false, .maxSpeed = 60});
        },
    });

    chassis.waitUntilDone();
macros::dual_pickup_height(1400);
    curve(chassis.getPose().x, chassis.getPose().y, 24, -48,
          -45, 90, 1.0, 5.0, 1500, 50, 127, false);

    chassis.waitUntilDone();
    chassis.setPoseFromGoal(24, -48);

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
            intake::spin(127);
            pivot::move_to(pivot::kScoredMotorDeg);
            elevator::set_target(elevator::height() + elevator::kOffset);
            pros::delay(500);
            macros::go_to(subsystems::elevator::kHome);
        },
    });
    chassis.moveToPoint(55.25,-54,800,{.forwards = false,.minSpeed = 60,.earlyExitRange = 3});
    chassis.moveToPoint(5,-24,1900,{.forwards = false,.minSpeed = 60,.earlyExitRange = 3});
    chassis.turnToPoint(0,0,800,{.forwards = false});
    chassis.moveToPoint(0,0,1500,{.forwards=false});
    macros::go_to(subsystems::elevator::kStage1 + subsystems::elevator::kOffset);
    claw::spin(-127);

}
}
