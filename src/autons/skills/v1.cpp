#include "autons/routine.hpp"

namespace autons::skills::v1 {

void setup() {}

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
            chassis.moveToPoint(-47.25, -48, 1000);
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(-47.5, -63.4, 1000);
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
    pros::delay(1800);
    parallel({
        [] {
            intake::stop();
            macros::go_to(subsystems::elevator::kStage1+subsystems::elevator::kOffset);
        },
        [] {
            chassis.moveToPoint(-47.5, -47.75, 1000, {.forwards = false});
            chassis.turnToHeading(-90, 1000);
            chassis.moveToPoint(-30, -48, 1000, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    parallel({
        [] {
            chassis.moveToPoint(-47.25, -48, 1000);
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(-47.5, -63.4, 1000);
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
    pros::delay(1800);
    parallel({
        [] {
            intake::stop();
            macros::go_to(subsystems::elevator::kStage2+subsystems::elevator::kOffset);
        },
        [] {
            chassis.moveToPoint(-47.5, -47.75, 1000, {.forwards = false});
            chassis.turnToHeading(-90, 1000);
            chassis.moveToPoint(-30, -48, 1000, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    parallel({
        [] {
            chassis.moveToPoint(-47.25, -48, 1000);
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(-47.5, -63.4, 1000);
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
    pros::delay(1800);
    parallel({
        [] {
            intake::stop();
            macros::go_to(subsystems::elevator::kStage3+subsystems::elevator::kOffset);
        },
        [] {
            chassis.moveToPoint(-47.25, -47.75, 1000, {.forwards = false});
            chassis.turnToHeading(-90, 1000);
            chassis.moveToPoint(-30, -48, 1000, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    parallel({
        [] {
            chassis.moveToPoint(-47.25, -48, 1000);
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(-47.75, -63.4, 1000);
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
    pros::delay(1800);
    parallel({
        [] {
            intake::stop();
            macros::go_to(subsystems::elevator::kStage4+subsystems::elevator::kOffset);
        },
        [] {
            chassis.moveToPoint(-47.75, -48, 1000, {.forwards = false});
            chassis.turnToHeading(-90, 1000);
            chassis.moveToPoint(-30, -48, 1000, {.forwards = false});
            chassis.waitUntilDone();
        },
    });
    claw::spin(-127);
    pros::delay(125);
    parallel({
        [] {
            chassis.moveToPoint(-47.25, -48, 1000);
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(-47.75, -63.4, 1000);
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
