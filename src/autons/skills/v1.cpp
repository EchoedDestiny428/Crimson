#include "autons/routine.hpp"

namespace autons::skills::v1 {

void setup() {}

void run() {
    // chassis.setPose(-39, -63.4, 180);
    // parallel({
    //     [] { macros::flip_out(10, 950); },
    //     [] {
    //         chassis.moveToPoint(-39, -48, 1000, {.forwards = false});
    //         chassis.turnToHeading(-90, 1000);
    //         chassis.moveToPoint(-30, -48, 1000, {.forwards = false});
    //         chassis.waitUntilDone();
    //     },
    // });
    // claw::spin(-127);
    // pros::delay(125);
    // parallel({
    //     [] {
    //         chassis.moveToPoint(-47.25, -48, 1000);
    //         chassis.turnToHeading(180, 1000);
    //         chassis.moveToPoint(-47.5, -63.4, 1000);
    //         chassis.waitUntilDone();
    //     },
    //     [] {
    //         pivot::move_to(pivot::kScoredMotorDeg);
    //         elevator::set_target(elevator::height() + elevator::kOffset);
    //         pros::delay(500);
    //         macros::go_to(subsystems::elevator::kHome);
    //     },
    // });
    // pros::delay(150);
    // claw::spin(127);
    // intake::spin(127);
    // chassis.setPose(-48, -61.9, 180);
    // pros::delay(1800);
    // parallel({
    //     [] {
    //         intake::stop();
    //         macros::go_to(subsystems::elevator::kStage1+subsystems::elevator::kOffset);
    //     },
    //     [] {
    //         chassis.moveToPoint(-47.5, -47.75, 1000, {.forwards = false});
    //         chassis.turnToHeading(-90, 1000);
    //         chassis.moveToPoint(-30, -48, 1000, {.forwards = false});
    //         chassis.waitUntilDone();
    //     },
    // });
    // claw::spin(-127);
    // pros::delay(125);
    // parallel({
    //     [] {
    //         chassis.moveToPoint(-47.25, -48, 1000);
    //         chassis.turnToHeading(180, 1000);
    //         chassis.moveToPoint(-47.5, -63.4, 1000);
    //         chassis.waitUntilDone();
    //     },
    //     [] {
    //         pivot::move_to(pivot::kScoredMotorDeg);
    //         elevator::set_target(elevator::height() + elevator::kOffset);
    //         pros::delay(500);
    //         macros::go_to(subsystems::elevator::kHome);
    //     },
    // });
    // pros::delay(150);
    // claw::spin(127);
    // intake::spin(127);
    // chassis.setPose(-48, -61.9, 180);
    // pros::delay(1800);
    // parallel({
    //     [] {
    //         intake::stop();
    //         macros::go_to(subsystems::elevator::kStage2+subsystems::elevator::kOffset);
    //     },
    //     [] {
    //         chassis.moveToPoint(-47.5, -47.75, 1000, {.forwards = false});
    //         chassis.turnToHeading(-90, 1000);
    //         chassis.moveToPoint(-30, -48, 1000, {.forwards = false});
    //         chassis.waitUntilDone();
    //     },
    // });
    // claw::spin(-127);
    // pros::delay(125);
    // parallel({
    //     [] {
    //         chassis.moveToPoint(-47.25, -48, 1000);
    //         chassis.turnToHeading(180, 1000);
    //         chassis.moveToPoint(-47.5, -63.4, 1000);
    //         chassis.waitUntilDone();
    //     },
    //     [] {
    //         pivot::move_to(pivot::kScoredMotorDeg);
    //         elevator::set_target(elevator::height() + elevator::kOffset);
    //         pros::delay(500);
    //         macros::go_to(subsystems::elevator::kHome);
    //     },
    // });
    // pros::delay(150);
    // claw::spin(127);
    // intake::spin(127);
    // chassis.setPose(-48, -61.9, 180);
    // pros::delay(1800);
    // parallel({
    //     [] {
    //         intake::stop();
    //         macros::go_to(subsystems::elevator::kStage3+subsystems::elevator::kOffset);
    //     },
    //     [] {
    //         chassis.moveToPoint(-47.25, -47.75, 1000, {.forwards = false});
    //         chassis.turnToHeading(-90, 1000);
    //         chassis.moveToPoint(-30, -48, 1000, {.forwards = false});
    //         chassis.waitUntilDone();
    //     },
    // });
    // claw::spin(-127);
    // pros::delay(125);
    // parallel({
    //     [] {
    //         chassis.moveToPoint(-47.25, -48, 1000);
    //         chassis.turnToHeading(180, 1000);
    //         chassis.moveToPoint(-47.5, -63.4, 1000);
    //         chassis.waitUntilDone();
    //     },
    //     [] {
    //         pivot::move_to(pivot::kScoredMotorDeg);
    //         elevator::set_target(elevator::height() + elevator::kOffset);
    //         pros::delay(500);
    //         macros::go_to(subsystems::elevator::kHome);
    //     },
    // });
    // pros::delay(150);
    // claw::spin(127);
    // intake::spin(127);
    // chassis.setPose(-48, -61.9, 180);
    // pros::delay(1800);
    // parallel({
    //     [] {
    //         intake::stop();
    //         macros::go_to(subsystems::elevator::kStage4+subsystems::elevator::kOffset);
    //     },
    //     [] {
    //         chassis.moveToPoint(-47.75, -48, 1000, {.forwards = false});
    //         chassis.turnToHeading(-90, 1000);
    //         chassis.moveToPoint(-30, -48, 1000, {.forwards = false});
    //         chassis.waitUntilDone();
    //     },
    // });
    // claw::spin(-127);
    // pros::delay(125);
    // parallel({
    //     [] {
    //         chassis.moveToPoint(-47.25, -48, 1000);
    //         chassis.turnToHeading(180, 1000);
    //         chassis.moveToPoint(-47.5, -63.4, 1000);
    //         chassis.waitUntilDone();
    //     },
    //     [] {
    //         pivot::move_to(pivot::kScoredMotorDeg);
    //         elevator::set_target(elevator::height() + elevator::kOffset);
    //         pros::delay(500);
    //         macros::go_to(subsystems::elevator::kHome);
    //     },
    // });

    //ends on intake cycle for some reason should be score last one i think, but elevator kept breaking so I left stages for now
    //middle intake
    chassis.setPose(-31.5,-48,-90);
    chassis.moveToPoint(-35.5,-48,500);
    chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
    parallel({
        [] {
        chassis.swingToPoint(-24,-28,lemlib::DriveSide::RIGHT,800);
        chassis.moveToPoint(-24,-28,800);
        chassis.waitUntilDone();
        chassis.moveToPoint(0,-48,800);
        },
        [] {
            macros::go_to(subsystems::elevator::kHome);
            intake::spin(127);
        }
    }); 
}

}
