#include "autons/routine.hpp"
#include "pros/rtos.hpp"
#include "subsystems/side_toggle.hpp"

namespace autons::match::snacky {

// Blue snacky, drive only. Blue goal 2 (24, 48) is red goal 2 (120, 96) turned 180 deg about the field centre, so the
// moves around it are copied from skills' transition / right_start at red goal 2: (x, y) -> (144 - x, 144 - y),
// heading + 180. Start like skills mirrored onto the blue wall.

void setup() {}

void run() {
    chassis.setPose(8.5, 72, -90);

    side_toggle_motors.move_relative(-8 * 360.0, 200); // 3 turns (~0.9 s at 200 rpm)
    pros::delay(900);
    side_toggle_motors.brake();

    chassis.moveToPoint(24, 48, 950, {.forwards = false}); // blue goal 2
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(24, 48, {.distance = 7.5});

    // skills transition: donut(0, 950) + moveToPoint(135.5, 96, 700)
    chassis.donut(180, 950, {.forwards = false, .slowSpeed = 26}); // facing the wall pin
    chassis.moveToPoint(8.5, 48, 700); // claw on wall pin (2.2, 48)
    chassis.moveToPoint(24, 48, 950, {.forwards = false}); // blue goal 2
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(24, 48, {.distance = 7.5});

    // skills transition's last donut(-90, 1750) + right_start's moveToPoint(120, 113.7, 750)
    chassis.donut(90, 1750, {.forwards = false, .slowSpeed = 26}); // facing pin (24, 24)
    chassis.moveToPoint(24, 30.3, 750); // claw on yellow pin (24, 24)
    chassis.moveToPoint(24, 48, 950, {.forwards = false}); // blue goal 2
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(24, 48, {.distance = 7.5});

    // skills right_start: donut(180, 1750) + moveToPoint(102.3, 96, 750)
    chassis.donut(0, 1750, {.forwards = false, .fastSpeed = 110, .slowSpeed = 26}); // facing pin (48, 48), faster than default 15
    chassis.moveToPoint(41.7, 48, 750); // claw on yellow pin (48, 48)
    chassis.turnToPoint(24,96,500,{.forwards=false});
    chassis.moveToPoint(24, 96, 950, {.forwards = false}); // neutral goal (24, 96)
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(24, 96, {.distance = 7.5});
}

}
