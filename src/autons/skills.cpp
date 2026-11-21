#include "autons/routine.hpp"
#include "subsystems/side_toggle.hpp"

namespace autons::skills {

void setup() {}

// skills, made in PointWeaver.
// Field frame: (0, 0) is the corner on blue's right, +X toward red, heading 0 faces red, clockwise. Inches.

// start
void start() {
    chassis.setPose(135.5, 72, -90);

    side_toggle_motors.move_velocity(-200); 
    pros::delay(1200);
    side_toggle_motors.brake();

    chassis.moveToPoint(120, 48, 600, {.forwards = false}); // goal 1
    chassis.waitUntil(10);
    chassis.cancelMotion();
    chassis.moveToPoint(120, 48, 1050, {.forwards = false, .maxSpeed = 50}); // last 65%
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 48, {.distance = 7.5});
    chassis.donut(0, 1000, {.forwards = false});
    chassis.moveToPoint(135.5, 48, 500); // claw on yellow pin (141.8, 48)
    chassis.waitUntil(5.6);
    chassis.cancelMotion();
    chassis.moveToPoint(135.5, 48, 300, {.maxSpeed = 50}); // last 30%
    chassis.moveToPoint(120, 48, 550, {.forwards = false, .maxSpeed = 50}); // goal 1
    chassis.waitUntil(6.2);
    chassis.cancelMotion();
    chassis.moveToPoint(120, 48, 750, {.forwards = false, .maxSpeed = 50}); // last 60%
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 48, {.distance = 7.5});
    chassis.donut(45, 950, {.forwards = false}); // facing point 4
    chassis.moveToPoint(132, 36, 700);
    chassis.turnToPoint(108, 12, 750, {.forwards = false}); // facing point 5
    chassis.moveToPoint(108, 12, 1300, {.forwards = false});
    chassis.turnToPoint(141.6, 13, 550); // facing point 6
    chassis.moveToPose(135.3, 13, 0, 900); // claw on red left loader (141.6, 13)
    chassis.waitUntil(21.85);
    chassis.cancelMotion();
    chassis.moveToPose(135.3, 13, 0, 650, {.maxSpeed = 50}); // last 20%
}

// left loop
void left_loop() {
    chassis.moveToPoint(125, 12, 750, {.forwards = false});
    chassis.turnToPoint(120, 48, 750, {.forwards = false}); // facing point 2
    chassis.moveToPoint(120, 48, 700, {.forwards = false}); // goal 1
    chassis.waitUntil(14.54);
    chassis.cancelMotion();
    chassis.moveToPoint(120, 48, 1200, {.forwards = false, .maxSpeed = 50}); // last 60%
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 48, {.distance = 7.5});
    chassis.moveToPoint(125, 12, 1200);
    chassis.turnToPoint(141.6, 13, 700); // facing point 4
    chassis.moveToPose(135.3, 13, 0, 600); // claw on red left loader (141.6, 13)
    chassis.waitUntil(8.28);
    chassis.cancelMotion();
    chassis.moveToPose(135.3, 13, 0, 750, {.maxSpeed = 50}); // last 20%
}

// transition
void transition() {
    chassis.moveToPoint(132.5, 12, 2150, {.forwards = false});
    chassis.turnToPoint(132.5, 84, 750, {.forwards = false}); // facing point 2
    chassis.moveToPoint(132.5, 84, 2200, {.forwards = false});
    chassis.turnToPoint(120, 96, 550, {.forwards = false}); // facing point 3
    chassis.moveToPoint(120, 96, 1300, {.forwards = false, .maxSpeed = 50}); // red goal 2
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});
    chassis.donut(0, 950, {.forwards = false}); // facing point 4
    chassis.moveToPoint(135.5, 96, 700); // claw on yellow pin (141.8, 96)
    chassis.moveToPoint(120, 96, 1200, {.forwards = false, .maxSpeed = 50}); // red goal 2
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});
    chassis.donut(-90, 1750, {.forwards = false});
}

// right start
void right_start() {
    chassis.moveToPoint(120, 113.7, 750); // claw on yellow pin (120, 120)
    chassis.moveToPoint(120, 96, 950, {.forwards = false}); // red goal 2
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});
    chassis.donut(180, 1750, {.forwards = false}); // facing point 3
    chassis.moveToPoint(102.3, 96, 750); // claw on yellow pin (96, 96)
    chassis.moveToPoint(120, 96, 950, {.forwards = false}); // red goal 2
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});
    chassis.donut(-82.09, 1900, {.forwards = false}); // facing point 5
    chassis.moveToPoint(125, 132, 1200);
    chassis.turnToPoint(141.6, 131, 700); // facing point 6
    chassis.moveToPoint(135.31, 131.38, 750); // claw on red right loader (141.6, 131)
}

// right loop
void right_loop() {
    chassis.moveToPoint(125, 132, 750, {.forwards = false});
    chassis.turnToPoint(120, 96, 750, {.forwards = false}); // facing point 2
    chassis.moveToPoint(120, 96, 1350, {.forwards = false}); // red goal 2
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});
    chassis.moveToPoint(125, 132, 1200);
    chassis.turnToPoint(141.6, 131, 700); // facing point 4
    chassis.moveToPose(135.3, 131, 0, 1250); // claw on red right loader (141.6, 131)
}

// ending
void ending() {
    chassis.moveToPoint(132, 132, 1000, {.forwards = false});
    chassis.turnToPoint(72, 72, 600, {.forwards = false}); // facing point 2
    chassis.moveToPoint(72, 72, 2500, {.forwards = false}); // goal 0
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(72, 72, {.distance = 7.5});
    chassis.donut(135, 3250, {.forwards = false}); // facing point 3
    chassis.moveToPoint(52.45, 52.45, 1000); // claw on yellow pin (48, 48)
    chassis.moveToPoint(72, 72, 1200, {.forwards = false}); // goal 0
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(72, 72, {.distance = 7.5});
    chassis.moveToPoint(28.45, 28.45, 1800); // claw on yellow pin (24, 24)
    chassis.moveToPoint(72, 72, 1950, {.forwards = false}); // goal 0
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(72, 72, {.distance = 7.5});
}

void run() {
    start();
    left_loop();
    transition();
    right_start();
    // right_loop();
    // ending();
}




}
