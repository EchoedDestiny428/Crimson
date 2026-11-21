#include "autons/routine.hpp"

namespace autons::skills {

void setup() {}

// skills, made in PointWeaver.
// Field frame: (0, 0) is the corner on blue's right, +X toward red, heading 0 faces red, clockwise. Inches.

// start
void start() {
    chassis.setPose(135.5, 72, -90);
    chassis.moveToPoint(120, 48, 1200, {.forwards = false}); // goal 1
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 48, {.distance = 7.5});
    chassis.donut(0, 800, {.forwards = false});
    chassis.moveToPoint(135.5, 48, 700); // claw on yellow pin (141.8, 48)
    chassis.moveToPoint(120, 48, 900, {.forwards = false}); // goal 1
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 48, {.distance = 7.5});
    chassis.donut(45, 750, {.forwards = false}); // facing point 4
    chassis.moveToPoint(132, 36, 700);
    chassis.turnToPoint(108, 12, 750, {.forwards = false}); // facing point 5
    chassis.moveToPoint(108, 12, 1300, {.forwards = false});
    chassis.turnToPoint(141.6, 12, 550); // facing point 6
    chassis.moveToPose(135.3, 12, 0, 1400); // claw on red loader (141.6, 12)
}

// left loop
void left_loop() {
    chassis.moveToPoint(125, 12, 750, {.forwards = false});
    chassis.turnToPoint(120, 48, 700, {.forwards = false}); // facing point 2
    chassis.moveToPoint(120, 48, 1400, {.forwards = false}); // goal 1
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 48, {.distance = 7.5});
    chassis.moveToPoint(125, 12, 1200);
    chassis.turnToPoint(141.6, 12, 700); // facing point 4
    chassis.moveToPoint(135.3, 12, 750); // claw on red loader (141.6, 12)
}

// transition
void transition() {
    chassis.moveToPoint(131, 12, 500, {.forwards = false});
    chassis.turnToPoint(131, 84, 750, {.forwards = false}); // facing point 2
    chassis.moveToPoint(131, 84, 2200, {.forwards = false});
    chassis.turnToPoint(120, 96, 550, {.forwards = false}); // facing point 3
    chassis.moveToPoint(120, 96, 900, {.forwards = false}); // red goal 2
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});
    chassis.donut(0, 800, {.forwards = false}); // facing point 4
    chassis.moveToPoint(135.5, 96, 650); // claw on yellow pin (141.8, 96)
    chassis.moveToPoint(120, 96, 900, {.forwards = false}); // red goal 2
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});
    chassis.donut(-90, 1350, {.forwards = false});
}

// right start
void right_start() {
    chassis.moveToPoint(120, 113.7, 750); // claw on yellow pin (120, 120)
    chassis.moveToPoint(120, 96, 950, {.forwards = false}); // red goal 2
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});
    chassis.donut(180, 1350, {.forwards = false}); // facing point 3
    chassis.moveToPoint(102.3, 96, 750); // claw on yellow pin (96, 96)
    chassis.moveToPoint(120, 96, 950, {.forwards = false}); // red goal 2
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});
    chassis.donut(-82.09, 1450, {.forwards = false}); // facing point 5
    chassis.moveToPoint(125, 132, 1200);
    chassis.turnToPoint(141.6, 132, 700); // facing point 6
    chassis.moveToPoint(135.3, 132, 750); // claw on red loader (141.6, 132)
}

// right loop
void right_loop() {
    chassis.moveToPoint(125, 132, 750, {.forwards = false});
    chassis.turnToPoint(120, 96, 700, {.forwards = false}); // facing point 2
    chassis.moveToPoint(120, 96, 1400, {.forwards = false}); // red goal 2
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});
    chassis.moveToPoint(125, 132, 1200);
    chassis.turnToPoint(141.6, 132, 700); // facing point 4
    chassis.moveToPoint(135.3, 132, 750); // claw on red loader (141.6, 132)
}

// ending
void ending() {
    chassis.moveToPoint(132, 132, 450, {.forwards = false});
    chassis.turnToPoint(72, 72, 550, {.forwards = false}); // facing point 2
    chassis.moveToPoint(72, 72, 2500, {.forwards = false}); // goal 0
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(72, 72, {.distance = 7.5});
    chassis.donut(135, 2450, {.forwards = false}); // facing point 3
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
    // left_loop();
    // transition();
    // right_start();
    // right_loop();
    // ending();
}


}
