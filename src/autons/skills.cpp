#include "autons/routine.hpp"
#include "subsystems/side_toggle.hpp"

namespace autons::skills {

void setup() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
}

// skills-loop-test, made in PointWeaver.
// Field frame: (0, 0) is the corner on blue's right, +X toward red, heading 0 faces red, clockwise. Inches.

// skills-loop-test/start
void skills_loop_test_start() {
    chassis.setPose(135, 72, -90);
    pros::delay(1000);
    telemetry::mark(1001);
    chassis.moveToPoint(120, 48, 550, {.forwards = false, .maxSpeed = 100}); // goal 1
    telemetry::mark(1002);
    chassis.waitUntil(5.66);
    telemetry::mark(1003);
    chassis.cancelMotion();
    telemetry::mark(1004);
    chassis.moveToPoint(120, 48, 950, {.forwards = false, .maxSpeed = 50, .minSpeed = 1, .earlyExitRange = 8}); // last 80%
    telemetry::mark(1005);
    chassis.waitUntilDone();
    telemetry::mark(1006);
    pros::delay(100);
    telemetry::mark(1007);
    chassis.setPoseFromGoal(120, 48, {.distance = 7.5});
    telemetry::mark(1008);
    chassis.donut(0, 1400, {.forwards = false, .earlyExitRange = 1.5});
    telemetry::mark(1009);
    chassis.moveToPoint(135.5, 48, 500); // claw on yellow pin (141.8, 48)
    telemetry::mark(1010);
    chassis.waitUntil(4);
    telemetry::mark(1011);
    chassis.cancelMotion();
    telemetry::mark(1012);
    chassis.moveToPoint(135.5, 48, 300, {.maxSpeed = 50, .minSpeed = 1, .earlyExitRange = 2}); // last 50%
    telemetry::mark(1013);
    chassis.waitUntilDone();
    telemetry::mark(1014);
    pros::delay(500);
    telemetry::mark(1015);
    chassis.moveToPoint(120, 48, 400, {.forwards = false}); // goal 1
    telemetry::mark(1016);
    chassis.waitUntil(2.17);
    telemetry::mark(1017);
    chassis.cancelMotion();
    telemetry::mark(1018);
    chassis.moveToPoint(120, 48, 500, {.forwards = false, .maxSpeed = 50, .minSpeed = 1, .earlyExitRange = 8}); // last 86%
    telemetry::mark(1019);
    chassis.waitUntilDone();
    telemetry::mark(1020);
    pros::delay(100);
    telemetry::mark(1021);
    chassis.setPoseFromGoal(120, 48, {.distance = 7.5});
    telemetry::mark(1022);
    chassis.donut(55, 1600, {.forwards = false, .earlyExitRange = 1.5});
    telemetry::mark(1023);
    chassis.moveToPoint(114.6, 13, 1500, {.minSpeed = 1, .earlyExitRange = 2});
    telemetry::mark(1024);
    chassis.waitUntilDone();
    telemetry::mark(1025);
    pros::delay(150);
    telemetry::mark(1026);
    chassis.turnToHeading(0, 1100, {.minSpeed = 1, .earlyExitRange = 1});
    telemetry::mark(1027);
    chassis.moveToPose(135.3, 13, 0, 1000); // claw on red left loader (141.6, 13)
    telemetry::mark(1028);
    chassis.waitUntil(16.56);
    telemetry::mark(1029);
    chassis.cancelMotion();
    telemetry::mark(1030);
    chassis.moveToPoint(135.3, 13, 300, {.maxSpeed = 50, .minSpeed = 1, .earlyExitRange = 2}); // last 20%
    telemetry::mark(1031);
    chassis.waitUntilDone();
    telemetry::mark(1032);
    pros::delay(730);
    telemetry::mark(1033);
}

// skills-loop-test/left loop
void skills_loop_test_left_loop() {
    chassis.moveToPoint(120, 48, 750, {.forwards = false}); // goal 1
    telemetry::mark(2001);
    chassis.waitUntil(11.46);
    telemetry::mark(2002);
    chassis.cancelMotion();
    telemetry::mark(2003);
    chassis.moveToPoint(120, 48, 950, {.forwards = false, .maxSpeed = 50, .minSpeed = 1, .earlyExitRange = 8}); // last 70%
    telemetry::mark(2004);
    chassis.waitUntilDone();
    telemetry::mark(2005);
    pros::delay(100);
    telemetry::mark(2006);
    chassis.setPoseFromGoal(120, 48, {.distance = 7.5});
    telemetry::mark(2007);
    chassis.moveToPoint(125.9, 13, 1400, {.minSpeed = 1, .earlyExitRange = 2});
    telemetry::mark(2008);
    chassis.waitUntilDone();
    telemetry::mark(2009);
    pros::delay(150);
    telemetry::mark(2010);
    chassis.turnToHeading(0, 950, {.minSpeed = 1, .earlyExitRange = 1});
    telemetry::mark(2011);
    chassis.moveToPoint(135.3, 13, 700, {.minSpeed = 1, .earlyExitRange = 2}); // claw on red left loader (141.6, 13)
    telemetry::mark(2012);
}

void run() {
    telemetry::route("skills-loop-test #d0b9409c");
    skills_loop_test_start();
    skills_loop_test_left_loop();
}



}
