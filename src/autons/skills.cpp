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
    chassis.moveToPoint(120, 48, 1050, {.forwards = false, .maxSpeed = 50, .minSpeed = 1, .earlyExitRange = 8}); // last 80%
    telemetry::mark(1005);
    chassis.waitUntilDone();
    telemetry::mark(1006);
    pros::delay(100);
    telemetry::mark(1007);
    chassis.setPoseFromGoal(120, 48, {.distance = 7.17});
    telemetry::mark(1008);
    chassis.donut(0, 900, {.forwards = false, .earlyExitRange = 1.5});
    telemetry::mark(1009);
    chassis.moveToPoint(135.5, 48, 600); // claw on yellow pin (141.8, 48)
    telemetry::mark(1010);
    chassis.waitUntil(5.83);
    telemetry::mark(1011);
    chassis.cancelMotion();
    telemetry::mark(1012);
    chassis.moveToPoint(135.5, 48, 500, {.maxSpeed = 50}); // last 30%
    telemetry::mark(1013);
    chassis.waitUntilDone();
    telemetry::mark(1014);
    pros::delay(500);
    telemetry::mark(1015);
    chassis.moveToPoint(120, 48, 450, {.forwards = false}); // goal 1
    telemetry::mark(1016);
    chassis.waitUntil(3.1);
    telemetry::mark(1017);
    chassis.cancelMotion();
    telemetry::mark(1018);
    chassis.moveToPoint(120, 48, 400, {.forwards = false, .maxSpeed = 50, .minSpeed = 1, .earlyExitRange = 8}); // last 80%
    telemetry::mark(1019);
    chassis.waitUntilDone();
    telemetry::mark(1020);
    pros::delay(100);
    telemetry::mark(1021);
    chassis.setPoseFromGoal(120, 48, {.distance = 7.17});
    telemetry::mark(1022);
    chassis.donut(55, 800, {.forwards = false, .earlyExitRange = 1.5});
    telemetry::mark(1023);
    chassis.moveToPoint(118.5, 13, 1800);
    telemetry::mark(1024);
    chassis.turnToHeading(0, 1050, {.minSpeed = 1, .earlyExitRange = 1});
    telemetry::mark(1025);
    chassis.moveToPose(135.3, 13, 0, 950); // claw on red left loader (141.6, 13)
    telemetry::mark(1026);
    chassis.waitUntil(13.44);
    telemetry::mark(1027);
    chassis.cancelMotion();
    telemetry::mark(1028);
    chassis.moveToPose(135.3, 13, 0, 550, {.maxSpeed = 50}); // last 20%
    telemetry::mark(1029);
    chassis.waitUntilDone();
    telemetry::mark(1030);
    pros::delay(500);
    telemetry::mark(1031);
}

// skills-loop-test/left loop
void skills_loop_test_left_loop() {
    chassis.moveToPoint(120, 48, 1200, {.forwards = false}); // goal 1
    telemetry::mark(2001);
    chassis.waitUntil(28.2);
    telemetry::mark(2002);
    chassis.cancelMotion();
    telemetry::mark(2003);
    chassis.moveToPoint(120, 48, 1200, {.forwards = false, .maxSpeed = 50, .minSpeed = 1, .earlyExitRange = 8}); // last 26%
    telemetry::mark(2004);
    chassis.waitUntilDone();
    telemetry::mark(2005);
    pros::delay(100);
    telemetry::mark(2006);
    chassis.setPoseFromGoal(120, 48, {.distance = 7.17});
    telemetry::mark(2007);
    chassis.waitUntilDone();
    telemetry::mark(2008);
    pros::delay(500);
    telemetry::mark(2009);
    chassis.moveToPoint(125.9, 13, 1150);
    telemetry::mark(2010);
    chassis.turnToHeading(0, 550, {.minSpeed = 1, .earlyExitRange = 1});
    telemetry::mark(2011);
    chassis.moveToPose(135.3, 13, 0, 1150); // claw on red left loader (141.6, 13)
    telemetry::mark(2012);
    chassis.waitUntilDone();
    telemetry::mark(2013);
    pros::delay(500);
    telemetry::mark(2014);
}

void run() {
    telemetry::route("skills-loop-test #30be7acb");
    skills_loop_test_start();
    skills_loop_test_left_loop();
}


}
