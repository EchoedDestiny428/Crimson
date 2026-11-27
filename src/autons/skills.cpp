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
    chassis.moveToPoint(120, 48, 1300, {.forwards = false, .maxSpeed = 40, .minSpeed = 1, .earlyExitRange = 8}); // last 80%
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
    chassis.moveToPoint(120, 48, 300, {.forwards = false}); // goal 1
    telemetry::mark(1014);
    chassis.waitUntil(0.78);
    telemetry::mark(1015);
    chassis.cancelMotion();
    telemetry::mark(1016);
    chassis.moveToPoint(120, 48, 700, {.forwards = false, .maxSpeed = 40, .minSpeed = 1, .earlyExitRange = 8}); // last 95%
    telemetry::mark(1017);
    chassis.waitUntilDone();
    telemetry::mark(1018);
    pros::delay(100);
    telemetry::mark(1019);
    chassis.setPoseFromGoal(120, 48, {.distance = 7.17});
    telemetry::mark(1020);
    chassis.donut(55, 800, {.forwards = false, .earlyExitRange = 1.5});
    telemetry::mark(1021);
    chassis.moveToPoint(118.5, 13, 1800);
    telemetry::mark(1022);
    chassis.turnToHeading(0, 1050, {.minSpeed = 1, .earlyExitRange = 1});
    telemetry::mark(1023);
    chassis.moveToPose(135.3, 12.75, 0, 800); // claw on red left loader (141.6, 12.8), note: Loader drive cut at 800 ms, about 5 in out (runs 31/34: 5 in out at 0.74-0.77 s), then chassis.pushUntilStopped at 45/127 finishes it. Before, LemLib crept the last inches to the timeout. The 0.2 slowdown is gone: the push makes the gentle contact now.
    telemetry::mark(1024);
    chassis.waitUntilDone();
    telemetry::mark(1025);
    chassis.pushUntilStopped(true, 45, 600);
    telemetry::mark(1026);
    chassis.waitUntilDone();
    telemetry::mark(1027);
    pros::delay(250);
    telemetry::mark(1028);
}

// skills-loop-test/left loop
void skills_loop_test_left_loop() {
    chassis.moveToPoint(120, 48, 1200, {.forwards = false}); // goal 1
    telemetry::mark(2001);
    chassis.waitUntil(23.06);
    telemetry::mark(2002);
    chassis.cancelMotion();
    telemetry::mark(2003);
    chassis.moveToPoint(120, 48, 1200, {.forwards = false, .maxSpeed = 40, .minSpeed = 1, .earlyExitRange = 8}); // last 40%
    telemetry::mark(2004);
    chassis.waitUntilDone();
    telemetry::mark(2005);
    pros::delay(100);
    telemetry::mark(2006);
    chassis.setPoseFromGoal(120, 48, {.distance = 7.17});
    telemetry::mark(2007);
    chassis.waitUntilDone();
    telemetry::mark(2008);
    pros::delay(250);
    telemetry::mark(2009);
    chassis.moveToPoint(125.9, 13, 1150, {.minSpeed = 1, .earlyExitRange = 2.5}); // note: Ends 2.5 in early (then the 150 ms settle) instead of waiting for LemLib's settle; turn timeout 450 (it always stopped ~2 deg short and ran out 550; the loader drive squares it).
    telemetry::mark(2010);
    chassis.waitUntilDone();
    telemetry::mark(2011);
    pros::delay(150);
    telemetry::mark(2012);
    chassis.turnToHeading(0, 450, {.minSpeed = 1, .earlyExitRange = 1});
    telemetry::mark(2013);
    chassis.moveToPose(135.3, 12.75, 0, 650); // claw on red left loader (141.6, 12.8), note: Loader drive cut at 650 ms, about 5 in out (runs 31/34: 5 in out at 0.59-0.60 s), then chassis.pushUntilStopped at 45/127 finishes it. Before, LemLib crept the last inches to the timeout.
    telemetry::mark(2014);
    chassis.waitUntilDone();
    telemetry::mark(2015);
    chassis.pushUntilStopped(true, 45, 600);
    telemetry::mark(2016);
    chassis.waitUntilDone();
    telemetry::mark(2017);
    pros::delay(250);
    telemetry::mark(2018);
}

// skills-loop-test/transition
void skills_loop_test_transition() {
    chassis.moveToPoint(128.5, 13, 900, {.forwards = false});
    telemetry::mark(3001);
    chassis.turnToHeading(65, 850, {.minSpeed = 1, .earlyExitRange = 1});
    telemetry::mark(3002);
    chassis.moveToPoint(129.5, 28, 800, {.forwards = false, .minSpeed = 40, .earlyExitRange = 4}); // note: Was (131, 28). Moved 1.5 in toward the middle so the robot still passes wall pin 4 (y 48) at about x 130.3, as before, now that point 3 sits further east.
    telemetry::mark(3003);
    chassis.moveToPoint(131.5, 77, 1500, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2}); // note: Was (130, 71). Odd spot on purpose: it makes the swing into red goal 2 steeper (line 58.8 deg, was 68.2; in the ghost the robot backs in at ~45 deg instead of ~61). Odometry reads ~3 in low in x by here (goal-2 resets moved x +2.2 to +3.1 in, runs 28-34), so the old line backed in 10-16 deg under-turned and ~2.5 in off-centre, leaning on the back aligner. Put it back if that x drift gets fixed.
    telemetry::mark(3004);
    chassis.moveToPoint(120, 96, 300, {.forwards = false, .maxSpeed = 100}); // red goal 2
    telemetry::mark(3005);
    chassis.waitUntil(4.89);
    telemetry::mark(3006);
    chassis.cancelMotion();
    telemetry::mark(3007);
    chassis.moveToPoint(120, 96, 850, {.forwards = false, .maxSpeed = 40, .minSpeed = 1, .earlyExitRange = 8}); // last 78%
    telemetry::mark(3008);
    chassis.waitUntilDone();
    telemetry::mark(3009);
    pros::delay(100);
    telemetry::mark(3010);
    chassis.setPoseFromGoal(120, 96, {.distance = 7.17});
    telemetry::mark(3011);
    chassis.donut(0, 1200, {.forwards = false, .earlyExitRange = 1.5});
    telemetry::mark(3012);
    chassis.moveToPoint(135.5, 96, 1000); // claw on yellow pin (141.8, 96)
    telemetry::mark(3013);
    chassis.moveToPoint(120, 96, 400, {.forwards = false}); // red goal 2
    telemetry::mark(3014);
    chassis.waitUntil(2.17);
    telemetry::mark(3015);
    chassis.cancelMotion();
    telemetry::mark(3016);
    chassis.moveToPoint(120, 96, 550, {.forwards = false, .maxSpeed = 40, .minSpeed = 1, .earlyExitRange = 8}); // last 86%
    telemetry::mark(3017);
    chassis.waitUntilDone();
    telemetry::mark(3018);
    pros::delay(100);
    telemetry::mark(3019);
    chassis.setPoseFromGoal(120, 96, {.distance = 7.17});
    telemetry::mark(3020);
    chassis.donut(-90, 1200, {.forwards = false, .earlyExitRange = 1.5});
    telemetry::mark(3021);
    chassis.moveToPoint(120, 113.7, 1100); // claw on yellow pin (120, 120)
    telemetry::mark(3022);
    chassis.moveToPoint(120, 96, 500, {.forwards = false, .maxSpeed = 65}); // red goal 2
    telemetry::mark(3023);
    chassis.waitUntil(4.6);
    telemetry::mark(3024);
    chassis.cancelMotion();
    telemetry::mark(3025);
    chassis.moveToPoint(120, 96, 550, {.forwards = false, .maxSpeed = 40, .minSpeed = 1, .earlyExitRange = 8}); // last 74%
    telemetry::mark(3026);
    chassis.waitUntilDone();
    telemetry::mark(3027);
    pros::delay(100);
    telemetry::mark(3028);
    chassis.setPoseFromGoal(120, 96, {.distance = 7.17});
    telemetry::mark(3029);
    chassis.waitUntilDone();
    telemetry::mark(3030);
    pros::delay(250);
    telemetry::mark(3031);
    chassis.moveToPoint(124, 131, 1750);
    telemetry::mark(3032);
    chassis.turnToHeading(0, 950, {.minSpeed = 1, .earlyExitRange = 1});
    telemetry::mark(3033);
    chassis.moveToPose(135.3, 132, 0, 750); // claw on red right loader (141.6, 132), note: Loader drive cut at 750 ms, about 5 in out (runs 31/34: 5 in out at 0.70 s), then chassis.pushUntilStopped at 45/127 finishes it. Before, LemLib crept the last inches to the timeout.
    telemetry::mark(3034);
    chassis.waitUntilDone();
    telemetry::mark(3035);
    chassis.pushUntilStopped(true, 45, 600);
    telemetry::mark(3036);
    chassis.waitUntilDone();
    telemetry::mark(3037);
    pros::delay(250);
    telemetry::mark(3038);
}

// skills-loop-test/right loop
void skills_loop_test_right_loop() {
    chassis.moveToPoint(120, 96, 1050, {.forwards = false}); // red goal 2
    telemetry::mark(4001);
    chassis.waitUntil(23.47);
    telemetry::mark(4002);
    chassis.cancelMotion();
    telemetry::mark(4003);
    chassis.moveToPoint(120, 96, 750, {.forwards = false, .maxSpeed = 40, .minSpeed = 1, .earlyExitRange = 8}); // last 40%
    telemetry::mark(4004);
    chassis.waitUntilDone();
    telemetry::mark(4005);
    pros::delay(100);
    telemetry::mark(4006);
    chassis.setPoseFromGoal(120, 96, {.distance = 7.17});
    telemetry::mark(4007);
    chassis.waitUntilDone();
    telemetry::mark(4008);
    pros::delay(250);
    telemetry::mark(4009);
    chassis.moveToPoint(124, 131, 1350, {.minSpeed = 1, .earlyExitRange = 2.5}); // note: Ends 2.5 in early (then the 150 ms settle) instead of waiting for LemLib's settle.
    telemetry::mark(4010);
    chassis.waitUntilDone();
    telemetry::mark(4011);
    pros::delay(150);
    telemetry::mark(4012);
    chassis.turnToHeading(0, 950, {.minSpeed = 1, .earlyExitRange = 1});
    telemetry::mark(4013);
    chassis.moveToPose(135.3, 132, 0, 750); // claw on red right loader (141.6, 132), note: Loader drive cut at 750 ms, about 5 in out (runs 31/34: 5 in out at 0.70-0.71 s), then chassis.pushUntilStopped at 45/127 finishes it. Before, LemLib crept the last inches to the timeout.
    telemetry::mark(4014);
    chassis.waitUntilDone();
    telemetry::mark(4015);
    chassis.pushUntilStopped(true, 45, 600);
    telemetry::mark(4016);
    chassis.waitUntilDone();
    telemetry::mark(4017);
    pros::delay(250);
    telemetry::mark(4018);
}

// skills-loop-test/ending
void skills_loop_test_ending() {
    chassis.moveToPoint(128.5, 131, 800, {.forwards = false, .maxSpeed = 40, .minSpeed = 30, .earlyExitRange = 0.5});
    telemetry::mark(5001);
    chassis.moveToPoint(109, 101, 1700, {.forwards = false, .maxSpeed = 58, .minSpeed = 50, .earlyExitRange = 7});
    telemetry::mark(5002);
    chassis.moveToPoint(105.71, 88.66, 1300, {.forwards = false});
    telemetry::mark(5003);
    chassis.turnToHeading(-26.3, 750, {.minSpeed = 1, .earlyExitRange = 1});
    telemetry::mark(5004);
    chassis.moveToPoint(72, 72, 850, {.forwards = false, .maxSpeed = 100}); // goal 0
    telemetry::mark(5005);
    chassis.waitUntil(13.16);
    telemetry::mark(5006);
    chassis.cancelMotion();
    telemetry::mark(5007);
    chassis.moveToPoint(72, 72, 850, {.forwards = false, .maxSpeed = 40, .minSpeed = 1, .earlyExitRange = 8}); // last 65%
    telemetry::mark(5008);
    chassis.waitUntilDone();
    telemetry::mark(5009);
    pros::delay(100);
    telemetry::mark(5010);
    chassis.setPoseFromGoal(72, 72, {.distance = 7.17});
    telemetry::mark(5011);
    chassis.donut(-45, 500, {.forwards = false, .earlyExitRange = 1.5}); // facing point 5
    telemetry::mark(5012);
    chassis.moveToPoint(91.55, 91.55, 1150); // claw on yellow pin (96, 96)
    telemetry::mark(5013);
    chassis.turnToPoint(72, 72, 150, {.forwards = false, .minSpeed = 1, .earlyExitRange = 1}); // facing point 6
    telemetry::mark(5014);
    chassis.moveToPoint(72, 72, 650, {.forwards = false, .maxSpeed = 80}); // goal 0
    telemetry::mark(5015);
    chassis.waitUntil(8.29);
    telemetry::mark(5016);
    chassis.cancelMotion();
    telemetry::mark(5017);
    chassis.moveToPoint(72, 72, 1050, {.forwards = false, .maxSpeed = 40, .minSpeed = 1, .earlyExitRange = 8}); // last 70%
    telemetry::mark(5018);
    chassis.waitUntilDone();
    telemetry::mark(5019);
    pros::delay(100);
    telemetry::mark(5020);
    chassis.setPoseFromGoal(72, 72, {.distance = 7.17});
    telemetry::mark(5021);
    chassis.donut(135, 2400, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .forwards = false, .earlyExitRange = 1.5});
    telemetry::mark(5022);
    chassis.moveToPoint(52.45, 52.45, 1500); // claw on yellow pin (48, 48)
    telemetry::mark(5023);
    chassis.moveToPoint(72, 72, 700, {.forwards = false, .maxSpeed = 80}); // goal 0
    telemetry::mark(5024);
    chassis.waitUntil(9.95);
    telemetry::mark(5025);
    chassis.cancelMotion();
    telemetry::mark(5026);
    chassis.moveToPoint(72, 72, 900, {.forwards = false, .maxSpeed = 40, .minSpeed = 1, .earlyExitRange = 8}); // last 64%
    telemetry::mark(5027);
    chassis.waitUntilDone();
    telemetry::mark(5028);
    pros::delay(100);
    telemetry::mark(5029);
    chassis.setPoseFromGoal(72, 72, {.distance = 7.17});
    telemetry::mark(5030);
    chassis.waitUntilDone();
    telemetry::mark(5031);
    pros::delay(250);
    telemetry::mark(5032);
    chassis.moveToPoint(28.45, 28.45, 2400); // claw on yellow pin (24, 24)
    telemetry::mark(5033);
    chassis.moveToPoint(72, 72, 1550, {.forwards = false}); // goal 0
    telemetry::mark(5034);
    chassis.waitUntil(43.11);
    telemetry::mark(5035);
    chassis.cancelMotion();
    telemetry::mark(5036);
    chassis.moveToPoint(72, 72, 950, {.forwards = false, .maxSpeed = 40, .minSpeed = 1, .earlyExitRange = 8}); // last 30%
    telemetry::mark(5037);
    chassis.waitUntilDone();
    telemetry::mark(5038);
    pros::delay(100);
    telemetry::mark(5039);
    chassis.setPoseFromGoal(72, 72, {.distance = 7.17});
    telemetry::mark(5040);
    chassis.waitUntilDone();
    telemetry::mark(5041);
    pros::delay(250);
    telemetry::mark(5042);
}

void run() {
    telemetry::route("skills-loop-test #1815ee86");
    skills_loop_test_start();
    skills_loop_test_left_loop();
    skills_loop_test_transition();
    skills_loop_test_right_loop();
    skills_loop_test_ending();
}



}
