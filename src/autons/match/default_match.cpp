#include "autons/routine.hpp"
#include "config/ports.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdint>

namespace autons::match::default_match {

namespace {

constexpr double kArriveLiftCm = 1.5;
constexpr double kArriveArmDeg = 4.0;
constexpr std::uint32_t kDropMs = 250;
constexpr float kGrabWithinIn = 1.0f;
constexpr int kNeutral = 1;
constexpr macros::Pose kWallPin{6.0, 30.0};
constexpr double kStillRpm = 10.0;
constexpr double kClawLowDeg = 45.0;
constexpr double kPreloadHoverDeg = 25.0;

bool lift_arrived() {
    return std::fabs(macros::pose_now().lift_cm - macros::target_now().lift_cm) < kArriveLiftCm;
}

bool arrived() {
    return lift_arrived() && std::fabs(macros::pose_now().arm_deg - macros::target_now().arm_deg) < kArriveArmDeg;
}

bool arrived_still() {
    const double rpm = arm_pivot_motor.get_actual_velocity();
    return arrived() && (!std::isfinite(rpm) || std::fabs(rpm) < kStillRpm);
}

macros::Pose preload_pose() {
    macros::Pose pose = macros::stage(1);
    pose.lift_cm -= 1.75;
    return pose;
}

macros::Pose preload_hover() {
    macros::Pose pose = preload_pose();
    pose.arm_deg -= kPreloadHoverDeg;
    return pose;
}

template <typename Done> void wait_until(Done done, std::uint32_t timeout_ms) {
    const std::uint32_t start = pros::millis();
    while (!done() && pros::millis() - start < timeout_ms) {
        pros::delay(10);
    }
}

void wait_arrived(std::uint32_t timeout_ms) {
    wait_until(arrived, timeout_ms);
}

void wait_claw_low() {
    wait_until([] { return macros::pose_now().arm_deg < kClawLowDeg && lift_arrived(); }, 2000);
}

void score() {
    wait_until(arrived_still, 1500);
    claw::release();
    pros::delay(kDropMs);
    macros::home_async();
}

void score_preload() {
    macros::aim(preload_pose());
    wait_until(arrived_still, 800);
    claw::release();
    pros::delay(kDropMs);
    macros::home_async();
}

void grab_near(float x, float y) {
    while (chassis.isInMotion()) {
        const lemlib::Pose pose = chassis.getPose();
        if (std::hypot(pose.x - x, pose.y - y) <= kGrabWithinIn) {
            break;
        }
        pros::delay(10);
    }
    claw::grab();
}

void grab_on_arrival() {
    chassis.waitUntilDone();
    claw::grab();
}

void release_when_arrived() {
    wait_until(arrived_still, 1500);
    claw::release();
}

void flick_side_toggles() {
    pros::Task task([] {
        pros::Motor toggle_left(ports::kSideToggleLeft);
        pros::Motor toggle_right(ports::kSideToggleRight);
        toggle_left.move_relative(-360.0, 200);
        pros::delay(150);
        toggle_right.move_relative(-360.0, 200);
        pros::delay(300);
        side_toggle_motors.brake();
    });
}

}

void setup() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
}

// default, made in PointWeaver.
// Field frame: (0, 0) is the corner on blue's right, +X toward red, heading 0 faces red, clockwise. Inches.

// red-snacky/main
void red_snacky_main() {
    pros::delay(450);
    telemetry::mark(1001);
    chassis.moveToPoint(120, 96, 900, {.forwards = false}); // red goal 2, note: Red goal 2, backing straight up x = 120. Slowed for the last ~10 in at 40/127 (RedSnacky: 11 in full, then 45). Crimson starts the chain bar to a hover 25 deg short of the preload drop during this back-in, then score_preload() after the reset. Donut 1250 ms (run 37: it stalled at 171 deg from 1.25 s and gained 0.7 deg in its last 0.25 s; was 1500). Donut to 180 faces pin (96, 96).
    telemetry::mark(1002);
    macros::aim(preload_hover());
    chassis.waitUntil(17.11);
    telemetry::mark(1003);
    chassis.cancelMotion();
    telemetry::mark(1004);
    chassis.moveToPoint(120, 96, 400, {.forwards = false, .maxSpeed = 40, .minSpeed = 1, .earlyExitRange = 8}); // last 40%
    telemetry::mark(1005);
    chassis.waitUntilDone();
    telemetry::mark(1006);
    chassis.setPoseFromGoal(120, 96, {.distance = 7.17});
    telemetry::mark(1007);
    score_preload();
    chassis.donut(180, 1250, {.forwards = false, .earlyExitRange = 6});
    telemetry::mark(1008);
    wait_claw_low();
    chassis.moveToPoint(102.3, 96, 950, {.minSpeed = 1, .earlyExitRange = 0.5}); // claw on yellow pin (96, 96), note: Claw on yellow pin (96, 96). Snapped (robot at 102.3); RedSnacky stopped 0.75 in short, at 103.05, if the grab suffers. Exit 0.5 in: done within half an inch instead of LemLib's settle. Mechanism: wait_homed() before this drive; claw::grab() when within 1 in (clamp in the last inch); then aim(stage(2)) while backing onto the goal.
    telemetry::mark(1009);
    grab_near(102.3, 96);
    chassis.moveToPoint(120, 96, 750, {.forwards = false}); // red goal 2, note: Red goal 2 again (RedSnacky: 5 in full, then the last 5 in at 45). Mechanism: wait until the chain bar arrives (<= 800 ms) before the donut so it doesn't lever the goal off; score during the donut: wait arrived, drop(), home_async(). Donut inner wheel 20, not 35: at 35 it walks the aligner off the stake (RedSnacky). Faces pin (120, 120).
    telemetry::mark(1010);
    macros::aim(macros::stage(2));
    chassis.waitUntil(7.08);
    telemetry::mark(1011);
    chassis.cancelMotion();
    telemetry::mark(1012);
    chassis.moveToPoint(120, 96, 350, {.forwards = false, .maxSpeed = 40, .minSpeed = 1, .earlyExitRange = 8}); // last 60%
    telemetry::mark(1013);
    chassis.waitUntilDone();
    telemetry::mark(1014);
    chassis.setPoseFromGoal(120, 96, {.distance = 7.17});
    telemetry::mark(1015);
    wait_arrived(800);
    chassis.donut(-90, 1300, {.forwards = false, .slowSpeed = 20, .earlyExitRange = 6});
    telemetry::mark(1016);
    score();
    wait_claw_low();
    chassis.moveToPoint(120, 113.7, 950, {.minSpeed = 1, .earlyExitRange = 0.5}); // claw on yellow pin (120, 120), note: Claw on yellow pin (120, 120). Snapped (robot at 113.7); RedSnacky stopped 0.75 in short, at 112.95, if the grab suffers. Mechanism: wait_homed() before the drive; claw::grab() within the last inch; the 100 ms lets the pneumatics bite before the arm whips back; then aim(stage(3)) while backing.
    telemetry::mark(1017);
    grab_near(120, 113.7);
    chassis.waitUntilDone();
    telemetry::mark(1018);
    pros::delay(100);
    telemetry::mark(1019);
    chassis.moveToPoint(120, 96, 650, {.forwards = false}); // red goal 2, note: Red goal 2, third pin, last part at 40/127. Mechanism: score during the donut, then wait until the claw is low before driving at the wall. Donut 1200 ms (run 37: it crept 0.6 deg in its last 50 ms; was 1250). Donut to 0 faces the wall pin.
    telemetry::mark(1020);
    macros::aim(macros::stage(3));
    chassis.waitUntil(7.08);
    telemetry::mark(1021);
    chassis.cancelMotion();
    telemetry::mark(1022);
    chassis.moveToPoint(120, 96, 350, {.forwards = false, .maxSpeed = 40, .minSpeed = 1, .earlyExitRange = 8}); // last 60%
    telemetry::mark(1023);
    chassis.waitUntilDone();
    telemetry::mark(1024);
    chassis.setPoseFromGoal(120, 96, {.distance = 7.17});
    telemetry::mark(1025);
    chassis.donut(0, 1200, {.forwards = false, .earlyExitRange = 6});
    telemetry::mark(1026);
    score();
    macros::aim_lift(kWallPin);
    wait_claw_low();
    chassis.moveToPoint(135.5, 96, 850, {.minSpeed = 1, .earlyExitRange = 0.5}); // claw on yellow pin (141.8, 96), note: Claw on wall pin 5. Turns toward goal 1a right away: with its nose ~1 in from the wall, the turn runs a corner into the wall and the wall cups (on purpose: it levers the pin out of its holder). Mechanism: aim_lift({6 cm, 30 deg}) before the drive; claw grab on arrival; aim(stage(2, neutral)) during the turn and the drive to goal 1a.
    telemetry::mark(1027);
    chassis.waitUntilDone();
    telemetry::mark(1028);
    claw::grab();
    pros::delay(150);
    telemetry::mark(1029);
    chassis.turnToPoint(120, 48, 850, {.forwards = false, .minSpeed = 1, .earlyExitRange = 3}); // facing point 7
    telemetry::mark(1030);
    macros::aim(macros::stage(2, kNeutral));
    chassis.moveToPoint(120, 48, 1150, {.forwards = false, .earlyExitRange = 11}); // goal 1, note: Neutral goal 1a. Run 37 hit it at ~60 in/s (slowdown 0.25 left only ~5 in at 40/127) and tipped back 32 deg. Now slowed for the last ~22 in (0.45: ~0.24 s and ~9 in to brake from 60). Exit 11 (not 8): goal 1a is touched ~2.5 in early (odometry d 9.7), so the slow part ends before the touch and the reset's 40/127 push seats it. Score during the donut. Donut 72 deg to face wall pin 4, 1250 ms (the drive waits for the claw anyway).
    telemetry::mark(1031);
    chassis.waitUntil(27.74);
    telemetry::mark(1032);
    chassis.cancelMotion();
    telemetry::mark(1033);
    chassis.moveToPoint(120, 48, 1050, {.forwards = false, .maxSpeed = 40, .minSpeed = 1, .earlyExitRange = 11}); // last 45%
    telemetry::mark(1034);
    chassis.waitUntilDone();
    telemetry::mark(1035);
    chassis.setPoseFromGoal(120, 48, {.distance = 7.17});
    telemetry::mark(1036);
    chassis.donut(0, 1250, {.forwards = false, .earlyExitRange = 6});
    telemetry::mark(1037);
    score();
    macros::aim_lift(kWallPin);
    wait_claw_low();
    chassis.moveToPoint(135.5, 48, 850, {.minSpeed = 1, .earlyExitRange = 0.5}); // claw on yellow pin (141.8, 48), note: Claw on wall pin 4 (141.8, 48). Mechanism: wait for the home before the drive (the claw can't come down on the wall); claw grab on arrival; aim(stage(3, neutral)) while backing.
    telemetry::mark(1038);
    grab_on_arrival();
    chassis.moveToPoint(120, 48, 700, {.forwards = false}); // goal 1, note: Neutral goal 1a again, 15 in back from the wall pin: most of it slowed (the slow part has to start 2 in before its exit line). Mechanism: score (wait arrived, drop). The 250 ms is the scoring dwell; the auton ends here.
    telemetry::mark(1039);
    macros::aim(macros::stage(3, kNeutral));
    chassis.waitUntil(5.43);
    telemetry::mark(1040);
    chassis.cancelMotion();
    telemetry::mark(1041);
    chassis.moveToPoint(120, 48, 300, {.forwards = false, .maxSpeed = 40, .minSpeed = 1, .earlyExitRange = 8}); // last 65%
    telemetry::mark(1042);
    chassis.waitUntilDone();
    telemetry::mark(1043);
    chassis.setPoseFromGoal(120, 48, {.distance = 7.17});
    telemetry::mark(1044);
    release_when_arrived();
    pros::delay(250);
    telemetry::mark(1045);
}

void run() {
    telemetry::route("default #6484ad95");
    chassis.setPose(135.4, 72, 90);
    claw::grab();
    flick_side_toggles();
    red_snacky_main();
}

}
