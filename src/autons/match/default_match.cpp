#include "autons/routine.hpp"
#include "config/ports.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "subsystems/elevator/arm_pivot.hpp"
#include <cmath>
#include <cstdint>

// The 15 s match auton "default": the user's own drawn version of RedSnacky (origin/new-robot
// src/autons/match/red_snacky.cpp), tuned in PointWeaver. It scores the preload and pins (96, 96) and (120, 120) onto
// red goal 2, then wall pins 5 and 4 onto neutral goal 1a.
//
// red_snacky_main() and run() are PointWeaver's generated code, pasted as it wrote it: leave every chassis.* line,
// pros::delay and telemetry::route / mark line exactly as it is (the Log tab lines the SD-card logs up against them,
// and PointWeaver owns the drive numbers). The elevator calls between them are hand-placed from the notes PointWeaver
// keeps on each point, following RedSnacky: right after each pickup the lift and chain bar head for the scoring pose
// while the robot backs onto the goal, the score runs during the donut, and the drive to the next pin waits for the
// home to finish.

namespace autons::match::default_match {

namespace {

namespace arm_pivot = subsystems::elevator::arm_pivot;

// RedSnacky's numbers.
constexpr double kArriveLiftCm = 1.5;
constexpr double kArriveArmDeg = 4.0;
constexpr std::uint32_t kDropMs = 250;        // the pin falls onto the goal before the chain bar moves
constexpr float kGrabWithinIn = 1.0f;         // clamp during the last inch of a pin drive
constexpr int kNeutral = 1;                   // goal tier: neutral goal 1a is alliance + 7 cm of lift
constexpr macros::Pose kWallPin{6.0, 30.0};   // lift 6 cm so the claw bites a wall pin instead of catching it loose

// The lift and chain bar are within kArrive* of their targets. Looser than macros::settled(), which also waits for both
// controllers to brake.
bool lift_arrived() {
    return std::fabs(macros::pose_now().lift_cm - macros::target_now().lift_cm) < kArriveLiftCm;
}

bool arrived() {
    return lift_arrived() && std::fabs(macros::pose_now().arm_deg - macros::target_now().arm_deg) < kArriveArmDeg;
}

template <typename Done> void wait_until(Done done, std::uint32_t timeout_ms) {
    const std::uint32_t start = pros::millis();
    while (!done() && pros::millis() - start < timeout_ms) {
        pros::delay(10);
    }
}

// Block until the lift and chain bar have arrived, at most timeout_ms.
void wait_arrived(std::uint32_t timeout_ms) {
    wait_until(arrived, timeout_ms);
}

// The arm is on its hardstop (the home's push has finished; it rests there, not on its 30 deg target) and the lift is
// back down. Before every drive at a pin: the claw can't come down on it.
void wait_homed() {
    wait_until([] { return !arm_pivot::homing() && lift_arrived(); }, 2000);
}

// During a donut: drop as soon as the lift and chain bar reach the scoring pose, then start homing straight away.
// Not macros::drop(): its wait for the robot to stop rocking is for a robot standing still at a high stage. These run
// during donuts, at L3 and below (the lift 27 cm at most), where the swing barely rocks it.
void score() {
    wait_until(arrived, 1500);
    claw::release();
    pros::delay(kDropMs);
    macros::home_async();
}

// The preload onto red goal 2, on the goal before the first donut: L1 with the lift 1.75 cm lower (RedSnacky's first
// pin), wait until it gets there (at most 800 ms), drop, home.
void score_preload() {
    macros::Pose preload = macros::stage(1);
    preload.lift_cm -= 1.75;
    macros::aim(preload);
    wait_until(arrived, 800);
    claw::release();
    pros::delay(kDropMs);
    macros::home_async();
}

// Block until the robot is within kGrabWithinIn of (x, y) (or the drive has ended), then clamp: the claw closes during
// the last inch of the drive.
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

// Clamp once the drive into a pin has ended (the next motion would wait for it anyway).
void grab_on_arrival() {
    chassis.waitUntilDone();
    claw::grab();
}

// The last score: release once the lift and chain bar have arrived. The generated 250 ms after it is the drop's dwell.
void release_when_arrived() {
    wait_until(arrived, 1500);
    claw::release();
}

// Side toggles one turn each (~0.3 s at 200 rpm), the right one 150 ms behind the left, then brake: 450 ms in all, as
// RedSnacky ran it. In its own task, so it runs during the generated pros::delay(450) instead of before it.
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

// The same brake mode as skills, which every log PointWeaver tuned the drive against was run with.
void setup() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
}

// default, made in PointWeaver.
// Field frame: (0, 0) is the corner on blue's right, +X toward red, heading 0 faces red, clockwise. Inches.

// red-snacky/main
void red_snacky_main() {
    pros::delay(450);
    telemetry::mark(1001);
    chassis.moveToPoint(120, 96, 900, {.forwards = false}); // red goal 2, note: Red goal 2, backing straight up x = 120. Slowed for the last ~10 in at 40/127 (RedSnacky: 11 in full, then 45). Mechanism after the reset, before the donut (PointWeaver can't put it there yet, ~0.6-1.0 s): aim(preload), wait until arrived (<= 800 ms), claw::release(), 250 ms, home_async(). Donut to 180 faces pin (96, 96); 1250 ms for ~90 deg (was 2100).
    telemetry::mark(1002);
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
    score_preload(); // on the goal: L1, drop, home (the home runs during the donut)
    chassis.donut(180, 1500, {.forwards = false, .earlyExitRange = 6});
    telemetry::mark(1008);
    wait_homed(); // homed before the claw reaches the pin
    chassis.moveToPoint(102.3, 96, 950, {.minSpeed = 1, .earlyExitRange = 0.5}); // claw on yellow pin (96, 96), note: Claw on yellow pin (96, 96). Snapped (robot at 102.3); RedSnacky stopped 0.75 in short, at 103.05, if the grab suffers. Exit 0.5 in: done within half an inch instead of LemLib's settle. Mechanism: wait_homed() before this drive; claw::grab() when within 1 in (clamp in the last inch); then aim(stage(2)) while backing onto the goal.
    telemetry::mark(1009);
    grab_near(102.3, 96); // clamp during the last inch
    chassis.moveToPoint(120, 96, 750, {.forwards = false}); // red goal 2, note: Red goal 2 again (RedSnacky: 5 in full, then the last 5 in at 45). Mechanism: wait until the chain bar arrives (<= 800 ms) before the donut so it doesn't lever the goal off; score during the donut: wait arrived, drop(), home_async(). Donut inner wheel 20, not 35: at 35 it walks the aligner off the stake (RedSnacky). Faces pin (120, 120).
    telemetry::mark(1010);
    macros::aim(macros::stage(2)); // lift and chain bar together while backing onto the goal
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
    wait_arrived(800); // chain bar done moving before the donut, so it doesn't lever the goal off
    chassis.donut(-90, 1300, {.forwards = false, .slowSpeed = 20, .earlyExitRange = 6});
    telemetry::mark(1016);
    score(); // during the donut
    wait_homed(); // homed before the claw reaches the pin
    chassis.moveToPoint(120, 113.7, 950, {.minSpeed = 1, .earlyExitRange = 0.5}); // claw on yellow pin (120, 120), note: Claw on yellow pin (120, 120). Snapped (robot at 113.7); RedSnacky stopped 0.75 in short, at 112.95, if the grab suffers. Mechanism: wait_homed() before the drive; claw::grab() within the last inch; the 100 ms lets the pneumatics bite before the arm whips back; then aim(stage(3)) while backing.
    telemetry::mark(1017);
    grab_near(120, 113.7); // clamp during the last inch
    chassis.waitUntilDone();
    telemetry::mark(1018);
    pros::delay(100);
    telemetry::mark(1019);
    chassis.moveToPoint(120, 96, 650, {.forwards = false}); // red goal 2, note: Red goal 2, third pin (RedSnacky backed in at full speed here; now the last part at 40/127 like the others). Mechanism: score during the donut (wait arrived, drop, home_async), then wait_homed() before driving at the wall: the claw can't come down on it. Donut to 0 faces the wall pin.
    telemetry::mark(1020);
    macros::aim(macros::stage(3)); // lift and chain bar together while backing onto the goal
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
    chassis.donut(0, 1250, {.forwards = false, .earlyExitRange = 6});
    telemetry::mark(1026);
    score(); // during the donut
    wait_homed(); // fully home before driving at the wall, or the claw comes down on it
    macros::aim_lift(kWallPin); // the lift to 6 cm, so the claw bites the wall pin
    chassis.moveToPoint(135.5, 96, 850, {.minSpeed = 1, .earlyExitRange = 0.5}); // claw on yellow pin (141.8, 96), note: Claw on wall pin 5. Turns toward goal 1a right away: with its nose ~1 in from the wall, the turn runs a corner into the wall and the wall cups (on purpose: it levers the pin out of its holder). Mechanism: aim_lift({6 cm, 30 deg}) before the drive; claw grab on arrival; aim(stage(2, neutral)) during the turn and the drive to goal 1a.
    telemetry::mark(1027);
    chassis.waitUntilDone();
    telemetry::mark(1028);
    claw::grab(); // on arrival; the 150 ms below lets it bite before the turn levers the pin out
    pros::delay(150);
    telemetry::mark(1029);
    chassis.turnToPoint(120, 48, 850, {.forwards = false, .minSpeed = 1, .earlyExitRange = 3}); // facing point 7
    telemetry::mark(1030);
    macros::aim(macros::stage(2, kNeutral)); // L2 on the neutral goal, during the turn and the drive
    chassis.moveToPoint(120, 48, 1400, {.forwards = false}); // goal 1, note: Neutral goal 1a (already has its pin). Slowed for the last ~8 in at 40/127. Score during the donut (wait arrived, drop, home_async): no wait. Donut 72 deg to face wall pin 4 (1100 ms; ~0.4 s + 72/136).
    telemetry::mark(1031);
    chassis.waitUntil(37.83);
    telemetry::mark(1032);
    chassis.cancelMotion();
    telemetry::mark(1033);
    chassis.moveToPoint(120, 48, 500, {.forwards = false, .maxSpeed = 40, .minSpeed = 1, .earlyExitRange = 8}); // last 25%
    telemetry::mark(1034);
    chassis.waitUntilDone();
    telemetry::mark(1035);
    chassis.setPoseFromGoal(120, 48, {.distance = 7.17});
    telemetry::mark(1036);
    chassis.donut(0, 1100, {.forwards = false, .earlyExitRange = 6});
    telemetry::mark(1037);
    score(); // during the donut
    wait_homed(); // fully home before driving at the wall
    macros::aim_lift(kWallPin); // as for wall pin 5
    chassis.moveToPoint(135.5, 48, 850, {.minSpeed = 1, .earlyExitRange = 0.5}); // claw on yellow pin (141.8, 48), note: Claw on wall pin 4 (141.8, 48). Mechanism: wait for the home before the drive (the claw can't come down on the wall); claw grab on arrival; aim(stage(3, neutral)) while backing.
    telemetry::mark(1038);
    grab_on_arrival();
    chassis.moveToPoint(120, 48, 700, {.forwards = false}); // goal 1, note: Neutral goal 1a again, 15 in back from the wall pin: most of it slowed (the slow part has to start 2 in before its exit line). Mechanism: score (wait arrived, drop). The 250 ms is the scoring dwell; the auton ends here.
    telemetry::mark(1039);
    macros::aim(macros::stage(3, kNeutral)); // L3 on the neutral goal, while backing
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
    release_when_arrived(); // the last score; the 250 ms below is its dwell
    pros::delay(250);
    telemetry::mark(1045);
}

void run() {
    telemetry::route("default #7de6f1a8");
    chassis.setPose(135.4, 72, 90);
    claw::grab();         // hold the preload; the claw boots open
    flick_side_toggles(); // runs during the generated 450 ms at the start of red_snacky_main
    red_snacky_main();
}

}
