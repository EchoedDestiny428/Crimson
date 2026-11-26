#include "autons/routine.hpp"
#include "config/ports.hpp"
#include "pros/rtos.hpp"
#include "subsystems/side_toggle.hpp"
#include "subsystems/elevator/claw.hpp"
#include "subsystems/elevator/arm_pivot.hpp"
#include "subsystems/elevator/macros.hpp"
#include <cmath>

namespace autons::match::red_snacky {

// Red snacky: starts on the red wall facing blue, drives out to x = 120 (one tile in), turns to 90 and backs straight
// up x = 120 into red goal 2 (120, 96), so the first alignment with the goal is square, not from the corner. Then the
// three pins: (96, 96) and (120, 120) onto red goal 2, and the wall pin (141.8, 96) onto the neutral goal (120, 48).
// Mechanism: everything runs alongside the driving. Right after each pickup the lift and chain bar both head for the
// scoring pose while the robot backs onto the goal; once there, the donut starts straight away and the score runs
// during it (the donut orbits the goal behind the robot, so the claw stays over it): wait for the lift and chain bar,
// drop, and start homing at once; the drive to each pin waits until the home has finished. On the two free-standing pins the claw clamps
// during the last inch of the drive. The preload goes up to L1 only once the robot is out in the x = 120 lane, clear
// of the wall. Before the wall pin the home has to finish (the claw can't come down on the wall): it runs
// during the donut, and the drive to the wall waits for it.

namespace claw = subsystems::elevator::claw;
namespace macros = subsystems::elevator::macros;
using subsystems::elevator::constants::Goal;

// Driver stage setpoint (1-7), raised kGoalTierStepCm per goal tier like the Right button. Same as skills.
macros::Pose stage(int level, Goal goal = Goal::Alliance) {
    const auto& setpoint = subsystems::elevator::constants::kStageSetpoints[level];
    return {setpoint.lift_cm + static_cast<int>(goal) * subsystems::elevator::constants::kGoalTierStepCm,
            setpoint.arm_deg};
}

constexpr double kArriveLiftCm = 1.5;
constexpr double kArriveArmDeg = 4.0;

// Robot center to goal center when the aligner is seated. The aligner is narrower than the old 7.5 in touch.
constexpr float kGoalTouchIn = 7.0f;

// The lift and chain bar are within kArrive* of their targets. macros::settled() also waits for both controllers to
// report settled, which hangs a few hundred ms after they're already there.
bool lift_arrived() {
    return std::fabs(macros::pose_now().lift_cm - macros::target_now().lift_cm) < kArriveLiftCm;
}

bool arrived() {
    return lift_arrived() && std::fabs(macros::pose_now().arm_deg - macros::target_now().arm_deg) < kArriveArmDeg;
}

// Block until `done` or timeout_ms.
template <typename Done> void wait_until(Done done, std::uint32_t timeout_ms) {
    const std::uint32_t start = pros::millis();
    while (!done() && pros::millis() - start < timeout_ms) {
        pros::delay(10);
    }
}

// The arm is on its hardstop (the home's push has finished; it rests there, not on its 30 deg target) and the lift is
// back down.
void wait_homed() {
    wait_until([] { return !subsystems::elevator::arm_pivot::homing() && lift_arrived(); }, 2000);
}

// At the goal after aim(): drop as soon as the lift and chain bar reach the scoring pose, then start homing straight
// away. The drive to the next pin waits for the home to finish (wait_homed).
void score() {
    wait_until(arrived, 1500);
    macros::drop();
    macros::home_async(); // home after every score
}

// Pin drives use earlyExitRange 0.5: done within half an inch, instead of sitting in the drive's settle window (within
// 1 in for 500 ms) before the pickup.

// Block until the robot is within `within` in of (x, y) or the current motions end; for clamping while still driving in.
void wait_near(float x, float y, float within) {
    while (chassis.isInMotion()) {
        const lemlib::Pose pose = chassis.getPose();
        if (std::hypot(pose.x - x, pose.y - y) <= within) {
            return;
        }
        pros::delay(10);
    }
}

void setup() {}

void run() {
    chassis.setPose(135.5, 72, 180); // the old start, turned to face blue
    claw::grab(); // hold the preload; the claw boots open

    // Side toggles one turn each (~0.3 s at 200 rpm), the right one 150 ms behind the left.
    pros::Motor toggle_left(ports::kSideToggleLeft);
    pros::Motor toggle_right(ports::kSideToggleRight);
    toggle_left.move_relative(-360.0, 200);
    pros::delay(150);
    toggle_right.move_relative(-360.0, 200);
    pros::delay(300);
    side_toggle_motors.brake();

    // Out to x = 120, in line with red goal 2, then square up and back straight into it.
    chassis.moveToPoint(120, 72, 800); // 15.5 in
    chassis.waitUntilDone();
    // Only now, clear of the wall: home, then the lift goes to the preload height and the chain bar swings to 25 deg
    // above L1 during the turn and the back-in. The pin stays clear of the goal. The last of the L1 swing happens
    // once the robot is stopped on it.
    macros::home_async();
    // L1's lift is 3 cm. This first pin releases 1.75 cm lower; later scores stay on the normal stages.
    macros::Pose preload = stage(1);
    preload.lift_cm -= 1.75;
    macros::Pose hover = preload;
    hover.arm_deg -= 25.0;
    macros::aim_staged(hover, [] { return !subsystems::elevator::arm_pivot::homing(); });
    chassis.turnToHeading(90, 700);
    chassis.waitUntilDone();
    // 17 in back to the goal. Full speed for the first 11 in, then the rest at 45 so it seats without bouncing.
    chassis.moveToPoint(120, 89.0, 800, {.forwards = false});
    chassis.waitUntil(11);
    chassis.cancelMotion();
    chassis.moveToPoint(120, 89.0, 500, {.forwards = false, .maxSpeed = 45});
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = kGoalTouchIn});
    macros::aim(preload); // the rest of the L1 motion, only now that the robot is on the goal
    wait_until(arrived, 800);
    claw::release();
    pros::delay(250); // pin has to fall onto the goal before the chain bar moves
    macros::home_async();

    // skills right_start: donut(180, 1750) + moveToPoint(102.3, 96, 750)
    chassis.donut(180, 2100, {.forwards = false, .slowSpeed = 35}); // facing pin (96, 96)
    wait_homed(); // homed before the claw reaches the pin
    chassis.moveToPoint(103.05, 96, 750, {.earlyExitRange = 0.5}); // claw on yellow pin (96, 96), 0.75 in short of skills' 102.3
    wait_near(103.05, 96, 1);
    claw::grab(); // clamp during the last inch
    chassis.waitUntilDone();
    macros::aim(stage(2)); // lift and chain bar together while backing onto the goal
    // 10 in back. Full speed, then the last 5 in at 45 so the aligner is seated before the donut.
    chassis.moveToPoint(113.0, 96, 900, {.forwards = false});
    chassis.waitUntil(5);
    chassis.cancelMotion();
    chassis.moveToPoint(113.0, 96, 500, {.forwards = false, .maxSpeed = 45});
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = kGoalTouchIn});
    wait_until(arrived, 800); // chain bar done moving before the pivot, so it doesn't lever the goal off

    // Inner wheel at 20, not 35: at 35 it drives forward hard enough to walk the aligner off the stake.
    chassis.donut(-90, 2100, {.forwards = false, .slowSpeed = 20}); // facing pin (120, 120)
    score(); // during the donut
    wait_homed(); // homed before the claw reaches the pin
    chassis.moveToPoint(120, 112.95, 750, {.earlyExitRange = 0.5}); // claw on yellow pin (120, 120), 0.75 in short of skills' 113.7
    wait_near(120, 112.95, 1);
    claw::grab(); // clamp during the last inch
    chassis.waitUntilDone();
    pros::delay(100); // allow pneumatics to fully bite before whipping arm backward
    macros::aim(stage(3)); // lift and chain bar together while backing onto the goal
    chassis.moveToPoint(120, 103.0, 1200, {.forwards = false}); // red goal 2, centre kGoalTouchIn out
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = kGoalTouchIn});

    // skills transition: donut(0, 950) + moveToPoint(135.5, 96, 700)
    chassis.donut(0, 2100, {.forwards = false, .slowSpeed = 35}); // facing the wall pin; a 90 deg turn like the others
    score(); // during the donut
    chassis.waitUntilDone();
    wait_homed(); // fully home before driving at the wall, or the claw comes down on it
    macros::aim_lift({6.0, 30.0}); // higher than the old 4 cm so the claw bites the wall pin instead of catching it loose
    chassis.moveToPoint(138.2, 96, 700, {.maxSpeed = 100}); // 1.5 in deeper than 136.7, into the wall pin (141.8, 96)
    chassis.waitUntilDone();
    macros::pickup();
    pros::delay(250); // let the claw finish biting before the arm swings, so the wall pin isn't flung
    macros::aim(stage(2, Goal::Neutral)); // L2 on the neutral goal; lift and chain bar move during the turn and drive
    chassis.turnToPoint(120, 48, 500, {.forwards = false});
    chassis.waitUntilDone();
    // neutral goal (120, 48), centre kGoalTouchIn out on the line from the wall pin: seated in the aligner
    chassis.moveToPoint(122.3, 54.6, 1600, {.forwards = false});
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 48, {.distance = kGoalTouchIn});
    score();
}

}
