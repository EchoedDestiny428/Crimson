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

// Robot center to goal center when the robot's back is against the goal: where every back-in aims, and the distance
// for setPoseFromGoal and the donut re-anchor.
constexpr float kGoalTouchIn = 7.5f;
// Every back-in aims this far out, 1 in past touching, so it keeps pushing until the goal stops it (wait_stopped).
// The pose reset and donut re-anchor use kGoalTouchIn.
constexpr float kGoalStopIn = 6.5f;

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

// Low enough to go for the next pin: the chain bar is at or below kHomedEnoughDeg and the lift is back down. The home
// keeps going in the background (the push onto the hardstop and the encoder reset there) while the robot drives.
constexpr double kHomedEnoughDeg = 35.0;

void wait_homed() {
    wait_until([] {
        const bool arm_down =
            !subsystems::elevator::arm_pivot::homing() || macros::pose_now().arm_deg <= kHomedEnoughDeg;
        return arm_down && lift_arrived();
    }, 2000);
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

// End a goal back-in as soon as the robot stops against the goal, instead of waiting out the drive's timeout: it
// rests past the drive's target there, so the drive never counts as done. Stopped: the tracking wheels moved under
// 0.01 in per 10 ms (1 in/s) for 60 ms, ignoring the first 100 ms. timeout_ms is the backstop.
void wait_stopped(std::uint32_t timeout_ms) {
    const std::uint32_t start = pros::millis();
    lemlib::Pose last = chassis.getPose();
    std::uint32_t still_since = 0;
    while (pros::millis() - start < timeout_ms) {
        pros::delay(10);
        const lemlib::Pose pose = chassis.getPose();
        const bool still = std::hypot(pose.x - last.x, pose.y - last.y) < 0.01f;
        last = pose;
        if (!still || pros::millis() - start < 100) {
            still_since = 0;
            continue;
        }
        if (still_since == 0) {
            still_since = pros::millis();
        }
        if (pros::millis() - still_since >= 60) {
            break;
        }
    }
    chassis.cancelMotion();
}

void setup() {}

void run() {
    chassis.setPose(135.5, 72, 180); // the old start, turned to face blue
    claw::grab(); // hold the preload; the claw boots open
    // Chain bar a little up off the hardstop (25 -> 40 deg, the claw ~4 cm higher, in front) so the preload doesn't
    // drag on the tiles during the toggles and the drive out; the home after the drive brings it back down.
    macros::aim({0.0, 40.0});

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
    // above L1 during the turn and the back-in. The pin stays clear of the goal. The last of the L1 swing starts on
    // the slow last 6 in.
    macros::home_async();
    // L1's lift is 3 cm. This first pin releases 1.75 cm lower; later scores stay on the normal stages.
    macros::Pose preload = stage(1);
    preload.lift_cm -= 1.75;
    macros::Pose hover = preload;
    hover.arm_deg -= 25.0;
    macros::aim_staged(hover, [] { return !subsystems::elevator::arm_pivot::homing(); });
    chassis.turnToHeading(90, 700);
    chassis.waitUntilDone();
    // Back-ins (Roy's): full speed, then the last 6 in before contact at 50 so it seats without bouncing. Every one
    // aims 1 in past touching the goal (centre kGoalStopIn from the goal's) and ends once the goal stops it.
    chassis.moveToPoint(120, 89.5, 800, {.forwards = false}); // red goal 2, 16.5 in to contact
    chassis.waitUntil(10.5); // 6 in before contact
    chassis.cancelMotion();
    chassis.moveToPoint(120, 89.5, 600, {.forwards = false, .maxSpeed = 50});
    macros::aim(preload); // the rest of the L1 motion (the last 25 deg), during the slow last 6 in; it finishes in the donut
    wait_stopped(600);
    chassis.setPoseFromGoal(120, 96, {.distance = kGoalTouchIn});

    // skills right_start: donut(180, 1750) + moveToPoint(102.3, 96, 750)
    chassis.donut(180, 1650, {.forwards = false, .slowSpeed = 35, .reanchor = true, .goalDistance = kGoalTouchIn}); // facing pin (96, 96)
    score(); // the preload, during the donut
    wait_homed(); // homed before the claw reaches the pin
    chassis.moveToPoint(103.05, 96, 750, {.earlyExitRange = 0.5}); // claw on yellow pin (96, 96), 0.75 in short of skills' 102.3
    wait_near(103.05, 96, 1);
    claw::grab(); // clamp during the last inch
    chassis.waitUntilDone();
    // Hover: the lift goes to L2 but the chain bar stops at 270 on the way in, so it isn't still swinging back when the
    // robot hits the goal (that rode it up onto the goal). The last ~40 deg to L2 (312) is after contact; at 180 that
    // was ~130 deg and held up the drop.
    macros::Pose hover_l2 = stage(2);
    hover_l2.arm_deg = 270.0;
    macros::aim(hover_l2);
    // Stops at contact (centre kGoalTouchIn from the goal's) instead of pushing 1 in past it into the goal.
    chassis.moveToPoint(112.5, 96, 900, {.forwards = false}); // red goal 2, 9.45 in to contact
    chassis.waitUntil(3.45); // 6 in before contact
    chassis.cancelMotion();
    chassis.moveToPoint(112.5, 96, 600, {.forwards = false, .maxSpeed = 50});
    wait_stopped(600); // still ends on a stall too, in case odometry is off and it reaches the goal early
    chassis.setPoseFromGoal(120, 96, {.distance = kGoalTouchIn});
    macros::aim(stage(2)); // finish the swing to L2 now that the robot is on the goal; it finishes in the donut

    // Roy: at 35 the inner wheel drives forward hard enough to walk the aligner off the stake.
    chassis.donut(-90, 1650, {.forwards = false, .slowSpeed = 35, .reanchor = true, .goalDistance = kGoalTouchIn}); // facing pin (120, 120)
    score(); // during the donut
    wait_homed(); // homed before the claw reaches the pin
    chassis.moveToPoint(120, 112.95, 750, {.earlyExitRange = 0.5}); // claw on yellow pin (120, 120), 0.75 in short of skills' 113.7
    wait_near(120, 112.95, 1);
    claw::grab(); // clamp during the last inch
    chassis.waitUntilDone();
    pros::delay(100); // allow pneumatics to fully bite before whipping arm backward
    macros::aim(stage(3)); // lift and chain bar together while backing onto the goal
    chassis.moveToPoint(120, 102.5, 900, {.forwards = false}); // red goal 2, 9.45 in to contact
    chassis.waitUntil(3.45); // 6 in before contact
    chassis.cancelMotion();
    chassis.moveToPoint(120, 102.5, 600, {.forwards = false, .maxSpeed = 50});
    wait_stopped(600);
    chassis.setPoseFromGoal(120, 96, {.distance = kGoalTouchIn});

    // skills transition: donut(0, 950) + moveToPoint(135.5, 96, 700)
    chassis.donut(0, 1650, {.forwards = false, .slowSpeed = 35, .reanchor = true, .goalDistance = kGoalTouchIn}); // facing the wall pin; a 90 deg turn like the others
    score(); // during the donut
    chassis.waitUntilDone();
    wait_homed(); // down to 35 deg before driving at the wall; the home finishes on the way
    macros::aim_lift({6.0, 30.0}); // higher than the old 4 cm so the claw bites the wall pin instead of catching it loose
    // claw on wall pin (141.8, 96): at 30 deg the claw is 7.09 in ahead of centre, so this puts it 0.5 in past the pin
    chassis.moveToPoint(135.21, 96, 900, {.earlyExitRange = 0.5});
    wait_stopped(900); // ends once stopped, on the target or against the pin and wall
    macros::pickup();
    // Two steps instead of a 250 ms wait for the bite: straight after the grip, the lift goes to L2-neutral and the
    // chain bar only a little way up (90 deg), a gentle move that doesn't fling the pin. The full swing to L2 starts
    // once the turn is done, during the drive to the neutral goal.
    macros::Pose lift_off = stage(2, Goal::Neutral);
    lift_off.arm_deg = 90.0;
    macros::aim(lift_off);
    chassis.turnToPoint(120, 48, 500, {.forwards = false});
    chassis.waitUntilDone();
    macros::aim(stage(2, Goal::Neutral)); // L2 on the neutral goal, during the drive
    // neutral goal (120, 48), centre kGoalTouchIn out on the line from the wall pin: seated in the aligner
    chassis.moveToPoint(122.44, 55.09, 1300, {.forwards = false}); // 43.3 in to contact; stops at contact, not 1 in in
    chassis.waitUntil(34.3); // 9 in before contact
    chassis.cancelMotion();
    chassis.moveToPoint(122.44, 55.09, 800, {.forwards = false, .maxSpeed = 50}); // last 9 in at 50
    wait_stopped(900); // 9 in at 50 takes longer than 600
    chassis.setPoseFromGoal(120, 48, {.distance = kGoalTouchIn});

    // Donut to 0 while scoring the wall pin, facing the next wall pin (141.8, 48).
    chassis.donut(0, 1650, {.forwards = false, .slowSpeed = 35, .reanchor = true, .goalDistance = kGoalTouchIn}); // facing wall pin (141.8, 48)
    score(); // during the donut
    chassis.waitUntilDone();

    // Final wall pin (141.8, 48), the same pickup as the first wall pin, then straight back onto the neutral goal as
    // its L3. The robot ends the donut on the goal at (127.5, 48), facing the pin, 7.7 in from the pickup point.
    wait_homed(); // down to 35 deg before driving at the wall; the home finishes on the way
    macros::aim_lift({6.0, 30.0});
    chassis.moveToPoint(135.21, 48, 900, {.earlyExitRange = 0.5}); // claw 0.5 in past the pin at 30 deg
    wait_stopped(900); // ends once stopped, on the target or against the pin and wall
    macros::pickup();
    // Gentle lift-off as at the first wall pin; the swing to L3 waits for contact, too short a drive to finish it.
    macros::Pose lift_off_l3 = stage(3, Goal::Neutral);
    lift_off_l3.arm_deg = 90.0;
    macros::aim(lift_off_l3);
    chassis.moveToPoint(127.5, 48, 800, {.forwards = false, .maxSpeed = 50}); // neutral goal, 7.7 in to contact
    wait_stopped(900);
    chassis.setPoseFromGoal(120, 48, {.distance = kGoalTouchIn});
    macros::aim(stage(3, Goal::Neutral)); // L3 on the neutral goal
    score();
}

}
