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
    // Only now, clear of the wall: home (the arm pushes onto its hardstop and resets its encoder there), then the
    // preload's lift and chain bar go to L1 (the chain bar once the home is done) during the turn and the back-in.
    macros::home_async();
    macros::aim_staged(stage(1), [] { return !subsystems::elevator::arm_pivot::homing(); });
    chassis.turnToHeading(90, 700);
    chassis.waitUntilDone();
    // Aim at where the robot's back meets the goal (centre 7.5 in from the goal's), so the drive slows down by itself
    // on the way in and ends at the goal instead of pushing into it until it times out.
    chassis.moveToPoint(120, 88.5, 1200, {.forwards = false, .maxSpeed = 70}); // red goal 2; 70 so it doesn't hit too hard
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});

    // skills right_start: donut(180, 1750) + moveToPoint(102.3, 96, 750)
    chassis.donut(180, 1750, {.forwards = false, .slowSpeed = 16}); // facing pin (96, 96)
    score(); // the preload, during the donut
    wait_homed(); // homed before the claw reaches the pin
    chassis.moveToPoint(103.05, 96, 750, {.earlyExitRange = 0.5}); // claw on yellow pin (96, 96), 0.75 in short of skills' 102.3
    wait_near(103.05, 96, 1);
    claw::grab(); // clamp during the last inch
    chassis.waitUntilDone();
    macros::aim(stage(2)); // lift and chain bar together while backing onto the goal
    chassis.moveToPoint(112.5, 96, 1200, {.forwards = false}); // red goal 2, centre 7.5 in out: just touching
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});

    // skills transition's last donut(-90, 1750) + right_start's moveToPoint(120, 113.7, 750)
    chassis.donut(-90, 1750, {.forwards = false, .slowSpeed = 16}); // facing pin (120, 120)
    score(); // during the donut
    wait_homed(); // homed before the claw reaches the pin
    chassis.moveToPoint(120, 112.95, 750, {.earlyExitRange = 0.5}); // claw on yellow pin (120, 120), 0.75 in short of skills' 113.7
    wait_near(120, 112.95, 1);
    claw::grab(); // clamp during the last inch
    chassis.waitUntilDone();
    pros::delay(100); // allow pneumatics to fully bite before whipping arm backward
    macros::aim(stage(3)); // lift and chain bar together while backing onto the goal
    chassis.moveToPoint(120, 103.5, 1200, {.forwards = false}); // red goal 2, centre 7.5 in out: just touching
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});

    // skills transition: donut(0, 950) + moveToPoint(135.5, 96, 700)
    chassis.donut(0, 1750, {.forwards = false, .slowSpeed = 16}); // facing the wall pin; a 90 deg turn like the others
    score(); // during the donut
    chassis.waitUntilDone();
    wait_homed(); // fully home before driving at the wall, or the claw comes down on it
    macros::aim_lift({4.0, 30.0}); // lift chain bar 2 cm higher to pick the wall pin (L2 pin)
    chassis.moveToPoint(136.7, 96, 900, {.earlyExitRange = 0.5}); // claw on wall pin (141.8, 96)
    chassis.waitUntilDone();
    macros::pickup();
    macros::aim(stage(2, Goal::Neutral)); // L2 on the neutral goal; lift and chain bar move during the turn and drive
    chassis.turnToPoint(120, 48, 500, {.forwards = false});
    chassis.waitUntilDone();
    // neutral goal (120, 48), centre 7.5 in out on the line from the wall pin (136.7, 96): just touching
    chassis.moveToPoint(122.5, 55.1, 1600, {.forwards = false});
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 48, {.distance = 7.5});
    score();
}

}
