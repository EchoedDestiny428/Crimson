#include "autons/routine.hpp"
#include "pros/rtos.hpp"
#include "subsystems/side_toggle.hpp"
#include "subsystems/elevator/claw.hpp"
#include "subsystems/elevator/macros.hpp"
#include <cmath>

namespace autons::match::red_snacky {

// Red snacky: blue snacky (snacky.cpp) turned 180 deg about the field centre, (x, y) -> (144 - x, 144 - y),
// heading + 180. Red goal 2 (120, 96) is where skills' transition / right_start already work, so the moves match
// those lines. Starts on the red wall, the same way round as blue on the blue wall.
// Mechanism: everything runs alongside the driving. Right after each pickup the lift and chain bar both head for the
// scoring pose while the robot backs onto the goal; once there, the donut starts straight away and the score runs
// during it (the donut orbits the goal behind the robot, so the claw stays over it): wait for the lift and chain bar,
// drop, give the pin 250 ms to land, and home on the way to the next pin. On the two free-standing pins the claw clamps during the last inch of the drive.
// The preload starts while the arm is still homing, so only its lift moves on the way and the chain bar swings at the
// goal. Before the wall pin the home has to finish (the claw can't come down on the wall): it runs during the donut,
// and the drive to the wall waits for it.

namespace claw = subsystems::elevator::claw;
namespace macros = subsystems::elevator::macros;
using subsystems::elevator::constants::Goal;

// Driver stage setpoint (1-7), raised kGoalTierStepCm per goal tier like the Right button. Same as skills.
macros::Pose stage(int level, Goal goal = Goal::Alliance) {
    const auto& setpoint = subsystems::elevator::constants::kStageSetpoints[level];
    return {setpoint.lift_cm + static_cast<int>(goal) * subsystems::elevator::constants::kGoalTierStepCm,
            setpoint.arm_deg};
}

constexpr std::uint32_t kScoreSettleMs = 250; // after the release, before anything moves: let the pin drop onto the goal

// At the goal after aim(): wait for the lift and chain bar to arrive, drop, then home without waiting.
void score() {
    macros::wait_settled();
    macros::drop();
    pros::delay(kScoreSettleMs);
    macros::home_async(); // home after every score
}

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
    chassis.setPose(135.5, 72, 90);
    claw::grab(); // hold the preload; the claw boots open

    side_toggle_motors.move_relative(-8 * 360.0, 200); // 3 turns (~0.9 s at 200 rpm)
    // Home while the toggle spins: the arm pushes onto its hardstop and resets its encoder there.
    macros::home_async();
    pros::delay(900);
    side_toggle_motors.brake();

    chassis.moveToPoint(120, 96, 600, {.forwards = false, .maxSpeed = 70}); // red goal 2, 28.6 in away; 70 so it doesn't hit too hard
    macros::aim_lift(stage(1)); // preload
    chassis.waitUntil(18.1); // 3 in before the robot's back meets the goal (7.5 in short of its centre)
    chassis.cancelMotion();
    chassis.moveToPoint(120, 96, 600, {.forwards = false, .maxSpeed = 50}); // last 3 in
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});

    // skills transition: donut(0, 950) + moveToPoint(135.5, 96, 700)
    // The donut orbits the goal behind the robot, so the claw stays over it: score and home while it turns.
    chassis.donut(0, 1200, {.forwards = false, .slowSpeed = 20}); // facing the wall pin
    macros::swing_arm(); // chain bar only once backed against the goal
    macros::drop();
    pros::delay(kScoreSettleMs);
    macros::home_async();
    chassis.waitUntilDone();
    macros::wait_settled(); // fully home before driving at the wall, or the claw comes down on it
    chassis.moveToPoint(136.5, 96, 700); // claw on wall pin (141.8, 96), 1 in deeper than skills' 135.5
    chassis.waitUntilDone();
    macros::pickup();
    macros::aim(stage(2)); // lift and chain bar together while backing onto the goal
    chassis.moveToPoint(120, 96, 950, {.forwards = false}); // red goal 2, 16.5 in away
    chassis.waitUntil(6); // 3 in before the robot's back meets the goal (7.5 in short of its centre)
    chassis.cancelMotion();
    chassis.moveToPoint(120, 96, 600, {.forwards = false, .maxSpeed = 50}); // last 3 in
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});

    // skills transition's last donut(-90, 1750) + right_start's moveToPoint(120, 113.7, 750)
    chassis.donut(-90, 1750, {.forwards = false, .slowSpeed = 20}); // facing pin (120, 120)
    score(); // during the donut
    chassis.moveToPoint(120, 113.95, 750); // claw on yellow pin (120, 120), 0.25 in further in than skills' 113.7
    wait_near(120, 113.95, 1);
    claw::grab(); // clamp during the last inch
    chassis.waitUntilDone();
    macros::aim(stage(3)); // lift and chain bar together while backing onto the goal
    chassis.moveToPoint(120, 96, 950, {.forwards = false}); // red goal 2, 17.95 in away
    chassis.waitUntil(7.45); // 3 in before the robot's back meets the goal (7.5 in short of its centre)
    chassis.cancelMotion();
    chassis.moveToPoint(120, 96, 600, {.forwards = false, .maxSpeed = 50}); // last 3 in
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});

    // skills right_start: donut(180, 1750) + moveToPoint(102.3, 96, 750)
    chassis.donut(180, 1750, {.forwards = false, .slowSpeed = 20}); // facing pin (96, 96)
    score(); // during the donut
    chassis.moveToPoint(102.05, 96, 750); // claw on yellow pin (96, 96), 0.25 in past skills' 102.3
    wait_near(102.05, 96, 1);
    claw::grab(); // clamp during the last inch
    chassis.waitUntilDone();
    macros::aim(stage(2, Goal::Neutral)); // L2 on the neutral goal; lift and chain bar move during the turn and drive
    chassis.turnToPoint(120, 48, 500, {.forwards = false});
    chassis.waitUntilDone(); // so waitUntil below measures the drive, not the turn
    chassis.moveToPoint(120, 48, 950, {.forwards = false}); // neutral goal (120, 48), 51.3 in away
    chassis.waitUntil(40.8); // 3 in before the robot's back meets the goal (7.5 in short of its centre)
    chassis.cancelMotion();
    chassis.moveToPoint(120, 48, 600, {.forwards = false, .maxSpeed = 50}); // last 3 in
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 48, {.distance = 7.5});
    score();
}

}
