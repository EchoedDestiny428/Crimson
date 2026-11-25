#include "autons/routine.hpp"
#include "pros/rtos.hpp"
#include "subsystems/side_toggle.hpp"
#include "subsystems/elevator/claw.hpp"
#include "subsystems/elevator/macros.hpp"

namespace autons::match::snacky {

// Blue snacky. Blue goal 2 (24, 48) is red goal 2 (120, 96) turned 180 deg about the field centre, so the
// moves around it are copied from skills' transition / right_start at red goal 2: (x, y) -> (144 - x, 144 - y),
// heading + 180. Start like skills mirrored onto the blue wall.
// Mechanism calls are commented out for now (drive only); uncomment them to run the claw, lift and chain bar.
// Mechanism as in skills: only the lift moves while driving onto a goal; once backed against it, swing the chain bar,
// drop, then home without waiting. The one exception is the wall pin: the claw can't come down on the wall, so the
// home finishes before driving there.

namespace claw = subsystems::elevator::claw;
namespace macros = subsystems::elevator::macros;
using subsystems::elevator::constants::Goal;

// Driver stage setpoint (1-7), raised kGoalTierStepCm per goal tier like the Right button. Same as skills.
macros::Pose stage(int level, Goal goal = Goal::Alliance) {
    const auto& setpoint = subsystems::elevator::constants::kStageSetpoints[level];
    return {setpoint.lift_cm + static_cast<int>(goal) * subsystems::elevator::constants::kGoalTierStepCm,
            setpoint.arm_deg};
}

void score() {
    macros::swing_arm(); // chain bar only once backed against the goal
    macros::drop();
    macros::home_async(); // home after every score
}

void setup() {}

void run() {
    chassis.setPose(8.5, 72, -90);
    // claw::grab(); // hold the preload; the claw boots open

    side_toggle_motors.move_relative(-8 * 360.0, 200); // 3 turns (~0.9 s at 200 rpm)
    // Home while the toggle spins: the arm pushes onto its hardstop and resets its encoder there.
    // macros::home_async();
    pros::delay(900);
    side_toggle_motors.brake();

    chassis.moveToPoint(24, 48, 950, {.forwards = false,.maxSpeed=50}); // blue goal 2
    // macros::aim_lift(stage(1)); // preload
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(24, 48, {.distance = 7.5});
    // score();

    // skills transition: donut(0, 950) + moveToPoint(135.5, 96, 700)
    chassis.donut(180, 950, {.forwards = false, .slowSpeed = 20}); // facing the wall pin
    chassis.waitUntilDone();
    // macros::wait_settled(); // homed before driving at the wall, or the claw comes down on it
    chassis.moveToPoint(8.5, 48, 700); // claw on wall pin (2.2, 48)
    chassis.waitUntilDone();
    // macros::pickup();
    chassis.moveToPoint(24, 48, 950, {.forwards = false}); // blue goal 2
    // macros::aim_lift(stage(2));
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(24, 48, {.distance = 7.5});
    // score();

    // skills transition's last donut(-90, 1750) + right_start's moveToPoint(120, 113.7, 750)
    chassis.donut(90, 1750, {.forwards = false, .slowSpeed = 20}); // facing pin (24, 24)
    chassis.moveToPoint(24, 30.3, 750); // claw on yellow pin (24, 24)
    chassis.waitUntilDone();
    // macros::pickup();
    chassis.moveToPoint(24, 48, 950, {.forwards = false}); // blue goal 2
    // macros::aim_lift(stage(3));
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(24, 48, {.distance = 7.5});
    // score();

    // skills right_start: donut(180, 1750) + moveToPoint(102.3, 96, 750)
    chassis.donut(0, 1750, {.forwards = false, .slowSpeed = 20}); // facing pin (48, 48)
    chassis.moveToPoint(41.7, 48, 750); // claw on yellow pin (48, 48)
    chassis.waitUntilDone();
    // macros::pickup();
    chassis.turnToPoint(24,96,500,{.forwards=false});
    chassis.moveToPoint(24, 96, 950, {.forwards = false}); // neutral goal (24, 96)
    // macros::aim_lift(stage(1, Goal::Neutral)); // first score on this goal
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(24, 96, {.distance = 7.5});
    // score();
}

}
