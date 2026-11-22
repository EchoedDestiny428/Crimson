#include "autons/routine.hpp"
#include "subsystems/side_toggle.hpp"
#include "subsystems/elevator/claw.hpp"
#include "subsystems/elevator/macros.hpp"
#include <cmath>
#include <functional>

namespace autons::skills {

void setup() {}

constexpr int kLeftLoops = 4;

// Mechanism: aim the lift and arm while driving onto a goal, drop, then home (arm onto the hardstop) without waiting,
// so the next pickup starts from home. Each goal's first score is L1.
namespace claw = subsystems::elevator::claw;
namespace macros = subsystems::elevator::macros;
using subsystems::elevator::constants::Goal;

// Driver stage setpoint (1-7), raised kGoalTierStepCm per goal tier like the Right button.
macros::Pose stage(int level, Goal goal = Goal::Alliance) {
    const auto& setpoint = subsystems::elevator::constants::kStageSetpoints[level];
    return {setpoint.lift_cm + static_cast<int>(goal) * subsystems::elevator::constants::kGoalTierStepCm,
            setpoint.arm_deg};
}

// For aim_staged(): true once the robot is kBackedOutIn from where it is now, i.e. off the loader with room to swing
// the arm. kBackedOutIn is the goal curve's straight exit leg.
constexpr float kBackedOutIn = 4.0f;
// Loader-to-goal curves. Each loop drives its curve onto the goal and the same path in reverse back to the loader,
// so both halves come from one definition (tuned in LoopTest). lead moves the carrot nearest the target toward the
// inside of the turn: 0.5 in onto the goal; the reverse trip's own 2 in keeps it off the wall by the loader.
constexpr crimson::CurveParams kLeftGoalCurve{.forwards = false, .minSpeed = 50, .exitBias = 0.6f, .entryBias = 0.45f,
                                              .finalMaxSpeed = 50, .finalTimeout = 1050, .lead = 0.5f};
constexpr crimson::CurveParams kRightGoalCurve = kLeftGoalCurve; // mirrored by the headings passed in
constexpr crimson::CurveParams kToLoader{.minSpeed = 50, .finalMaxSpeed = 70, .finalTimeout = 600, .lead = 2.0f,
                                         .deeper = 3.0f}; // then straight 3 in into the loader to load

std::function<bool()> backed_out() {
    const lemlib::Pose from = chassis.getPose();
    return [from] {
        const lemlib::Pose now = chassis.getPose();
        return std::hypot(now.x - from.x, now.y - from.y) >= kBackedOutIn;
    };
}

// skills, made in PointWeaver.
// Field frame: (0, 0) is the corner on blue's right, +X toward red, heading 0 faces red, clockwise. Inches.

// start
void start() {
    chassis.setPose(135.5, 72, -90);
    claw::grab(); // hold the preload; the claw boots open

    side_toggle_motors.move_velocity(-200);
    // Home while the toggle spins: the arm pushes onto its hardstop and resets its encoder there.
    macros::home_async();
    pros::delay(1200);
    side_toggle_motors.brake();

    chassis.moveToPoint(120, 48, 600, {.forwards = false}); // goal 1
    macros::aim(stage(1, Goal::Neutral)); // preload
    chassis.waitUntil(10);
    chassis.cancelMotion();
    chassis.moveToPoint(120, 48, 900, {.forwards = false, .maxSpeed = 50}); // last 65%
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 48, {.distance = 7.5});
    macros::drop();
    macros::home_async(); // home after every score
    chassis.donut(0, 1000, {.forwards = false});
    chassis.moveToPoint(135.5, 48, 500); // claw on yellow pin (141.8, 48)
    chassis.waitUntil(5.6);
    chassis.cancelMotion();
    chassis.moveToPoint(135.5, 48, 300, {.maxSpeed = 50}); // last 30%
    chassis.waitUntilDone();
    macros::pickup();
    chassis.moveToPoint(120, 48, 550, {.forwards = false, .maxSpeed = 50}); // goal 1
    macros::aim(stage(2, Goal::Neutral));
    chassis.waitUntil(6.2);
    chassis.cancelMotion();
    chassis.moveToPoint(120, 48, 650, {.forwards = false, .maxSpeed = 50}); // last 60%
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 48, {.distance = 7.5});
    macros::drop();
    macros::home_async(); // home after every score
    chassis.donut(45, 950, {.forwards = false}); // facing point 4
    chassis.moveToPoint(134, 36, 700); // 2 in further in x than PointWeaver's 132
    chassis.turnToPoint(108, 12, 750, {.forwards = false}); // facing point 5
    chassis.moveToPoint(108, 12, 1300, {.forwards = false});
    chassis.turnToPoint(141.6, 13, 550); // facing point 6
    chassis.moveToPose(135.3, 13, 0, 900); // claw on red left loader (141.6, 13)
    chassis.waitUntil(21.85);
    chassis.cancelMotion();
    chassis.moveToPose(135.3, 13, 0, 650, {.maxSpeed = 50}); // last 20%
    chassis.waitUntilDone();
    macros::pickup();
}

// left loop
// pass 0 to kLeftLoops - 1: each one starts and ends on the left loader, so they chain.
void left_loop(int pass) {
    // Lift first while backing off the loader, then the arm, then the rest of the lift (curve() only returns once its
    // last leg starts, so this has to come before it).
    macros::aim_staged(stage(3 + pass, Goal::Neutral), backed_out());
    // Back off the loader, swing around and back onto goal 1 heading 90 (back toward +Y).
    chassis.curve(120, 48, 0, 90, 2, 10, 1200, kLeftGoalCurve); // goal 1
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 48, {.distance = 7.5});
    macros::drop();
    macros::home_async(); // home after every score
    // The same curve in reverse, ending square on the loader at heading 0.
    chassis.curveReverse(135.3, 13, 120, 48, 0, 90, 2, 10, kLeftGoalCurve, 1000,
                         kToLoader); // claw on red left loader (141.6, 13)
    chassis.waitUntilDone();
    macros::pickup();
}

// transition
void transition() {
    chassis.moveToPoint(132.5, 12, 2150, {.forwards = false});
    chassis.turnToPoint(132.5, 84, 750, {.forwards = false}); // facing point 2
    chassis.moveToPoint(132.5, 84, 2200, {.forwards = false});
    chassis.turnToPoint(120, 96, 550, {.forwards = false}); // facing point 3
    chassis.moveToPoint(120, 96, 1300, {.forwards = false, .maxSpeed = 50}); // red goal 2
    macros::aim(stage(1)); // the loader piece, first on the empty goal
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});
    macros::drop();
    macros::home_async(); // home after every score
    chassis.donut(0, 950, {.forwards = false}); // facing point 4
    chassis.moveToPoint(135.5, 96, 700); // claw on yellow pin (141.8, 96)
    chassis.waitUntilDone();
    macros::pickup();
    chassis.moveToPoint(120, 96, 1200, {.forwards = false, .maxSpeed = 50}); // red goal 2
    macros::aim(stage(2));
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});
    macros::drop();
    macros::home_async(); // home after every score
    chassis.donut(-90, 1750, {.forwards = false});
}

// right start
void right_start() {
    chassis.moveToPoint(120, 113.7, 750); // claw on yellow pin (120, 120)
    chassis.waitUntilDone();
    macros::pickup();
    chassis.moveToPoint(120, 96, 950, {.forwards = false}); // red goal 2
    macros::aim(stage(3));
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});
    macros::drop();
    macros::home_async(); // home after every score
    chassis.donut(180, 1750, {.forwards = false}); // facing point 3
    chassis.moveToPoint(102.3, 96, 750); // claw on yellow pin (96, 96)
    chassis.waitUntilDone();
    macros::pickup();
    chassis.moveToPoint(120, 96, 950, {.forwards = false}); // red goal 2
    macros::aim(stage(4));
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});
    macros::drop();
    macros::home_async(); // home after every score
    chassis.donut(-82.09, 1900, {.forwards = false, .slowSpeed = 19}); // facing point 5
    // To the right loader along right_loop's goal curve in reverse, ending square at heading 0.
    chassis.curveReverse(135.3, 131, 120, 96, 0, -90, 2, 10, kRightGoalCurve, 1000,
                         kToLoader); // claw on red right loader (141.6, 131)
    chassis.waitUntilDone();
    macros::pickup();
}

// right loop
void right_loop() {
    macros::aim_staged(stage(5), backed_out()); // like the left loop
    // Mirror of the left loop: back onto red goal 2 heading -90 (back toward -Y), then the same curve in reverse.
    chassis.curve(120, 96, 0, -90, 2, 10, 1200, kRightGoalCurve); // red goal 2
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(120, 96, {.distance = 7.5});
    macros::drop();
    macros::home_async(); // home after every score
    chassis.curveReverse(135.3, 131, 120, 96, 0, -90, 2, 10, kRightGoalCurve, 1000,
                         kToLoader); // claw on red right loader (141.6, 131)
    chassis.waitUntilDone();
    macros::pickup();
}

// ending
void ending() {
    chassis.moveToPoint(132, 132, 1000, {.forwards = false});
    chassis.turnToPoint(72, 72, 600, {.forwards = false}); // facing point 2
    chassis.moveToPoint(72, 72, 2500, {.forwards = false}); // goal 0
    macros::aim(stage(1, Goal::Center));
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(72, 72, {.distance = 7.5});
    macros::drop();
    macros::home_async(); // home after every score
    chassis.donut(135, 3250, {.forwards = false}); // facing point 3
    chassis.moveToPoint(52.45, 52.45, 1000); // claw on yellow pin (48, 48)
    chassis.waitUntilDone();
    macros::pickup();
    chassis.moveToPoint(72, 72, 1200, {.forwards = false}); // goal 0
    macros::aim(stage(2, Goal::Center));
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(72, 72, {.distance = 7.5});
    macros::drop();
    macros::home_async(); // home after every score
    chassis.moveToPoint(28.45, 28.45, 1800); // claw on yellow pin (24, 24)
    chassis.waitUntilDone();
    macros::pickup();
    chassis.moveToPoint(72, 72, 1950, {.forwards = false}); // goal 0
    macros::aim(stage(3, Goal::Center));
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(72, 72, {.distance = 7.5});
    macros::drop();
    macros::home_async(); // home after every score
}

void run() {
    start();
    for (int pass = 0; pass < kLeftLoops; ++pass) {
        left_loop(pass);
    }
    transition();
    right_start();
    right_loop(); // one pass: score red goal 2 L5, back to the right loader for the last pin
    ending();
}




}
