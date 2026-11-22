#include "autons/routine.hpp"

namespace autons::loop_test {

// One cycle of the skills left loop, path only (no claw, lift or arm), for tuning the curves.
// Starts where a score on goal 1 leaves the robot: back pressed against it, facing 90 (front toward -Y),
// which is setPoseFromGoal(120, 48, {.distance = 7.5}).

void setup() {}

namespace {

// The loader-to-goal curve that works, shared by both halves so the trip out is the same path driven backwards.
constexpr float kLoaderX = 135.3f; // claw on red left loader (141.6, 13)
constexpr float kLoaderY = 13.0f;
constexpr float kGoalX = 120.0f;   // goal 1
constexpr float kGoalY = 48.0f;
constexpr crimson::CurveParams kGoalCurve{.forwards = false, .minSpeed = 50, .exitBias = 0.6f, .entryBias = 0.45f,
                                          .finalMaxSpeed = 50, .finalTimeout = 1050, // 150 ms under skills' 1200
                                          .lead = 0.5f}; // curve() only: entry carrot toward the loader side

}

void run() {
    chassis.setPose(120, 40.5, 90);

    // Goal 1 to the left loader: the curve below run in reverse, ending square on the loader at heading 0.
    chassis.curveReverse(kLoaderX, kLoaderY, kGoalX, kGoalY, 0, 90, 2, 10, kGoalCurve, 1000,
                         {.minSpeed = 50, .finalMaxSpeed = 70, .finalTimeout = 900,
                          .lead = 2.0f}); // lead keeps it off the south wall

    // Left loader back onto goal 1 heading 90, the same curve skills uses.
    chassis.curve(kGoalX, kGoalY, 0, 90, 2, 10, 1200, kGoalCurve);
    chassis.waitUntilDone();
    chassis.setPoseFromGoal(kGoalX, kGoalY, {.distance = 7.5});
}

}
