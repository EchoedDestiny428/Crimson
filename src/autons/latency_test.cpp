#include "autons/routine.hpp"
#include "config/controls.hpp"
#include <atomic>
#include <cmath>
#include <cstdint>

// Camera latency and drivetrain limits, in one logged run. Analyse the log with tools/latency_test.py.
//
// Start: robot centre 24 in from a goal's centre, back to the goal and square to it, tag in the middle of the camera.
// Needs about 4 ft of clear floor in front of the robot for the full-speed bursts.
// Run: pick "Latency test" in the selector (competition switch / field), or press LEFT in driver control.
// Abort: push either stick past half; the driver gets the robot back.
//
// Phases (logged as test_phase):
//   1 settle            still, camera locks
//   2-4 wiggle          turn 20 deg each side of the tag and back, slow / medium / fast  -> latency (turning)
//   5-6 drive           7 in toward the goal and back, slow / fast                       -> latency (driving)
//   7 settle
//   8 forward burst     full power forward                                               -> top speed, acceleration
//   9 brake                                                                              -> braking
//   10 reverse burst    full power back toward the start
//   11 brake
//   12 spin burst right full power turn clockwise                                        -> top turn rate, turn accel
//   13 brake
//   14 spin burst left  full power turn counterclockwise
//   15 brake
//   16 return           drive back to the start pose
//   17 done
namespace autons::latency_test {

namespace {

constexpr float kWiggleDeg = 20.0f;
constexpr int kWiggleSpeeds[] = {40, 70, 110};
constexpr int kWiggleRepeats = 3;
constexpr float kDriveIn = 7.0f;
constexpr float kDriveSpeeds[] = {50.0f, 100.0f};
constexpr int kDriveRepeats = 2;
constexpr std::uint32_t kSettleMs = 1000;
constexpr std::uint32_t kBrakeMs = 800;
// Full-power time. Shorten it if the floor in front of the robot is short: about 30 in of travel at 600 ms.
constexpr std::uint32_t kBurstMs = 600;
constexpr std::uint32_t kSpinMs = 1200;
// The reverse burst stops this close to where the forward burst started, so it can't run into the goal.
constexpr float kReturnMarginIn = 3.0f;
constexpr int kMotionTimeoutMs = 3000;
constexpr int kAbortStick = 64;
constexpr std::uint32_t kPollMs = 10;

std::atomic<int> current_phase{0};

bool aborted() {
    return std::abs(controller.get_analog(controls::kThrottle)) > kAbortStick ||
           std::abs(controller.get_analog(controls::kTurn)) > kAbortStick;
}

bool pause(std::uint32_t ms) {
    const std::uint32_t start = pros::millis();
    while (pros::millis() - start < ms) {
        if (aborted()) {
            return false;
        }
        pros::delay(kPollMs);
    }
    return true;
}

// Waits for the LemLib motion just started; false (and the motion cancelled) on abort.
bool finish_motion() {
    pros::delay(kPollMs);
    while (chassis.isInMotion()) {
        if (aborted()) {
            chassis.cancelAllMotions();
            return false;
        }
        pros::delay(kPollMs);
    }
    return true;
}

void drive(int left, int right) {
    left_motors.move(left);
    right_motors.move(right);
}

// Distance along the starting heading from the starting point, in inches.
float along(const lemlib::Pose& start) {
    const lemlib::Pose now = chassis.getPose();
    const float h = lemlib::degToRad(start.theta);
    return (now.x - start.x) * std::cos(h) - (now.y - start.y) * std::sin(h);
}

bool burst(int phase, int left, int right, std::uint32_t ms, const lemlib::Pose& start, bool stop_near_start) {
    current_phase = phase;
    const std::uint32_t begin = pros::millis();
    drive(left, right);
    while (pros::millis() - begin < ms) {
        if (aborted() || (stop_near_start && along(start) <= kReturnMarginIn)) {
            break;
        }
        pros::delay(kPollMs);
    }
    drive(0, 0);
    left_motors.brake();
    right_motors.brake();
    return !aborted();
}

bool steps(const lemlib::Pose& start) {
    current_phase = 1;
    if (!pause(kSettleMs)) {
        return false;
    }

    for (int i = 0; i < 3; ++i) {
        current_phase = 2 + i;
        const lemlib::TurnToHeadingParams turn{.maxSpeed = kWiggleSpeeds[i]};
        for (int r = 0; r < kWiggleRepeats; ++r) {
            for (const float offset : {kWiggleDeg, -kWiggleDeg}) {
                chassis.turnToHeading(start.theta + offset, kMotionTimeoutMs, turn);
                if (!finish_motion()) {
                    return false;
                }
            }
        }
        chassis.turnToHeading(start.theta, kMotionTimeoutMs, turn);
        if (!finish_motion()) {
            return false;
        }
    }

    // The back faces the goal, so toward the goal is backwards.
    const float h = lemlib::degToRad(start.theta);
    const float near_x = start.x - kDriveIn * std::cos(h);
    const float near_y = start.y + kDriveIn * std::sin(h);
    for (int i = 0; i < 2; ++i) {
        current_phase = 5 + i;
        for (int r = 0; r < kDriveRepeats; ++r) {
            chassis.moveToPoint(near_x, near_y, kMotionTimeoutMs, {.forwards = false, .maxSpeed = kDriveSpeeds[i]});
            if (!finish_motion()) {
                return false;
            }
            chassis.moveToPoint(start.x, start.y, kMotionTimeoutMs, {.forwards = true, .maxSpeed = kDriveSpeeds[i]});
            if (!finish_motion()) {
                return false;
            }
        }
    }

    current_phase = 7;
    chassis.turnToHeading(start.theta, kMotionTimeoutMs);
    if (!finish_motion() || !pause(kSettleMs)) {
        return false;
    }

    const lemlib::Pose burst_start = chassis.getPose();
    if (!burst(8, 127, 127, kBurstMs, burst_start, false)) {
        return false;
    }
    current_phase = 9;
    if (!pause(kBrakeMs) || !burst(10, -127, -127, kBurstMs, burst_start, true)) {
        return false;
    }
    current_phase = 11;
    if (!pause(kBrakeMs) || !burst(12, 127, -127, kSpinMs, burst_start, false)) {
        return false;
    }
    current_phase = 13;
    if (!pause(kBrakeMs) || !burst(14, -127, 127, kSpinMs, burst_start, false)) {
        return false;
    }
    current_phase = 15;
    if (!pause(kBrakeMs)) {
        return false;
    }

    current_phase = 16;
    chassis.moveToPoint(start.x, start.y, kMotionTimeoutMs);
    if (!finish_motion()) {
        return false;
    }
    chassis.turnToHeading(start.theta, kMotionTimeoutMs);
    return finish_motion();
}

}

void setup() {}

void run() {
    chassis.cancelAllMotions();
    const lemlib::Pose start = chassis.getPose();
    const auto left_brake = left_motors.get_brake_mode_all();
    const auto right_brake = right_motors.get_brake_mode_all();
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

    const bool done = steps(start);
    drive(0, 0);
    current_phase = done ? 17 : -1;
    pause(500);
    current_phase = 0;

    if (!left_brake.empty()) {
        left_motors.set_brake_mode_all(left_brake.front());
    }
    if (!right_brake.empty()) {
        right_motors.set_brake_mode_all(right_brake.front());
    }
    controller.rumble(done ? "." : "-");
}

int phase() {
    return current_phase;
}

}
