#include "autons/routine.hpp"
#include "config/controls.hpp"
#include "pros/misc.hpp"
#include "vision/localizer.hpp"
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

// Camera latency and drivetrain limits, in one recorded run. Analyse it with tools/latency_test.py.
//
// Start: robot centre 24 in from a goal's centre, back to the goal and square to it, tag in the middle of the camera.
// Needs about 4 ft of clear floor in front of the robot for the full-speed bursts.
// Run: pick "Latency test" in the selector (competition switch / field), or press LEFT in driver control.
// Abort: push either stick past half; the driver gets the robot back.
//
// Recording: only while the test runs, every 10 ms, just the columns the analysis needs. Each row goes two places:
//   - live down the program's terminal ("LT," lines; USB cable or the controller's radio), which
//     tools/latency-capture.ps1 turns into logs/latency_*.csv as the test runs;
//   - /usd/latency_NNN.csv on the SD card, in case the live copy drops lines. Hold UP and press DOWN in driver control
//     to send the newest one down the terminal (tools/pull-logs.ps1 catches it).
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

// ---- Recording ----

constexpr std::uint32_t kRecordMs = 10;
// Live rows go out every kStreamEveryRows-th sample. Raise it (2 = every 20 ms) if the radio drops lines.
constexpr std::uint32_t kStreamEveryRows = 1;
constexpr std::uint32_t kFlushEveryRows = 25;
constexpr std::uint32_t kStopWaitMs = 300;
constexpr const char* kColumns = "time_ms,pose_x_in,pose_y_in,pose_theta_deg,vision_tag_count,vision_read_ms,"
                                 "vision_raw_u_px,vision_raw_v_px,test_phase,left_motor_rpm,right_motor_rpm";

std::atomic<bool> recording{false};
std::atomic<bool> recorder_done{true};
std::FILE* record_file = nullptr;
char record_name[32] = "";
char last_name[32] = "";

double average(const std::vector<double>& values) {
    double sum = 0.0;
    int count = 0;
    for (const double value : values) {
        if (std::isfinite(value)) {
            sum += value;
            ++count;
        }
    }
    return count > 0 ? sum / count : NAN;
}

void format_row(char* out, std::size_t size) {
    const lemlib::Pose pose = chassis.getPose();
    const vision::localizer::Snapshot camera = vision::localizer::snapshot();
    std::snprintf(out, size, "%lu,%.3f,%.3f,%.3f,%d,%lu,%.2f,%.2f,%d,%.1f,%.1f",
                  static_cast<unsigned long>(pros::millis()), pose.x, pose.y, pose.theta, camera.tags,
                  static_cast<unsigned long>(camera.read_ms), camera.raw ? camera.raw->u : NAN,
                  camera.raw ? camera.raw->v : NAN, current_phase.load(), average(left_motors.get_actual_velocity_all()),
                  average(right_motors.get_actual_velocity_all()));
}

// Picks /usd/latency_NNN.csv with the next free number; false if there is no SD card.
bool open_record_file() {
    if (pros::usd::is_installed() == 0) {
        return false;
    }
    for (unsigned n = 1; n < 1000; ++n) {
        char path[48];
        std::snprintf(record_name, sizeof(record_name), "latency_%03u.csv", n);
        std::snprintf(path, sizeof(path), "/usd/%s", record_name);
        if (std::FILE* existing = std::fopen(path, "r")) {
            std::fclose(existing);
            continue;
        }
        record_file = std::fopen(path, "w");
        return record_file != nullptr;
    }
    return false;
}

void record_loop() {
    std::uint32_t now = pros::millis();
    std::uint32_t rows = 0;
    char row[192];
    while (recording) {
        format_row(row, sizeof(row));
        if (record_file != nullptr) {
            std::fputs(row, record_file);
            std::fputc('\n', record_file);
            if (rows % kFlushEveryRows == 0) {
                std::fflush(record_file);
            }
        }
        if (rows % kStreamEveryRows == 0) {
            std::printf("LT,%s\n", row);
        }
        ++rows;
        pros::Task::delay_until(&now, kRecordMs);
    }
    if (record_file != nullptr) {
        std::fclose(record_file);
        record_file = nullptr;
        std::strcpy(last_name, record_name);
    }
    std::printf("LT_END,%s,%lu\n", record_name[0] != '\0' ? record_name : "live-only", static_cast<unsigned long>(rows));
    recorder_done = true;
}

void start_recording() {
    record_name[0] = '\0';
    if (open_record_file()) {
        std::fprintf(record_file, "%s\n", kColumns);
    }
    std::printf("LT_BEGIN,%s\nLT_HEADER,%s\n", record_name[0] != '\0' ? record_name : "live-only", kColumns);
    recording = true;
    recorder_done = false;
    pros::Task task(record_loop, "Latency rec");
}

void stop_recording() {
    recording = false;
    const std::uint32_t start = pros::millis();
    while (!recorder_done && pros::millis() - start < kStopWaitMs) {
        pros::delay(kRecordMs);
    }
}

// ---- The test ----

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

    start_recording();
    const bool done = steps(start);
    drive(0, 0);
    current_phase = done ? 17 : -1;
    pause(500);
    stop_recording();
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

void dump_latest() {
    if (pros::usd::is_installed() == 0) {
        std::printf("LOG_DUMP_ERROR,sd_not_installed\n");
        return;
    }
    char name[32];
    std::strcpy(name, last_name);
    if (name[0] == '\0') {
        // Nothing recorded since boot: send the newest test file already on the card.
        char list[4096]{};
        if (pros::usd::list_files("/", list, sizeof(list)) < 0) {
            std::printf("LOG_DUMP_ERROR,list_files_failed\n");
            return;
        }
        for (char* entry = std::strtok(list, "\n"); entry != nullptr; entry = std::strtok(nullptr, "\n")) {
            if (std::strncmp(entry, "latency_", 8) == 0 && std::strstr(entry, ".csv") != nullptr &&
                std::strlen(entry) < sizeof(name) && std::strcmp(entry, name) > 0) {
                std::strcpy(name, entry);
            }
        }
    }
    if (name[0] == '\0') {
        std::printf("LOG_DUMP_ERROR,no_latency_file\n");
        return;
    }

    char path[48];
    std::snprintf(path, sizeof(path), "/usd/%s", name);
    std::FILE* file = std::fopen(path, "r");
    if (file == nullptr) {
        std::printf("LOG_DUMP_ERROR,open_failed,%s\n", name);
        return;
    }
    std::printf("LOG_DUMP_BEGIN,%s\n", name);
    char line[256];
    while (std::fgets(line, sizeof(line), file) != nullptr) {
        std::printf("%s", line);
        pros::delay(2); // lets the terminal keep up
    }
    std::fclose(file);
    std::printf("LOG_DUMP_END,%s\nLOG_DUMP_COMPLETE\n", name);
}

}
