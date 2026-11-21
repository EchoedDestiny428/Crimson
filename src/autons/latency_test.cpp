#include "autons/latency_analysis.hpp"
#include "autons/routine.hpp"
#include "config/controls.hpp"
#include "pros/misc.hpp"
#include "pros/screen.hpp"
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
// Results: when the test ends, the brain works them out itself (src/autons/latency_analysis.cpp) and shows them on
// its screen until the screen is touched. No laptop or SD card needed.
//
// Recording: only while the test runs, every 10 ms, just the columns the analysis needs. Each row is kept in memory
// for the on-brain results, and also goes:
//   - live down the program's terminal ("LT," lines; USB cable or the controller's radio), which
//     tools/latency-capture.ps1 turns into logs/latency_*.csv as the test runs;
//   - to /usd/latency_NNN.csv if there is an SD card. Hold UP and press DOWN in driver control to send the newest one
//     down the terminal (tools/pull-logs.ps1 catches it).
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
constexpr const char* kColumns = "time_ms,pose_x_in,pose_y_in,pose_theta_deg,vision_tag_count,vision_read_ms,"
                                 "vision_raw_u_px,vision_raw_v_px,test_phase,left_motor_rpm,right_motor_rpm";

constexpr std::size_t kMaxSamples = 9000; // 90 s at 10 ms; the test takes about 35 s
constexpr std::uint32_t kStopWaitMs = 1000;

std::atomic<bool> recording{false};
std::atomic<bool> recorder_done{true};
std::FILE* record_file = nullptr;
char record_name[32] = "";
char last_name[32] = "";
std::vector<latency_analysis::Sample> samples;

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

latency_analysis::Sample take_sample() {
    const lemlib::Pose pose = chassis.getPose();
    const vision::localizer::Snapshot camera = vision::localizer::snapshot();
    return {pros::millis(),
            pose.x,
            pose.y,
            pose.theta,
            camera.tags,
            camera.read_ms,
            camera.raw ? camera.raw->u : NAN,
            camera.raw ? camera.raw->v : NAN,
            current_phase.load(),
            average(left_motors.get_actual_velocity_all()),
            average(right_motors.get_actual_velocity_all())};
}

void format_row(const latency_analysis::Sample& s, char* out, std::size_t size) {
    std::snprintf(out, size, "%lu,%.3f,%.3f,%.3f,%d,%lu,%.2f,%.2f,%d,%.1f,%.1f", static_cast<unsigned long>(s.ms),
                  s.x_in, s.y_in, s.heading_deg, s.tags, static_cast<unsigned long>(s.read_ms), s.u_px, s.v_px,
                  s.phase, s.left_rpm, s.right_rpm);
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
        const latency_analysis::Sample sample = take_sample();
        if (samples.size() < kMaxSamples) {
            samples.push_back(sample);
        }
        format_row(sample, row, sizeof(row));
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
    samples.clear();
    samples.reserve(kMaxSamples);
    record_name[0] = '\0';
    if (open_record_file()) {
        std::fprintf(record_file, "%s\n", kColumns);
    }
    std::printf("LT_BEGIN,%s\nLT_HEADER,%s\n", record_name[0] != '\0' ? record_name : "live-only", kColumns);
    recording = true;
    recorder_done = false;
    pros::Task task(record_loop, "Latency rec");
}

// True once the recorder task has finished, so `samples` is safe to read.
bool stop_recording() {
    recording = false;
    const std::uint32_t start = pros::millis();
    while (!recorder_done && pros::millis() - start < kStopWaitMs) {
        pros::delay(kRecordMs);
    }
    return recorder_done;
}

// ---- Results on the brain screen ----

constexpr std::uint32_t kBackground = 0x1E1E24;
constexpr std::uint32_t kText = 0xF8F9FA;
constexpr std::uint32_t kAccent = 0x00F0B5;
constexpr std::uint32_t kWarn = 0xFBBF24;
constexpr int kLineX = 12;
constexpr int kLineY = 6;
constexpr int kLineStep = 23;
constexpr double kFlatSharpness = 1.1;

std::atomic<bool> results_shown{false};
std::int32_t results_presses = 0;

void line(int row, std::uint32_t color, const char* text) {
    pros::screen::set_pen(color);
    pros::screen::print(pros::E_TEXT_MEDIUM, kLineX, kLineY + row * kLineStep, "%s", text);
}

void latency_text(char* out, std::size_t size, const char* name, const latency_analysis::Latency& l) {
    if (!l.valid) {
        std::snprintf(out, size, "%s --", name);
    } else {
        std::snprintf(out, size, "%s %d ms x%.1f%s", name, l.ms, l.sharpness, l.sharpness < kFlatSharpness ? "?" : "");
    }
}

void show_results(const latency_analysis::Results& r) {
    results_presses = pros::screen::touch_status().press_count;
    results_shown = true;
    pros::screen::set_eraser(kBackground);
    pros::screen::erase();

    char text[96];
    char turning[40];
    char driving[40];
    if (r.completed) {
        std::snprintf(text, sizeof(text), "LATENCY TEST: DONE (%zu rows)", samples.size());
    } else {
        std::snprintf(text, sizeof(text), "LATENCY TEST: ABORTED in phase %d", r.last_phase);
    }
    line(0, r.completed ? kText : kWarn, text);

    if (r.latency_valid) {
        std::snprintf(text, sizeof(text), "kCameraLatencyMs = %d", r.latency_ms);
        line(1, kAccent, text);
    } else {
        line(1, kWarn, "Camera delay: too few tag views");
    }
    latency_text(turning, sizeof(turning), "turn", r.turning);
    latency_text(driving, sizeof(driving), "drive", r.driving);
    std::snprintf(text, sizeof(text), "%s  %s", turning, driving);
    line(2, kText, text);

    if (r.drive_valid) {
        std::snprintf(text, sizeof(text), "DRIVE top %.1f fwd / %.1f rev in/s", r.top_forward, r.top_reverse);
        line(3, kAccent, text);
        std::snprintf(text, sizeof(text), "accel %.0f/%.0f  brake %.0f/%.0f in/s2", r.accel_forward,
                      r.accel_reverse, r.brake_forward, r.brake_reverse);
        line(4, kText, text);
        std::snprintf(text, sizeof(text), "90%% speed in %.0f ms  motors %.1f in/s", r.ms_to_90, r.motors_forward);
        line(5, kText, text);
    } else {
        line(3, kWarn, "DRIVE: bursts did not run");
    }
    if (r.turn_valid) {
        std::snprintf(text, sizeof(text), "TURN top %.0f R / %.0f L deg/s", r.turn_right, r.turn_left);
        line(6, kAccent, text);
        std::snprintf(text, sizeof(text), "accel %.0f/%.0f  brake %.0f/%.0f deg/s2", r.turn_accel_right,
                      r.turn_accel_left, r.turn_brake_right, r.turn_brake_left);
        line(7, kText, text);
        std::snprintf(text, sizeof(text), "90%% turn rate in %.0f ms", r.turn_ms_to_90);
        line(8, kText, text);
    } else {
        line(6, kWarn, "TURN: spins did not run");
    }
    line(9, kWarn, "Touch the screen to go back");
}

void print_results(const latency_analysis::Results& r) {
    std::printf("LT_RESULT,completed=%d,rows=%zu,latency_ms=%d,turn_ms=%d,turn_x=%.2f,drive_ms=%d,drive_x=%.2f\n",
                r.completed, samples.size(), r.latency_valid ? r.latency_ms : -1, r.turning.ms, r.turning.sharpness,
                r.driving.ms, r.driving.sharpness);
    std::printf("LT_RESULT,top_fwd=%.1f,top_rev=%.1f,accel_fwd=%.1f,accel_rev=%.1f,brake_fwd=%.1f,brake_rev=%.1f,"
                "ms_to_90=%.0f,motors_fwd=%.1f\n",
                r.top_forward, r.top_reverse, r.accel_forward, r.accel_reverse, r.brake_forward, r.brake_reverse,
                r.ms_to_90, r.motors_forward);
    std::printf("LT_RESULT,turn_right=%.0f,turn_left=%.0f,turn_accel_right=%.0f,turn_accel_left=%.0f,"
                "turn_brake_right=%.0f,turn_brake_left=%.0f,turn_ms_to_90=%.0f\n",
                r.turn_right, r.turn_left, r.turn_accel_right, r.turn_accel_left, r.turn_brake_right,
                r.turn_brake_left, r.turn_ms_to_90);
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
    const bool recorded = stop_recording();
    current_phase = 0;

    if (!left_brake.empty()) {
        left_motors.set_brake_mode_all(left_brake.front());
    }
    if (!right_brake.empty()) {
        right_motors.set_brake_mode_all(right_brake.front());
    }
    if (recorded) {
        const latency_analysis::Results results = latency_analysis::analyze(samples);
        print_results(results);
        show_results(results);
    }
    controller.rumble(done ? "." : "-");
}

bool keep_results_on_screen() {
    if (!results_shown) {
        return false;
    }
    if (pros::screen::touch_status().press_count != results_presses) {
        results_shown = false;
        return false;
    }
    return true;
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
