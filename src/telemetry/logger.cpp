#include "telemetry/logger.hpp"
#include "config/controls.hpp"
#include "lemlib/logger/logger.hpp"
#include "pros/error.h"
#include "pros/misc.hpp"
#include "robotconfig.hpp"
#include "vision/localizer.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <new>

namespace telemetry {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// Timing. The sampler takes a row every period (10 ms); while disabled only every kDisabledEvery periods; while the
// ring is over kSlowFill full (the SD card fell behind) every other period, until it drains below kFastFill.
constexpr std::uint32_t kDisabledEvery = 10;
constexpr std::uint32_t kRingMs = 4000; // rows the ring holds while the SD card is busy
constexpr double kSlowFill = 0.6;
constexpr double kFastFill = 0.2;
// The writer wakes every kWriterPollMs, writes once kWriteBytes are waiting, and flushes at least every kFlushMs, so
// a power-off loses at most about that much.
constexpr std::uint32_t kWriterPollMs = 20;
constexpr std::uint32_t kFlushMs = 500;
constexpr std::size_t kBufferBytes = 64 * 1024;
constexpr std::size_t kWriteBytes = 16 * 1024;
constexpr std::size_t kMaxValueChars = 34; // a value and its comma
constexpr std::size_t kMaxNoteChars = 128; // one "# key,value" line

// PointWeaver mark: the id in the low 32 bits, the millis it was set at in the high 32. kNoMark before the first.
constexpr std::uint64_t kNoMark = ~std::uint64_t{0};
std::atomic<std::uint64_t> g_mark{kNoMark};

struct Column {
    const char* name;
    int decimals; // 0: written as an integer
};

// Every row starts with these.
constexpr Column kLeadColumns[] = {
    {"time_ms", 0},          // pros::millis(): since the program started
    {"auton_ms", 0},         // since autonomous last started; -1 before
    {"pw_mark", 0},          // latest telemetry::mark() id; -1 before the first of this auton
    {"pw_mark_ms", 0},       // pros::millis() when it was set; -1 before
    {"in_motion", 0},        // chassis.isInMotion()
    {"competition_controller_connected", 0},
    {"field_control", 0},
    {"competition_switch", 0},
    {"autonomous", 0},
    {"disabled", 0},
    {"in_match", 0},
    {"battery_mV", 0},
    {"battery_pct", 1},
    {"battery_mA", 0},
    {"pose_resets", 0},      // chassis.poseResets(): setPose calls so far
    {"pose_reset_dx_in", 3}, // the last setPose's correction, field frame (new - old); 0 after a skipped goal reset
    {"pose_reset_dy_in", 3},
    {"pose_reset_skips", 0}, // goal resets setPoseFromGoal skipped (too far off)
    {"dropped_rows", 0},     // rows lost because the ring was full (the SD card fell far behind)
};
constexpr std::size_t kLeadCount = sizeof(kLeadColumns) / sizeof(kLeadColumns[0]);

// Then these for each motor port, as port_<n>_<name>.
constexpr Column kPortColumns[] = {
    {"position", 2},        // deg (encoder units are set to degrees)
    {"velocity", 1},        // rpm of the cartridge
    {"voltage_mV", 0},      // applied
    {"temperature_C", 0},
    {"current_mA", 0},
    {"target_position", 2}, // deg
    {"target_velocity", 0}, // rpm
};
constexpr std::size_t kPortCount = sizeof(kPortColumns) / sizeof(kPortColumns[0]);

// Then the registered fields.
constexpr int kFieldDecimals = 4;

double int_or_nan(std::int32_t value) {
    return value == PROS_ERR ? kNaN : static_cast<double>(value);
}

// One value with its decimals, trailing zeros trimmed ("12.5", "0", "nan"). Returns the length written.
std::size_t put_value(char* out, double value, int decimals) {
    if (!std::isfinite(value)) {
        std::memcpy(out, "nan", 3);
        return 3;
    }
    if (std::fabs(value) >= 1e9) {
        const int n = std::snprintf(out, kMaxValueChars - 1, "%.6g", value);
        return n > 0 ? static_cast<std::size_t>(n) : 0;
    }
    if (decimals == 0) {
        const int n = std::snprintf(out, kMaxValueChars - 1, "%lld", static_cast<long long>(std::llround(value)));
        return n > 0 ? static_cast<std::size_t>(n) : 0;
    }
    int n = std::snprintf(out, kMaxValueChars - 1, "%.*f", decimals, value);
    if (n <= 0) {
        return 0;
    }
    while (n > 1 && out[n - 1] == '0') {
        --n;
    }
    if (out[n - 1] == '.') {
        --n;
    }
    if (n == 2 && out[0] == '-' && out[1] == '0') {
        out[0] = '0';
        n = 1;
    }
    return static_cast<std::size_t>(n);
}

// Letters, digits, '-' and '_' only, so the name can go in a file name.
void file_safe(char* out, std::size_t size, const char* name) {
    std::size_t n = 0;
    for (const char* c = name != nullptr ? name : ""; *c != '\0' && n + 1 < size; ++c) {
        const bool ok = (*c >= 'a' && *c <= 'z') || (*c >= 'A' && *c <= 'Z') || (*c >= '0' && *c <= '9') ||
                        *c == '-' || *c == '_';
        out[n++] = ok ? *c : '_';
    }
    if (n == 0 && size > 3) {
        std::memcpy(out, "run", 3);
        n = 3;
    }
    out[n] = '\0';
}

// The robot fields read the pose and the vision snapshot once per row, from this cache (the sampler refreshes it).
lemlib::Pose g_pose{0.0f, 0.0f, 0.0f};
vision::localizer::Snapshot g_vision{};

void refresh_robot_cache() {
    g_pose = chassis.getPose();
    g_vision = vision::localizer::snapshot();
}

double chassis_x(const void*) {
    return g_pose.x;
}

double chassis_y(const void*) {
    return g_pose.y;
}

double chassis_theta(const void*) {
    return g_pose.theta;
}

double rotation_position(const void* context) {
    return int_or_nan(static_cast<const pros::Rotation*>(context)->get_position());
}

double rotation_velocity(const void* context) {
    return int_or_nan(static_cast<const pros::Rotation*>(context)->get_velocity());
}

double imu_heading(const void* context) {
    return static_cast<const pros::Imu*>(context)->get_heading();
}

double imu_rotation(const void* context) {
    return static_cast<const pros::Imu*>(context)->get_rotation();
}

double imu_pitch(const void* context) {
    return static_cast<const pros::Imu*>(context)->get_pitch();
}

double imu_roll(const void* context) {
    return static_cast<const pros::Imu*>(context)->get_roll();
}

double imu_gyro_x(const void* context) {
    return static_cast<const pros::Imu*>(context)->get_gyro_rate().x;
}

double imu_gyro_y(const void* context) {
    return static_cast<const pros::Imu*>(context)->get_gyro_rate().y;
}

double imu_gyro_z(const void* context) {
    return static_cast<const pros::Imu*>(context)->get_gyro_rate().z;
}

double imu_accel_x(const void* context) {
    return static_cast<const pros::Imu*>(context)->get_accel().x;
}

double imu_accel_y(const void* context) {
    return static_cast<const pros::Imu*>(context)->get_accel().y;
}

double imu_accel_z(const void* context) {
    return static_cast<const pros::Imu*>(context)->get_accel().z;
}

double imu_status(const void* context) {
    return static_cast<double>(static_cast<int>(static_cast<const pros::Imu*>(context)->get_status()));
}

// Camera estimate from vision/localizer (field frame, inches and degrees, same frame as pose_*).
double vision_locked(const void*) {
    return g_vision.status == vision::localizer::Status::Locked ? 1.0 : 0.0;
}

double vision_tag_count(const void*) {
    return g_vision.tags;
}

double vision_tag_id(const void*) {
    return g_vision.raw ? g_vision.raw->id : -1.0;
}

double vision_range_in(const void*) {
    return g_vision.candidates > 0 ? g_vision.range_in : kNaN;
}

double vision_error_px(const void*) {
    return g_vision.candidates > 0 ? g_vision.error_px : kNaN;
}

double vision_pose_x(const void*) {
    return g_vision.pose.x_in;
}

double vision_pose_y(const void*) {
    return g_vision.pose.y_in;
}

double vision_pose_theta(const void*) {
    return g_vision.pose.heading_deg;
}

// 0 no tags, 1 no fit, 2 accepted, 3 ambiguous, 4 rejected, 5 stale, 6 turning, 7 no history (vision::localizer::Outcome)
double vision_outcome(const void*) {
    return static_cast<double>(static_cast<int>(g_vision.outcome));
}

double vision_correction_in(const void*) {
    return g_vision.correction_in;
}

// How far the camera feed has slid odometry since the last setPose (field-frame inches; config::kCameraFeedsOdometry).
double camera_nudge_x(const void*) {
    return chassis.nudgeX();
}

double camera_nudge_y(const void*) {
    return chassis.nudgeY();
}

// The raw camera read, for recalibrating the camera (and checking its latency) from logs against odometry: when the
// sensor was last read and how many reads so far (a new read is a new vision_reads; rows in between repeat the last
// read), then the largest tag in that read in pixels: its centre, size and four corners. nan when it saw no tag.
double vision_read_ms(const void*) {
    return g_vision.reads > 0 ? static_cast<double>(g_vision.read_ms) : -1.0;
}

double vision_reads(const void*) {
    return g_vision.reads;
}

// context: which number. 0-3 centre u, v, width, height; 4-11 corners TL u, v, TR u, v, BR u, v, BL u, v.
constexpr int kTagValues[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

double vision_tag_value(const void* context) {
    if (!g_vision.raw) {
        return kNaN;
    }
    const vision::localizer::RawTag& tag = *g_vision.raw;
    const int which = *static_cast<const int*>(context);
    switch (which) {
    case 0:
        return tag.u;
    case 1:
        return tag.v;
    case 2:
        return tag.width_px;
    case 3:
        return tag.height_px;
    default: {
        const vision::Pixel& corner = tag.corners[static_cast<std::size_t>((which - 4) / 2)];
        return (which - 4) % 2 == 0 ? corner.u : corner.v;
    }
    }
}

// Raw driver stick, -127..127, before the drive curve, slew and arcade mixing.
double controller_axis(const void* context) {
    const auto axis = *static_cast<const controls::Axis*>(context);
    return int_or_nan(controller.get_analog(axis));
}

}

void route(const char* name) {
    logger.route(name);
}

void mark(int id) {
    const std::uint64_t ms = pros::millis();
    g_mark.store((ms << 32) | static_cast<std::uint32_t>(id));
}

Logger::Logger(const char* directory, std::uint32_t period) : directory(directory), period(period) {}

bool Logger::attach(pros::AbstractMotor* motor) {
    if (motor == nullptr || started) {
        return false;
    }
    bool attached = false;
    const std::vector<std::int8_t> ports = motor->get_port_all();
    for (std::size_t index = 0; index < ports.size(); ++index) {
        if (motorPortCount >= motorPorts.size()) {
            break;
        }
        const std::int8_t port = ports[index];
        bool duplicate = false;
        for (std::size_t i = 0; i < motorPortCount; ++i) {
            const int existingPort = motorPorts[i].port < 0 ? -motorPorts[i].port : motorPorts[i].port;
            const int currentPort = port < 0 ? -port : port;
            if (existingPort == currentPort) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            motorPorts[motorPortCount++] = {motor, static_cast<std::uint8_t>(index), port};
            attached = true;
        }
    }
    return attached;
}

bool Logger::register_field(const char* name, FieldCallback callback, const void* context) {
    if (name == nullptr || callback == nullptr || fieldCount >= fields.size() || started) {
        return false;
    }
    for (std::size_t i = 0; i < fieldCount; ++i) {
        if (std::strcmp(fields[i].name, name) == 0) {
            return false;
        }
    }
    fields[fieldCount++] = {name, callback, context};
    return true;
}

void Logger::register_robot_fields() {
    robotFields = true;
    register_field("pose_x_in", chassis_x);
    register_field("pose_y_in", chassis_y);
    register_field("pose_theta_deg", chassis_theta);
    register_field("odom_vertical_position_cdeg", rotation_position, &vertical_odom);
    register_field("odom_vertical_velocity_cdeg_s", rotation_velocity, &vertical_odom);
    register_field("odom_horizontal_position_cdeg", rotation_position, &horizontal_odom);
    register_field("odom_horizontal_velocity_cdeg_s", rotation_velocity, &horizontal_odom);
    register_field("imu_heading_deg", imu_heading, &imu);
    register_field("imu_rotation_deg", imu_rotation, &imu);
    register_field("imu_pitch_deg", imu_pitch, &imu);
    register_field("imu_roll_deg", imu_roll, &imu);
    register_field("imu_gyro_x_dps", imu_gyro_x, &imu);
    register_field("imu_gyro_y_dps", imu_gyro_y, &imu);
    register_field("imu_gyro_z_dps", imu_gyro_z, &imu);
    register_field("imu_accel_x_g", imu_accel_x, &imu);
    register_field("imu_accel_y_g", imu_accel_y, &imu);
    register_field("imu_accel_z_g", imu_accel_z, &imu);
    register_field("imu_status", imu_status, &imu);
    register_field("vision_locked", vision_locked);
    register_field("vision_tag_count", vision_tag_count);
    register_field("vision_tag_id", vision_tag_id);
    register_field("vision_range_in", vision_range_in);
    register_field("vision_error_px", vision_error_px);
    register_field("vision_pose_x_in", vision_pose_x);
    register_field("vision_pose_y_in", vision_pose_y);
    register_field("vision_pose_theta_deg", vision_pose_theta);
    register_field("vision_outcome", vision_outcome);
    register_field("vision_correction_in", vision_correction_in);
    register_field("camera_nudge_x_in", camera_nudge_x);
    register_field("camera_nudge_y_in", camera_nudge_y);
    register_field("vision_read_ms", vision_read_ms);
    register_field("vision_reads", vision_reads);
    register_field("vision_tag_u_px", vision_tag_value, &kTagValues[0]);
    register_field("vision_tag_v_px", vision_tag_value, &kTagValues[1]);
    register_field("vision_tag_w_px", vision_tag_value, &kTagValues[2]);
    register_field("vision_tag_h_px", vision_tag_value, &kTagValues[3]);
    register_field("vision_tag_tl_u_px", vision_tag_value, &kTagValues[4]);
    register_field("vision_tag_tl_v_px", vision_tag_value, &kTagValues[5]);
    register_field("vision_tag_tr_u_px", vision_tag_value, &kTagValues[6]);
    register_field("vision_tag_tr_v_px", vision_tag_value, &kTagValues[7]);
    register_field("vision_tag_br_u_px", vision_tag_value, &kTagValues[8]);
    register_field("vision_tag_br_v_px", vision_tag_value, &kTagValues[9]);
    register_field("vision_tag_bl_u_px", vision_tag_value, &kTagValues[10]);
    register_field("vision_tag_bl_v_px", vision_tag_value, &kTagValues[11]);
    register_field("stick_throttle", controller_axis, &controls::kThrottle);
    register_field("stick_turn", controller_axis, &controls::kTurn);
}

std::size_t Logger::columnCount() const {
    return kLeadCount + motorPortCount * kPortCount + fieldCount;
}

void Logger::start() {
    if (started) {
        return;
    }
    started = true;
    columns = columnCount();
    capacity = kRingMs / std::max<std::uint32_t>(period, 1) + 1;
    slotRun.reset(new (std::nothrow) std::uint32_t[capacity]);
    slotValues.reset(new (std::nothrow) double[capacity * columns]);
    columnDecimals.reset(new (std::nothrow) std::int8_t[columns]);
    if (!slotRun || !slotValues || !columnDecimals) {
        lemlib::infoSink()->warn("Telemetry: not enough memory for the log ring");
        return;
    }
    std::size_t c = 0;
    for (const Column& column : kLeadColumns) {
        columnDecimals[c++] = static_cast<std::int8_t>(column.decimals);
    }
    for (std::size_t port = 0; port < motorPortCount; ++port) {
        for (const Column& column : kPortColumns) {
            columnDecimals[c++] = static_cast<std::int8_t>(column.decimals);
        }
    }
    for (std::size_t i = 0; i < fieldCount; ++i) {
        columnDecimals[c++] = kFieldDecimals;
    }

    // Below the motion code's priority (default), and the writer below everything that does real work.
    pros::Task sampler([this] { sampleLoop(); }, TASK_PRIORITY_DEFAULT - 1, TASK_STACK_DEPTH_DEFAULT, "Log sample");
    pros::Task writer([this] { writeLoop(); }, TASK_PRIORITY_MIN + 1, TASK_STACK_DEPTH_DEFAULT, "Log write");
}

void Logger::begin_run(const char* name, bool autonomous) {
    const std::uint32_t now = pros::millis();
    runsMutex.take();
    const std::uint32_t id = currentRun.load() + 1;
    Run& run = runs[id % kMaxRuns];
    run.id = id;
    run.autonomous = autonomous;
    run.start_ms = now;
    file_safe(run.name, sizeof(run.name), name);
    runsMutex.give();
    if (autonomous) {
        autonStartMs = now;
        autonStarted = true;
        g_mark.store(kNoMark);
    }
    currentRun = id; // rows from here on go to the new file
}

void Logger::route(const char* name) {
    routeMutex.take();
    std::snprintf(routeName, sizeof(routeName), "%s", name != nullptr ? name : "");
    for (char* c = routeName; *c != '\0'; ++c) {
        if (*c == '\n' || *c == '\r') {
            *c = ' ';
        }
    }
    routeRun = currentRun.load();
    ++routeSeq;
    routeMutex.give();
}

void Logger::note(const char* key, const char* value) {
    if (key == nullptr || value == nullptr) {
        return;
    }
    // Also on the terminal, so it shows up even without an SD card.
    std::printf("LOG_NOTE,%s,%s\n", key, value);
    notesMutex.take();
    if (noteCount < notes.size()) {
        notes[noteCount].run = currentRun.load();
        std::snprintf(notes[noteCount].key, sizeof(notes[noteCount].key), "%s", key);
        std::snprintf(notes[noteCount].value, sizeof(notes[noteCount].value), "%s", value);
        ++noteCount;
    }
    notesMutex.give();
}

// ---- Sampler ----

void Logger::sampleLoop() {
    std::uint32_t now = pros::millis();
    std::uint32_t tick = 0;
    bool behind = false;
    while (true) {
        const std::size_t fill = (head.load() + capacity - tail.load()) % capacity;
        if (fill > capacity * kSlowFill) {
            behind = true;
        } else if (fill < capacity * kFastFill) {
            behind = false;
        }
        const std::uint32_t every = pros::competition::is_disabled() ? kDisabledEvery : (behind ? 2 : 1);
        const std::uint32_t run = currentRun.load();
        if (run != 0 && tick % every == 0) {
            takeSample(run, pros::millis());
        }
        ++tick;
        pros::Task::delay_until(&now, period);
    }
}

void Logger::takeSample(std::uint32_t run, std::uint32_t now) {
    const std::size_t at = head.load(std::memory_order_relaxed);
    const std::size_t next = (at + 1) % capacity;
    if (next == tail.load(std::memory_order_acquire)) {
        ++dropped; // full: the writer is far behind
        return;
    }
    double* value = &slotValues[at * columns];
    const std::uint64_t mark = g_mark.load();
    const bool fieldControl = pros::competition::is_field_control();
    const bool competitionSwitch = pros::competition::is_competition_switch();

    *value++ = now;
    *value++ = autonStarted ? static_cast<double>(now - autonStartMs.load()) : -1.0;
    *value++ = mark == kNoMark ? -1.0 : static_cast<double>(static_cast<std::int32_t>(mark & 0xFFFFFFFFu));
    *value++ = mark == kNoMark ? -1.0 : static_cast<double>(mark >> 32);
    *value++ = chassis.isInMotion() ? 1.0 : 0.0;
    *value++ = pros::competition::is_connected() ? 1.0 : 0.0;
    *value++ = fieldControl ? 1.0 : 0.0;
    *value++ = competitionSwitch ? 1.0 : 0.0;
    *value++ = pros::competition::is_autonomous() ? 1.0 : 0.0;
    *value++ = pros::competition::is_disabled() ? 1.0 : 0.0;
    *value++ = fieldControl || competitionSwitch ? 1.0 : 0.0;
    *value++ = int_or_nan(pros::battery::get_voltage());
    *value++ = pros::battery::get_capacity();
    *value++ = int_or_nan(pros::battery::get_current());
    *value++ = chassis.poseResets();
    *value++ = chassis.lastResetDx();
    *value++ = chassis.lastResetDy();
    *value++ = chassis.poseResetSkips();
    *value++ = dropped.load();

    for (std::size_t i = 0; i < motorPortCount; ++i) {
        pros::AbstractMotor* motor = motorPorts[i].motor;
        const std::uint8_t index = motorPorts[i].index;
        *value++ = motor->get_position(index);
        *value++ = motor->get_actual_velocity(index);
        *value++ = int_or_nan(motor->get_voltage(index));
        *value++ = motor->get_temperature(index);
        *value++ = int_or_nan(motor->get_current_draw(index));
        *value++ = motor->get_target_position(index);
        *value++ = int_or_nan(motor->get_target_velocity(index));
    }

    if (robotFields) {
        refresh_robot_cache();
    }
    for (std::size_t i = 0; i < fieldCount; ++i) {
        *value++ = fields[i].callback(fields[i].context);
    }

    slotRun[at] = run;
    head.store(next, std::memory_order_release);
}

// ---- Writer ----

std::size_t Logger::formatRow(char* out, const double* values) const {
    std::size_t n = 0;
    for (std::size_t c = 0; c < columns; ++c) {
        if (c > 0) {
            out[n++] = ',';
        }
        n += put_value(out + n, values[c], columnDecimals[c]);
    }
    out[n++] = '\n';
    return n;
}

std::size_t Logger::formatNotes(char* out, std::size_t room, std::uint32_t run) {
    std::size_t n = 0;
    notesMutex.take();
    std::size_t kept = 0;
    for (std::size_t i = 0; i < noteCount; ++i) {
        if (notes[i].run <= run && room - n > kMaxNoteChars) {
            const int length = std::snprintf(out + n, kMaxNoteChars, "# %s,%s\n", notes[i].key, notes[i].value);
            n += length > 0 ? std::min<std::size_t>(static_cast<std::size_t>(length), kMaxNoteChars - 1) : 0;
        } else {
            notes[kept++] = notes[i]; // a later run's (its file isn't open yet), or no room: next time
        }
    }
    noteCount = kept;
    notesMutex.give();
    return n;
}

std::size_t Logger::formatRoute(char* out, std::size_t room, std::uint32_t run, std::uint32_t& routeWritten) {
    std::size_t n = 0;
    routeMutex.take();
    if (routeSeq != routeWritten && routeRun == run && room > kRouteLength + 16) {
        const int length = std::snprintf(out, kRouteLength + 16, "# route,%s\n", routeName);
        n = length > 0 ? std::min<std::size_t>(static_cast<std::size_t>(length), kRouteLength + 15) : 0;
        routeWritten = routeSeq;
    }
    routeMutex.give();
    return n;
}

bool Logger::reserveSession(std::uint32_t& id) {
    char counterPath[64] {};
    const int pathLength = std::snprintf(counterPath, sizeof(counterPath), "%s/telemetry_session.txt", directory);
    if (pathLength < 0 || static_cast<std::size_t>(pathLength) >= sizeof(counterPath)) {
        return false;
    }

    unsigned long previous = 0;
    std::FILE* counter = std::fopen(counterPath, "r");
    if (counter != nullptr) {
        const int result = std::fscanf(counter, "%lu", &previous);
        std::fclose(counter);
        if (result != 1) {
            previous = 0;
        }
    }

    id = static_cast<std::uint32_t>(previous + 1);
    counter = std::fopen(counterPath, "w");
    if (counter == nullptr) {
        return false;
    }
    const bool written = std::fprintf(counter, "%lu\n", static_cast<unsigned long>(id)) >= 0;
    std::fclose(counter);
    return written;
}

bool Logger::writeHeader(std::FILE* file) {
    for (std::size_t i = 0; i < kLeadCount; ++i) {
        if (std::fprintf(file, i == 0 ? "%s" : ",%s", kLeadColumns[i].name) < 0) {
            return false;
        }
    }
    for (std::size_t i = 0; i < motorPortCount; ++i) {
        const int port = motorPorts[i].port < 0 ? -motorPorts[i].port : motorPorts[i].port;
        for (const Column& column : kPortColumns) {
            if (std::fprintf(file, ",port_%d_%s", port, column.name) < 0) {
                return false;
            }
        }
    }
    for (std::size_t i = 0; i < fieldCount; ++i) {
        if (std::fprintf(file, ",%s", fields[i].name) < 0) {
            return false;
        }
    }
    return std::fputc('\n', file) != EOF;
}

// The file for a run: /usd/telemetry_<NNNN>_<name>.csv, "# key,value" metadata (the route too, if route() came
// first), then the header row.
std::FILE* Logger::openRun(std::uint32_t run, std::uint32_t& routeWritten) {
    Run info;
    runsMutex.take();
    info = runs[run % kMaxRuns];
    runsMutex.give();
    if (info.id != run) { // overwritten by newer runs before the writer got here
        info = Run{};
        info.id = run;
        std::snprintf(info.name, sizeof(info.name), "run");
    }

    if (pros::usd::is_installed() == 0) {
        return nullptr;
    }
    fileMutex.take();
    std::uint32_t session = 0;
    std::FILE* file = nullptr;
    char path[96] {};
    if (reserveSession(session)) {
        std::snprintf(path, sizeof(path), "%s/telemetry_%04lu_%s.csv", directory, static_cast<unsigned long>(session),
                      info.name);
        file = std::fopen(path, "w");
    }
    if (file != nullptr) {
        bool ok = std::fprintf(file,
                               "# session_id,%lu\n"
                               "# run,%s\n"
                               "# mode,%s\n"
                               "# run_start_ms,%lu\n"
                               "# period_ms,%lu\n"
                               "# disabled_period_ms,%lu\n",
                               static_cast<unsigned long>(session), info.name,
                               info.autonomous ? "autonomous" : "driver", static_cast<unsigned long>(info.start_ms),
                               static_cast<unsigned long>(period),
                               static_cast<unsigned long>(period * kDisabledEvery)) >= 0;
        char routeLine[kRouteLength + 16];
        const std::size_t routeLength = formatRoute(routeLine, sizeof(routeLine), run, routeWritten);
        ok = ok && (routeLength == 0 || std::fwrite(routeLine, 1, routeLength, file) == routeLength);
        ok = ok && writeHeader(file);
        if (ok) {
            std::fflush(file);
        } else {
            std::fclose(file);
            file = nullptr;
        }
    }
    fileMutex.give();
    if (file != nullptr) {
        std::printf("LOG_FILE,%s\n", path);
    } else {
        lemlib::infoSink()->warn("Telemetry: unable to open a log file for run {}", info.name);
    }
    return file;
}

void Logger::writeLoop() {
    std::unique_ptr<char[]> buffer(new (std::nothrow) char[kBufferBytes]);
    if (!buffer) {
        lemlib::infoSink()->warn("Telemetry: not enough memory for the write buffer");
        return;
    }
    const std::size_t rowRoom = columns * kMaxValueChars + 2;
    const std::size_t extraRoom = kMaxNotes * kMaxNoteChars + kRouteLength + 16;
    std::size_t length = 0;
    std::FILE* file = nullptr;
    std::uint32_t fileRun = 0;
    std::uint32_t routeWritten = 0;
    std::uint32_t lastFlush = pros::millis();

    // Hands the buffer to the SD card; sync also flushes it there (what survives a power-off).
    const auto write = [&](bool sync) {
        if (file != nullptr) {
            fileMutex.take();
            const bool ok = length == 0 || std::fwrite(buffer.get(), 1, length, file) == length;
            if (sync) {
                std::fflush(file); // syncs the file on the card; its result isn't trusted to mean a failed write
            }
            fileMutex.give();
            if (!ok) {
                lemlib::infoSink()->warn("Telemetry: write failed, this run's log stops here");
                fileMutex.take();
                std::fclose(file);
                fileMutex.give();
                file = nullptr;
            }
        }
        length = 0;
    };

    while (true) {
        std::size_t at = tail.load(std::memory_order_relaxed);
        while (at != head.load(std::memory_order_acquire)) {
            const std::uint32_t run = slotRun[at];
            if (run != fileRun) { // a new run: finish the old file, start the new one
                write(true);
                if (file != nullptr) {
                    fileMutex.take();
                    std::fclose(file);
                    fileMutex.give();
                }
                fileRun = run;
                file = openRun(run, routeWritten);
                lastFlush = pros::millis();
            }
            if (file != nullptr) {
                if (kBufferBytes - length < rowRoom + extraRoom) {
                    write(false);
                }
                length += formatRoute(buffer.get() + length, kBufferBytes - length, fileRun, routeWritten);
                length += formatNotes(buffer.get() + length, kBufferBytes - length, fileRun);
                length += formatRow(buffer.get() + length, &slotValues[at * columns]);
            }
            at = (at + 1) % capacity;
            tail.store(at, std::memory_order_release);
            if (length >= kWriteBytes) {
                write(false);
            }
        }
        const std::uint32_t now = pros::millis();
        if (now - lastFlush >= kFlushMs) {
            write(true);
            lastFlush = now;
        }
        pros::delay(kWriterPollMs);
    }
}

void Logger::dump() {
    if (pros::usd::is_installed() == 0) {
        std::printf("LOG_DUMP_ERROR,sd_not_installed\n");
        return;
    }

    char fileList[4096] {};
    if (pros::usd::list_files("/", fileList, sizeof(fileList)) < 0) {
        std::printf("LOG_DUMP_ERROR,list_files_failed\n");
        return;
    }

    fileMutex.take();
    char* fileName = fileList;
    while (*fileName != '\0') {
        char* lineEnd = std::strchr(fileName, '\n');
        if (lineEnd != nullptr) {
            *lineEnd = '\0';
        }

        if (std::strncmp(fileName, "telemetry_", 10) == 0 && std::strstr(fileName, ".csv") != nullptr) {
            char path[96] {};
            std::snprintf(path, sizeof(path), "/usd/%s", fileName);
            std::FILE* file = std::fopen(path, "r");
            if (file != nullptr) {
                std::printf("LOG_DUMP_BEGIN,%s\n", fileName);
                char line[256];
                while (std::fgets(line, sizeof(line), file) != nullptr) {
                    std::printf("%s", line);
                    pros::delay(2);
                }
                std::fclose(file);
                std::printf("LOG_DUMP_END,%s\n", fileName);
            }
        }

        if (lineEnd == nullptr) {
            break;
        }
        fileName = lineEnd + 1;
    }
    fileMutex.give();
    std::printf("LOG_DUMP_COMPLETE\n");
}

}
