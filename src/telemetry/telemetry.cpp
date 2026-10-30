#include "telemetry/telemetry.hpp"
#include "config/drivetrain.hpp"
#include "config/ports.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "telemetry/keys.hpp"
#include "telemetry/logger.hpp"
#include "telemetry/samplers.hpp"
#include <atomic>
#include <cstdio>
#include <string_view>

extern "C" const char* const _PROS_COMPILE_TIMESTAMP;

namespace telemetry {

namespace {

constexpr std::uint32_t kPeriodMs = 20;
constexpr std::uint32_t kSlowEveryTicks = 10;
constexpr std::uint32_t kFlushEveryTicks = 5;
constexpr std::size_t kSourceLength = 160;
constexpr std::size_t kMetadataLength = 384;
constexpr std::string_view kSourceRoot = "src/";
constexpr std::string_view kUsdPrefix = "/usd/";

struct EventIds {
    std::uint32_t reset_from;
    std::uint32_t reset_to;
    std::uint32_t reset_source;
    std::uint32_t motion_command;
    std::uint32_t motion_source;
    std::uint32_t motion_id;
    std::uint32_t auton_selected;
    std::uint32_t auton_running;
};

EventIds events{};
bool started = false;
std::atomic<std::uint32_t> overruns{0};
std::atomic<std::uint32_t> motion_count{0};
std::atomic<bool> auton_running{false};
std::atomic<std::uint8_t> auton_status{0};

struct Source {
    char text[kSourceLength];
};

Source format_source(const std::source_location& where) {
    std::string_view file = where.file_name();
    for (std::size_t at = file.find(kSourceRoot); at != std::string_view::npos; at = file.find(kSourceRoot, at + 1)) {
        if (at == 0 || file[at - 1] == '/' || file[at - 1] == '\\') {
            file.remove_prefix(at);
            break;
        }
    }
    Source source{};
    std::snprintf(source.text, sizeof(source.text), "%.*s:%lu", static_cast<int>(file.size()), file.data(),
                  static_cast<unsigned long>(where.line()));
    for (char& c : source.text) {
        if (c == '\\') {
            c = '/';
        }
    }
    return source;
}

template <typename Struct>
void write_schema(std::uint64_t timestamp) {
    logger.record_string(logger.entry(Struct::kSchemaKey, types::kStructSchema), Struct::kSchema, timestamp);
}

void write_metadata(const char* key, const char* value, std::uint64_t timestamp) {
    logger.record_string(logger.entry(key, types::kString), value, timestamp);
}

void write_pid(const char* key, const config::PidConstants& pid, std::uint64_t timestamp) {
    char text[kMetadataLength];
    std::snprintf(text, sizeof(text),
                  "kP=%g,kI=%g,kD=%g,windupRange=%g,smallError=%g,smallErrorTimeout=%g,largeError=%g,"
                  "largeErrorTimeout=%g,slew=%g",
                  pid.kP, pid.kI, pid.kD, pid.windupRange, pid.smallError, pid.smallErrorTimeout, pid.largeError,
                  pid.largeErrorTimeout, pid.slew);
    write_metadata(key, text, timestamp);
}

void write_metadata(std::uint64_t timestamp) {
    char build[64];
    std::snprintf(build, sizeof(build), "%s", _PROS_COMPILE_TIMESTAMP);
    for (char& c : build) {
        if (c == ',' || c == '=') {
            c = ' ';
        }
    }
    std::string_view file = logger.path();
    if (file.starts_with(kUsdPrefix)) {
        file.remove_prefix(kUsdPrefix.size());
    }
    char text[kMetadataLength];
    std::snprintf(text, sizeof(text), "timestamp=%s,file=%.*s", build, static_cast<int>(file.size()), file.data());
    write_metadata(keys::kMetadataBuild, text, timestamp);

    std::snprintf(text, sizeof(text),
                  "imu=%d,leftDriveA=%d,leftDriveB=%d,rightDriveA=%d,rightDriveB=%d,verticalOdom=%d,horizontalOdom=%d",
                  ports::kImu, ports::kLeftDriveA, ports::kLeftDriveB, ports::kRightDriveA, ports::kRightDriveB,
                  ports::kVerticalOdom, ports::kHorizontalOdom);
    write_metadata(keys::kMetadataPorts, text, timestamp);

    std::snprintf(text, sizeof(text), "trackWidth=%g,wheelDiameter=%g,rpm=%g,horizontalDrift=%g", config::kTrackWidthIn,
                  config::kWheelDiameterIn, config::kDriveRpm, config::kHorizontalDrift);
    write_metadata(keys::kMetadataDrivetrain, text, timestamp);

    std::snprintf(text, sizeof(text), "trackingWheelDiameter=%g,verticalOffset=%g,horizontalOffset=%g",
                  config::kTrackingWheelDiameterIn, config::kVerticalWheelOffsetIn, config::kHorizontalWheelOffsetIn);
    write_metadata(keys::kMetadataOdometry, text, timestamp);

    write_pid(keys::kMetadataLinearPid, config::kLinearPid, timestamp);
    write_pid(keys::kMetadataAngularPid, config::kAngularPid, timestamp);

    std::snprintf(text, sizeof(text),
                  "throttleDeadband=%g,throttleMinOutput=%g,throttleCurve=%g,steerDeadband=%g,steerMinOutput=%g,"
                  "steerCurve=%g",
                  config::kThrottleCurve.deadband, config::kThrottleCurve.minOutput, config::kThrottleCurve.curve,
                  config::kSteerCurve.deadband, config::kSteerCurve.minOutput, config::kSteerCurve.curve);
    write_metadata(keys::kMetadataDriveCurves, text, timestamp);

    write_metadata(keys::kMetadataUnits,
                   "distance=in,heading=deg,headingZero=+Y,headingPositive=clockwise,timestamp=us,"
                   "motorPosition=deg,motorVelocity=rpm,motorVoltage=V,motorCurrent=A,motorPower=W,motorTorque=Nm,"
                   "motorTemperature=C,motorEfficiency=pct,wheel=centideg,wheelVelocity=centideg/s,imuAngle=deg,"
                   "imuGyro=deg/s,imuAccel=g,batteryVoltage=V,batteryCurrent=A,batteryCapacity=pct,"
                   "batteryTemperature=C",
                   timestamp);
}

void check_auton_aborted() {
    if (auton_running && pros::competition::get_status() != auton_status) {
        auton_finished();
    }
}

void run() {
    std::uint32_t now = pros::millis();
    std::uint32_t tick = 0;
    while (true) {
        const std::uint64_t timestamp = pros::micros();
        const bool slow = tick % kSlowEveryTicks == 0;
        const bool enabled = !pros::competition::is_disabled();

        check_auton_aborted();
        system_log::every_tick(timestamp);
        if (enabled || slow) {
            drive_log::fast(timestamp);
            system_log::fast(timestamp);
        }
        if (slow) {
            drive_log::slow(timestamp);
            system_log::slow(timestamp);
        }
        if (tick % kFlushEveryTicks == 0) {
            logger.flush();
        }
        ++tick;

        if (pros::millis() - now >= kPeriodMs) {
            ++overruns;
            now = pros::millis();
        }
        pros::Task::delay_until(&now, kPeriodMs);
    }
}

}

std::uint32_t loop_overruns() {
    return overruns;
}

void start() {
    if (started) {
        return;
    }
    started = true;

    char header[96];
    std::snprintf(header, sizeof(header), "Crimson %s", _PROS_COMPILE_TIMESTAMP);
    if (!logger.open(header)) {
        return;
    }

    const std::uint64_t timestamp = pros::micros();
    write_schema<structs::Pose>(timestamp);
    write_schema<structs::Motor>(timestamp);
    write_schema<structs::MotorHealth>(timestamp);
    write_schema<structs::Imu>(timestamp);
    write_schema<structs::Wheel>(timestamp);
    write_schema<structs::Battery>(timestamp);
    write_schema<structs::Controller>(timestamp);
    write_schema<structs::Motion>(timestamp);
    write_metadata(timestamp);

    events = {
        logger.entry(keys::kOdometryResetFrom, structs::Pose::kType),
        logger.entry(keys::kOdometryResetTo, structs::Pose::kType),
        logger.entry(keys::kOdometryResetSource, types::kString),
        logger.entry(keys::kMotionCommand, structs::Motion::kType),
        logger.entry(keys::kMotionSource, types::kString),
        logger.entry(keys::kMotionId, types::kInt64),
        logger.entry(keys::kAutonSelected, types::kString),
        logger.entry(keys::kAutonRunning, types::kBoolean),
    };
    logger.flush();

    pros::Task task(run, "Telemetry");
}

void pose_reset(const lemlib::Pose& from, const lemlib::Pose& to, const std::source_location& where) {
    if (!logger.is_open()) {
        return;
    }
    const std::uint64_t timestamp = pros::micros();
    const Source source = format_source(where);
    logger.record_struct(events.reset_from, structs::Pose{from.x, from.y, from.theta}.pack(), timestamp);
    logger.record_struct(events.reset_to, structs::Pose{to.x, to.y, to.theta}.pack(), timestamp);
    logger.record_string(events.reset_source, source.text, timestamp);
}

void motion_started(const structs::Motion& motion, const std::source_location& where) {
    if (!logger.is_open()) {
        return;
    }
    const std::uint64_t timestamp = pros::micros();
    const Source source = format_source(where);
    logger.record_int64(events.motion_id, ++motion_count, timestamp);
    logger.record_string(events.motion_source, source.text, timestamp);
    logger.record_struct(events.motion_command, motion.pack(), timestamp);
}

void auton_started(const char* name) {
    const std::uint64_t timestamp = pros::micros();
    auton_status = pros::competition::get_status();
    auton_running = true;
    logger.record_string(events.auton_selected, name != nullptr ? name : "", timestamp);
    logger.record_boolean(events.auton_running, true, timestamp);
}

void auton_finished() {
    if (auton_running.exchange(false)) {
        logger.record_boolean(events.auton_running, false, pros::micros());
    }
}

}
