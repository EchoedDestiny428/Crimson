#include "robotconfig.hpp"
#include "telemetry/channels.hpp"
#include "telemetry/keys.hpp"
#include "telemetry/samplers.hpp"

namespace telemetry::drive_log {

namespace {

structs::Pose to_struct(const lemlib::Pose& pose) {
    return {pose.x, pose.y, pose.theta};
}

structs::Wheel read_wheel(const pros::Rotation& sensor) {
    return {sensor.get_position(), sensor.get_velocity(), sensor.get_angle()};
}

structs::Imu read_imu() {
    const pros::imu_gyro_s_t gyro = imu.get_gyro_rate();
    const pros::imu_accel_s_t accel = imu.get_accel();
    return {
        to_double(imu.get_heading()),
        to_double(imu.get_rotation()),
        to_float(imu.get_pitch()),
        to_float(imu.get_roll()),
        to_float(imu.get_yaw()),
        to_float(gyro.x),
        to_float(gyro.y),
        to_float(gyro.z),
        to_float(accel.x),
        to_float(accel.y),
        to_float(accel.z),
        static_cast<std::uint8_t>(imu.get_status()),
    };
}

const std::array<MotorChannel, 4>& motors() {
    static const std::array<MotorChannel, 4> channels{
        MotorChannel(left_motors, 0, keys::kMotorLeftA, keys::kMotorLeftAHealth),
        MotorChannel(left_motors, 1, keys::kMotorLeftB, keys::kMotorLeftBHealth),
        MotorChannel(right_motors, 0, keys::kMotorRightA, keys::kMotorRightAHealth),
        MotorChannel(right_motors, 1, keys::kMotorRightB, keys::kMotorRightBHealth),
    };
    return channels;
}

}

void fast(std::uint64_t timestamp) {
    static const std::uint32_t robot = logger.entry(keys::kOdometryRobot, structs::Pose::kType);
    static const std::uint32_t raw = logger.entry(keys::kOdometryRaw, structs::Pose::kType);
    static const std::uint32_t vertical = logger.entry(keys::kOdometryVerticalWheel, structs::Wheel::kType);
    static const std::uint32_t horizontal = logger.entry(keys::kOdometryHorizontalWheel, structs::Wheel::kType);
    static const std::uint32_t inertial = logger.entry(keys::kOdometryImu, structs::Imu::kType);
    static BooleanChannel active(keys::kMotionActive);

    logger.record_struct(robot, to_struct(chassis.getPose()).pack(), timestamp);
    logger.record_struct(raw, to_struct(chassis.getRawPose()).pack(), timestamp);
    logger.record_struct(vertical, read_wheel(vertical_odom).pack(), timestamp);
    logger.record_struct(horizontal, read_wheel(horizontal_odom).pack(), timestamp);
    logger.record_struct(inertial, read_imu().pack(), timestamp);
    for (const MotorChannel& motor : motors()) {
        motor.record(timestamp);
    }
    active.record(chassis.isInMotion(), timestamp);
}

void slow(std::uint64_t timestamp) {
    for (const MotorChannel& motor : motors()) {
        motor.record_health(timestamp);
    }
}

}
