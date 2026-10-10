#include "main.h"
#include "autons/autons.hpp"
#include "display/controller_hud.hpp"
#include "macros.hpp"
#include "robotconfig.hpp"
#include "subsystems/claw.hpp"
#include "subsystems/drive.hpp"
#include "subsystems/elevator.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/pivot.hpp"
#include "subsystems/vision.hpp"
#include "util/driver_task.hpp"
#include "util/system_check.hpp"
#include "util/motor_utils.hpp"
#include <cmath>

using namespace subsystems;

namespace {

double chassis_x(const void* context) {
    return static_cast<crimson::Chassis*>(const_cast<void*>(context))->getPose().x;
}

double chassis_y(const void* context) {
    return static_cast<crimson::Chassis*>(const_cast<void*>(context))->getPose().y;
}

double chassis_theta(const void* context) {
    return static_cast<crimson::Chassis*>(const_cast<void*>(context))->getPose().theta;
}

double rotation_position(const void* context) {
    return static_cast<const pros::Rotation*>(context)->get_position();
}

double rotation_velocity(const void* context) {
    return static_cast<const pros::Rotation*>(context)->get_velocity();
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

double vision_has_target(const void* context) {
    return static_cast<const crimson::Crimson*>(context)->has_target() ? 1.0 : 0.0;
}

double vision_tag_count(const void* context) {
    return static_cast<const crimson::Crimson*>(context)->tag_count();
}

double vision_primary_tag_id(const void* context) {
    return static_cast<const crimson::Crimson*>(context)->primary_tag_id();
}

double vision_tx(const void* context) {
    return static_cast<const crimson::Crimson*>(context)->get_tx();
}

double vision_ty(const void* context) {
    return static_cast<const crimson::Crimson*>(context)->get_ty();
}

double vision_data_age_ms(const void* context) {
    return static_cast<const crimson::Crimson*>(context)->get_data_age_ms();
}

double vision_pose_x(const void* context) {
    const auto pose = static_cast<const crimson::Crimson*>(context)->estimate_global_pose(imu.get_heading());
    return pose ? pose->x : NAN;
}

double vision_pose_y(const void* context) {
    const auto pose = static_cast<const crimson::Crimson*>(context)->estimate_global_pose(imu.get_heading());
    return pose ? pose->y : NAN;
}

double vision_pose_theta(const void* context) {
    const auto pose = static_cast<const crimson::Crimson*>(context)->estimate_global_pose(imu.get_heading());
    return pose ? pose->theta : NAN;
}

void register_telemetry() {
    logger.register_field("pose_x_in", chassis_x, &chassis);
    logger.register_field("pose_y_in", chassis_y, &chassis);
    logger.register_field("pose_theta_deg", chassis_theta, &chassis);
    logger.register_field("odom_vertical_position_cdeg", rotation_position, &vertical_odom);
    logger.register_field("odom_vertical_velocity_cdeg_s", rotation_velocity, &vertical_odom);
    logger.register_field("odom_horizontal_position_cdeg", rotation_position, &horizontal_odom);
    logger.register_field("odom_horizontal_velocity_cdeg_s", rotation_velocity, &horizontal_odom);
    logger.register_field("imu_heading_deg", imu_heading, &imu);
    logger.register_field("imu_rotation_deg", imu_rotation, &imu);
    logger.register_field("imu_pitch_deg", imu_pitch, &imu);
    logger.register_field("imu_roll_deg", imu_roll, &imu);
    logger.register_field("imu_gyro_x_dps", imu_gyro_x, &imu);
    logger.register_field("imu_gyro_y_dps", imu_gyro_y, &imu);
    logger.register_field("imu_gyro_z_dps", imu_gyro_z, &imu);
    logger.register_field("imu_accel_x_g", imu_accel_x, &imu);
    logger.register_field("imu_accel_y_g", imu_accel_y, &imu);
    logger.register_field("imu_accel_z_g", imu_accel_z, &imu);
    logger.register_field("imu_status", imu_status, &imu);
    logger.register_field("vision_has_target", vision_has_target, &crimson_cam);
    logger.register_field("vision_tag_count", vision_tag_count, &crimson_cam);
    logger.register_field("vision_primary_tag_id", vision_primary_tag_id, &crimson_cam);
    logger.register_field("vision_tx_deg", vision_tx, &crimson_cam);
    logger.register_field("vision_ty_deg", vision_ty, &crimson_cam);
    logger.register_field("vision_data_age_ms", vision_data_age_ms, &crimson_cam);
    logger.register_field("vision_pose_x_in", vision_pose_x, &crimson_cam);
    logger.register_field("vision_pose_y_in", vision_pose_y, &crimson_cam);
    logger.register_field("vision_pose_theta_deg", vision_pose_theta, &crimson_cam);
}

}

void initialize() {
    controller.clear();
    vision::init();
    dashboard.show_status("CALIBRATING");

    drive::init();
    claw::init();
    pivot::init();
    elevator::init();
    logger.attach(&left_motors);
    logger.attach(&right_motors);
    logger.attach(&elevator_motors);
    logger.attach(&intake_motor);
    logger.attach(&claw_motor);
    logger.attach(&pivot_motor);
    register_telemetry();
    logger.start();
    pros::Task startup_home(macros::home, "Startup home");

    vision::start();
}

void disabled() {
    elevator::hold();
    autons::select();
}

void competition_initialize() {}

void autonomous() {
    autons::run_selected();
}

void opcontrol() {
    macros::cancel();
    chassis.cancelAllMotions();
    claw::automatic();
    util::start_driver_task("System check", util::system_check::update, util::system_check::stop);
    util::start_driver_task("Intake", intake::update, intake::stop);
    util::start_driver_task("Claw", claw::update, claw::automatic);
    util::start_driver_task("Pivot", pivot::update, pivot::stop);
    util::start_driver_task("Macros", macros::update, macros::stop);
    util::start_driver_task("HUD", display::controller_hud::update);
    
    drive::reset();
    std::uint32_t now = pros::millis();
    while (true) {
        drive::update();
        // util::two_button(elevator_motors, controls::kElevatorUp, controls::kElevatorDown);
        pros::Task::delay_until(&now, util::kLoopMs);
    }
}
