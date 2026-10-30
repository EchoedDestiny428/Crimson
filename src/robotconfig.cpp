#include "robotconfig.hpp"
#include "config/drivetrain.hpp"
#include "config/ports.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Imu imu(ports::kImu);
display::Dashboard dashboard;

pros::MotorGroup left_motors({ports::kLeftDriveA, ports::kLeftDriveB}, pros::MotorGears::blue);
pros::MotorGroup right_motors({ports::kRightDriveA, ports::kRightDriveB}, pros::MotorGears::blue);
pros::Rotation vertical_odom(ports::kVerticalOdom);
pros::Rotation horizontal_odom(ports::kHorizontalOdom);

namespace {

lemlib::ControllerSettings controller_settings(const config::PidConstants& pid) {
    return {pid.kP,         pid.kI,
            pid.kD,         pid.windupRange,
            pid.smallError, pid.smallErrorTimeout,
            pid.largeError, pid.largeErrorTimeout,
            pid.slew};
}

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, config::kTrackWidthIn, config::kWheelDiameterIn,
                              config::kDriveRpm, config::kHorizontalDrift);

lemlib::ControllerSettings linear_controller = controller_settings(config::kLinearPid);
lemlib::ControllerSettings angular_controller = controller_settings(config::kAngularPid);

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_odom, config::kTrackingWheelDiameterIn,
                                              config::kVerticalWheelOffsetIn);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_odom, config::kTrackingWheelDiameterIn,
                                                config::kHorizontalWheelOffsetIn);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

lemlib::ExpoDriveCurve throttle_curve(config::kThrottleCurve.deadband, config::kThrottleCurve.minOutput,
                                      config::kThrottleCurve.curve);
lemlib::ExpoDriveCurve steer_curve(config::kSteerCurve.deadband, config::kSteerCurve.minOutput,
                                   config::kSteerCurve.curve);

}

crimson::Chassis chassis(drivetrain, linear_controller, angular_controller, sensors, &throttle_curve, &steer_curve);
