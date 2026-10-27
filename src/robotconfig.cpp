#include "robotconfig.hpp"
#include "config/ports.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Imu imu(ports::kImu);
crimson::Crimson crimson_cam(ports::kVision);
display::Dashboard dashboard;

pros::MotorGroup left_motors({ports::kLeftDriveA, ports::kLeftDriveB}, pros::MotorGears::blue);
pros::MotorGroup right_motors({ports::kRightDriveA, ports::kRightDriveB}, pros::MotorGears::blue);
telemetry::Logger logger;
pros::Rotation horizontal_odom(ports::kHorizontalOdom);

namespace {

constexpr float kTrackWidthIn = 12.5f;
constexpr float kDriveRpm = 360.0f;
constexpr float kHorizontalDrift = 2.0f;

constexpr float kTrackingWheelDiameterIn = 2.0f;
constexpr float kHorizontalWheelOffsetIn = -0.7f; // FIX OFFSET: MOST ISSUES ARE PROBABLY FROM NOT FIXING OFFSET

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, kTrackWidthIn, lemlib::Omniwheel::NEW_325, kDriveRpm,
                              kHorizontalDrift);

lemlib::ControllerSettings linear_controller(9.5, 0, 12, 1, 0.2, 100, 1, 500, 40);
lemlib::ControllerSettings angular_controller(5.5, 0, 40, 3, 1, 50, 3, 300, 0);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_odom, kTrackingWheelDiameterIn, kHorizontalWheelOffsetIn);

lemlib::OdomSensors sensors(nullptr, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

}

crimson::Chassis chassis(drivetrain, linear_controller, angular_controller, sensors);
