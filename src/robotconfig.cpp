#include "robotconfig.hpp"
#include "config/ports.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Imu imu(ports::kImu);
crimson::Crimson crimson_cam(ports::kVision);
display::Dashboard dashboard;

pros::MotorGroup left_motors({ports::kLeftDriveA, ports::kLeftDriveB}, pros::MotorGears::blue);
pros::MotorGroup right_motors({ports::kRightDriveA, ports::kRightDriveB}, pros::MotorGears::blue);
pros::MotorGroup elevator_motors({ports::kElevatorA, ports::kElevatorB}, pros::MotorGears::blue);
pros::Motor intake_motor(ports::kIntake, pros::MotorGears::blue);
pros::Motor claw_motor(ports::kClaw, pros::MotorGears::green);
pros::Motor pivot_motor(ports::kPivot, pros::MotorGears::green);

pros::adi::Pneumatics intake_lift(ports::kIntakeLift, false);

namespace {

constexpr float kTrackWidthIn = 12.5f;
constexpr float kDriveRpm = 360.0f;
constexpr float kHorizontalDrift = 2.0f;

constexpr float kTrackingWheelDiameterIn = 2.0f;
constexpr float kVerticalWheelOffsetIn = 0.5f;
constexpr float kHorizontalWheelOffsetIn = -0.7f;

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, kTrackWidthIn, lemlib::Omniwheel::NEW_325, kDriveRpm,
                              kHorizontalDrift);

lemlib::ControllerSettings linear_controller(10, 0, 12, 1, 0.2, 100, 1, 500, 40);
lemlib::ControllerSettings angular_controller(5.5, 0, 40, 3, 1, 50, 3, 300, 0);

pros::Rotation vertical_odom(ports::kVerticalOdom);
pros::Rotation horizontal_odom(ports::kHorizontalOdom);

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_odom, kTrackingWheelDiameterIn, kVerticalWheelOffsetIn);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_odom, kTrackingWheelDiameterIn, kHorizontalWheelOffsetIn);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

lemlib::ExpoDriveCurve throttle_curve(2, 2.5, 1.019);
lemlib::ExpoDriveCurve steer_curve(2, 2.5, 1.0);

}

lemlib::Chassis chassis(drivetrain, linear_controller, angular_controller, sensors, &throttle_curve, &steer_curve);
