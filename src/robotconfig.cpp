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
pros::Motor claw_motor(ports::kClaw, pros::MotorGears::blue);
pros::Motor pivot_motor(ports::kPivot, pros::MotorGears::blue);

namespace {

constexpr float kTrackWidthIn = 12.5f;
constexpr float kDriveRpm = 360.0f;
constexpr float kHorizontalDrift = 2.0f;

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, kTrackWidthIn, lemlib::Omniwheel::NEW_325, kDriveRpm,
                              kHorizontalDrift);

lemlib::ControllerSettings linear_controller(10, 0, 12, 1, 0.2, 100, 1, 500, 40);
lemlib::ControllerSettings angular_controller(5.5, 0, 40, 3, 1, 50, 3, 300, 0);

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

lemlib::ExpoDriveCurve throttle_curve(2, 2.5, 1.019);
lemlib::ExpoDriveCurve steer_curve(2, 2.5, 1.0);

}

lemlib::Chassis chassis(drivetrain, linear_controller, angular_controller, sensors, &throttle_curve, &steer_curve);
