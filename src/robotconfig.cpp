#include "robotconfig.hpp"
#include "config/ports.hpp"
#include <utility>

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Imu imu(ports::kImu);
display::Dashboard dashboard;

pros::MotorGroup left_motors({ports::kLeftDriveA, ports::kLeftDriveB}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::MotorGroup right_motors({ports::kRightDriveA, ports::kRightDriveB}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::MotorGroup extension_motors({ports::kExtensionA, ports::kExtensionB}, pros::MotorGears::blue,
                                  pros::MotorUnits::degrees);
pros::MotorGroup arm_pivot_motor({ports::kArmPivot}, pros::MotorGears::red,
                                 pros::MotorUnits::degrees);
telemetry::Logger logger;
pros::Rotation vertical_odom(ports::kVerticalOdom);
pros::Rotation horizontal_odom(ports::kHorizontalOdom);

pros::Distance claw_distance(ports::kClawDistance);
pros::adi::Pneumatics claw_piston(ports::kClawPiston, false);

namespace {

constexpr std::int32_t kMotorCurrentLimitMa = 2500;

constexpr float kTrackWidthIn = 28.5f / 2.54f;
constexpr float kDriveRpm = 360.0f;
constexpr float kHorizontalDrift = 2.0f;

constexpr float kTrackingWheelDiameterIn = 2.0f;
constexpr float kVerticalWheelOffsetIn = 0.0f;
constexpr float kHorizontalWheelOffsetIn = 1.4f;

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, kTrackWidthIn, lemlib::Omniwheel::NEW_325, kDriveRpm,
                              kHorizontalDrift);

lemlib::ControllerSettings linear_controller(9.5, 0, 12, 1, 0.2, 100, 1, 500, 40);
lemlib::ControllerSettings angular_controller(5.5, 0, 40, 3, 1, 50, 3, 300, 0);

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_odom, kTrackingWheelDiameterIn, kVerticalWheelOffsetIn);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_odom, kTrackingWheelDiameterIn, kHorizontalWheelOffsetIn);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

}

crimson::Chassis chassis(drivetrain, linear_controller, angular_controller, sensors);

void apply_motor_config() {
    const std::pair<pros::MotorGroup*, pros::MotorGears> groups[] = {
        {&left_motors, pros::MotorGears::blue},
        {&right_motors, pros::MotorGears::blue},
        {&extension_motors, pros::MotorGears::blue},
        {&arm_pivot_motor, pros::MotorGears::red},
    };
    for (const auto& [group, gears] : groups) {
        group->set_gearing_all(gears);
        group->set_encoder_units_all(pros::MotorUnits::degrees);
        group->set_current_limit_all(kMotorCurrentLimitMa);
    }
}
