#include "robotconfig.hpp"
#include "config/ports.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Imu imu(ports::kImu);
crimson::Crimson crimson_cam(ports::kVision);
display::Dashboard dashboard;

pros::MotorGroup left_motors({ports::kLeftDriveA, ports::kLeftDriveB}, pros::MotorGears::blue);
pros::MotorGroup right_motors({ports::kRightDriveA, ports::kRightDriveB}, pros::MotorGears::blue);
pros::MotorGroup extension_motors({ports::kExtensionA, ports::kExtensionB}, pros::MotorGears::blue);
pros::Motor arm_pivot_motor(ports::kArmPivot, pros::MotorGears::red);
pros::Motor claw_pivot_motor(ports::kClawPivot, pros::MotorGears::green);
pros::Motor side_toggle_motor(ports::kSideToggle, pros::MotorGears::green); // flex wheels; change if the cartridge is blue
telemetry::Logger logger;
pros::Rotation vertical_odom(ports::kVerticalOdom);
pros::Rotation horizontal_odom(ports::kHorizontalOdom);

pros::Distance claw_distance(ports::kClawDistance);
pros::adi::Pneumatics claw_piston(ports::kClawPiston, false);

namespace {

constexpr std::int32_t kMotorCurrentLimitMa = 2500; // V5 maximum; set explicitly so every motor has it

constexpr float kTrackWidthIn = 28.5f / 2.54f; // measured on the new bot (newbot-localization)
constexpr float kDriveRpm = 360.0f;
constexpr float kHorizontalDrift = 2.0f;

constexpr float kTrackingWheelDiameterIn = 2.0f;
constexpr float kVerticalWheelOffsetIn = 0.0f;   // measured (newbot-localization)
constexpr float kHorizontalWheelOffsetIn = 1.4f; // measured (newbot-localization)

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, kTrackWidthIn, lemlib::Omniwheel::NEW_325, kDriveRpm,
                              kHorizontalDrift);

lemlib::ControllerSettings linear_controller(9.5, 0, 12, 1, 0.2, 100, 1, 500, 40);
lemlib::ControllerSettings angular_controller(5.5, 0, 40, 3, 1, 50, 3, 300, 0);

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_odom, kTrackingWheelDiameterIn, kVerticalWheelOffsetIn);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_odom, kTrackingWheelDiameterIn, kHorizontalWheelOffsetIn);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

}

crimson::Chassis chassis(drivetrain, linear_controller, angular_controller, sensors);

void apply_motor_current_limits() {
    for (pros::MotorGroup* group : {&left_motors, &right_motors, &extension_motors}) {
        group->set_current_limit_all(kMotorCurrentLimitMa);
    }
    for (pros::Motor* motor : {&arm_pivot_motor, &claw_pivot_motor, &side_toggle_motor}) {
        motor->set_current_limit(kMotorCurrentLimitMa);
    }
}
