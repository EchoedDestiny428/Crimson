#include "robotconfig.hpp"
#include "config/ports.hpp"
#include <utility>

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Imu imu(ports::kImu);
display::Dashboard dashboard;

// Encoder units are set explicitly: left unset, a motor keeps whatever units the last
// program gave it (rotations or counts), and every angle and height would be off.
pros::MotorGroup left_motors({ports::kLeftDriveA, ports::kLeftDriveB}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::MotorGroup right_motors({ports::kRightDriveA, ports::kRightDriveB}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::MotorGroup extension_motors({ports::kExtensionA, ports::kExtensionB}, pros::MotorGears::blue,
                                  pros::MotorUnits::degrees);
pros::MotorGroup arm_pivot_motor({ports::kArmPivot}, pros::MotorGears::red,
                                 pros::MotorUnits::degrees); // a group so SmartMotor can drive it
// Side toggle: two 5.5W motors (fixed 200 rpm, no cartridge to set), driven open loop together,
// so they are left out of apply_motor_config() and keep their own current limit.
pros::MotorGroup side_toggle_motors({ports::kSideToggleLeft, ports::kSideToggleRight}, pros::MotorGears::green);
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

// (kP, kI, kD, windup, smallError, smallErrorTimeout ms, largeError, largeErrorTimeout ms, slew). The large-error
// timeouts were 500 / 300 ms: stops that stall just short (inside 1 in / 3 deg but never reaching 0.2 in / 1 deg) sat
// still that long (telemetry_0014: about 1 s per loop). 200 / 150 ms ends them sooner at the same error.
lemlib::ControllerSettings linear_controller(9.5, 0, 12, 1, 0.2, 100, 1, 200, 40);
lemlib::ControllerSettings angular_controller(5.5, 0, 40, 3, 1, 50, 3, 150, 0);

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_odom, kTrackingWheelDiameterIn, kVerticalWheelOffsetIn);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_odom, kTrackingWheelDiameterIn, kHorizontalWheelOffsetIn);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

}

crimson::Chassis chassis(drivetrain, linear_controller, angular_controller, sensors);

// Cartridge, encoder units and current limit, applied at runtime: the constructors run
// as globals before the devices are up, so the settings they pass can silently not take,
// leaving the default green cartridge (a red arm then reads double its real angle).
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
