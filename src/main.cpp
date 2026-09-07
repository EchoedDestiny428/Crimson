#include "main.h"
#include "display/dashboard.hpp"
#include "lemlib/api.hpp"
#include "vision/crimson.hpp"

namespace
{
    constexpr std::uint8_t kLoopDelayMs = 20;

    // Hardware Configuration
    constexpr std::int8_t kIntakePort = 3; // Change to your intake port (use negative if reversed, e.g. -3)

    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    pros::Motor intake(kIntakePort);
    crimson::Crimson crimson_cam(1);
    pros::Imu imu(2);
    display::Dashboard dashboard;

    // --------------------------------------------------
    // LemLib Drivetrain Configuration
    // --------------------------------------------------

    // Left motor group (adjust ports and signs for your robot)
    pros::MotorGroup left_motors({-20, 13, -11}, pros::MotorGears::blue);
    // Right motor group (adjust ports and signs for your robot)
    pros::MotorGroup right_motors({14, 18, -17}, pros::MotorGears::blue);

    // Drivetrain settings
    lemlib::Drivetrain drivetrain(
        &left_motors,                // left motor group
        &right_motors,               // right motor group
        12.5,                        // 12.5 inch track width
        lemlib::Omniwheel::NEW_325,  // using new 3.25" omnis
        450,                         // drivetrain rpm
        2                            // horizontal drift is 2 (for omni wheels)
    );

    // Lateral motion controller (forward/backward)
    lemlib::ControllerSettings linearController(
        10,   // proportional gain (kP)
        0,    // integral gain (kI)
        12,   // derivative gain (kD)
        1,    // anti windup
        0.2,  // small error range, in inches
        100,  // small error range timeout, in milliseconds
        1,    // large error range, in inches
        500,  // large error range timeout, in milliseconds
        40    // maximum acceleration (slew)
    );

    // Angular motion controller (turning)
    lemlib::ControllerSettings angularController(
        5.5,  // proportional gain (kP)
        0,    // integral gain (kI)
        40,   // derivative gain (kD)
        3,    // anti windup
        1,    // small error range, in degrees
        50,   // small error range timeout, in milliseconds
        3,    // large error range, in degrees
        300,  // large error range timeout, in milliseconds
        0     // maximum acceleration (slew)
    );

    // Sensors for odometry
    lemlib::OdomSensors sensors(
        nullptr, // vertical tracking wheel 1
        nullptr, // vertical tracking wheel 2
        nullptr, // horizontal tracking wheel 1
        nullptr, // horizontal tracking wheel 2
        &imu     // inertial sensor
    );

    // Throttle input curve for driver control
    lemlib::ExpoDriveCurve throttleCurve(
        2,    // joystick deadband out of 127
        2.5,  // minimum output where drivetrain will move
        1.019 // expo curve gain
    );

    // Steer input curve for driver control
    lemlib::ExpoDriveCurve steerCurve(
        2,    // joystick deadband out of 127
        2.5,  // minimum output where drivetrain will move
        1.0   // expo curve gain
    );

    // LemLib Chassis
    lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

    void update_dashboard()
    {
        double heading = imu.get_heading();
        auto pose = crimson_cam.estimate_global_pose(heading);

        int tag_id = -1;
        if (crimson_cam.has_target())
        {
            tag_id = crimson_cam.get_raw_target().id;
        }

        dashboard.update(pose, crimson_cam.get_tv(), tag_id);
    }
} // namespace

void initialize()
{
    crimson_cam.initialize();
    crimson_cam.set_camera_mount_metrics(100.0, 0.0);
    dashboard.initialize();

    pros::screen::set_pen(0xFFFFFF);
    pros::screen::print(pros::E_TEXT_MEDIUM, 250, 100, "Calibrating chassis...");

    // Calibrate chassis (also calibrates the IMU connected in sensors)
    chassis.calibrate();
}

void disabled() {}

void competition_initialize() {}

void autonomous()
{
    // Example LemLib autonomous routine:
    // chassis.setPose(0, 0, 0);
    // chassis.moveToPoint(0, 24, 2000);
}

void opcontrol()
{
    while (true)
    {
        // LemLib Arcade Drive (split arcade: left stick Y for throttle, right stick X for turn)
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(throttle, turn);

        crimson_cam.update();
        update_dashboard();

        // Intake controls (Hold R1 = in, Hold R2 = out)
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            intake.move(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            intake.move(-127);
        }
        else
        {
            intake.move(0);
        }

        pros::delay(kLoopDelayMs);
    }
}