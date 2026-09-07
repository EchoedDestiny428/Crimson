#include "main.h"
#include "display/dashboard.hpp"
#include "vision/crimson.hpp"

namespace
{
    constexpr std::uint8_t kLoopDelayMs = 20;

    constexpr std::int8_t kIntakePort = 3; // Change to your intake port (use negative if reversed, e.g. -3)

    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    pros::Motor intake(kIntakePort);
    crimson::Crimson crimson_cam(1);
    pros::Imu imu(2);
    display::Dashboard dashboard;

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

    imu.reset();
    dashboard.initialize();

    pros::screen::set_pen(0xFFFFFF);
    pros::screen::print(pros::E_TEXT_MEDIUM, 250, 100, "Initializing IMU...");

    while (imu.is_calibrating())
    {
        pros::delay(10);
    }
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol()
{
    while (true)
    {
        crimson_cam.update();
        update_dashboard();

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
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