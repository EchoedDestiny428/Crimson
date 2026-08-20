#include "main.h"
#include "vision/crimson.hpp"
#include "display/dashboard.hpp"

namespace {
	constexpr std::uint8_t kLoopDelayMs = 20;
	crimson::Crimson crimson_cam(1);
    pros::Imu imu(2);
    display::Dashboard dashboard;

	void update_dashboard() {
        double heading = imu.get_heading();
        auto pose = crimson_cam.estimate_global_pose(heading);

        int tag_id = -1;
        if (crimson_cam.has_target()) {
            tag_id = crimson_cam.get_raw_target().id;
        }

        dashboard.update(pose, crimson_cam.get_tv(), tag_id);
	}
}

void initialize() {
	// Remove pros::lcd::initialize() to avoid UI conflicts with pros::screen
	crimson_cam.initialize();
    crimson_cam.set_camera_mount_metrics(100.0, 0.0);
    
    imu.reset();
    dashboard.initialize();
    
    // Print init status to our dashboard area
    pros::screen::set_pen(0xFFFFFF);
    pros::screen::print(pros::E_TEXT_MEDIUM, 250, 100, "Initializing IMU...");
    
    while (imu.is_calibrating()) {
        pros::delay(10);
    }
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	while (true) {
		crimson_cam.update();
		update_dashboard();
		pros::delay(kLoopDelayMs);
	}
}