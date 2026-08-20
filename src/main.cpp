#include "main.h"
#include "vision/crimson.hpp"

namespace {
	constexpr std::uint8_t kLoopDelayMs = 20;
	crimson::Crimson crimson_cam(1);
    pros::Imu imu(2);

	void print_crimson_status() {
        double heading = imu.get_heading();
        auto pose = crimson_cam.estimate_global_pose(heading);

		pros::lcd::print(0, "tv:%d age:%ums", crimson_cam.get_tv(), crimson_cam.get_data_age_ms());

        if (pose.has_value()) {
            pros::lcd::print(1, "POSE X:%5.1f Y:%5.1f", pose->x, pose->y);
            pros::lcd::print(2, "HEADING: %5.1f", pose->theta);
        } else if (!crimson_cam.has_target()) {
			pros::lcd::print(1, "tx: --  ty: --");
			pros::lcd::print(2, "ta: --  ts: --");
		} else {
            pros::lcd::print(1, "tx:%5.1f ty:%5.1f", crimson_cam.get_tx(), crimson_cam.get_ty());
            pros::lcd::print(2, "NO POSE ESTIMATE");
        }

        if (crimson_cam.has_target()) {
		    const auto target = crimson_cam.get_raw_target();
		    pros::lcd::print(3, "TAG ID:%d", target.id);
        } else {
            pros::lcd::print(3, "target: none");
        }
	}
}

void initialize() {
	pros::lcd::initialize();
	crimson_cam.initialize();
    crimson_cam.set_camera_mount_metrics(100.0, 0.0);
    
    imu.reset();
	pros::lcd::set_text(0, "crimson AI vision ready");
	pros::lcd::set_text(1, "Initializing IMU...");
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
		print_crimson_status();
		pros::delay(kLoopDelayMs);
	}
}