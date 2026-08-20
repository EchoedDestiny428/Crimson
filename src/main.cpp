#include "main.h"
#include "crimson.hpp"

namespace {
	constexpr std::uint8_t kLoopDelayMs = 20;
	crimson::Crimson crimson_cam(1, 1, VISION_ZERO_TOPLEFT);

	void print_crimson_status() {
		pros::lcd::print(0, "tv:%d sig:%u age:%ums", crimson_cam.get_tv(), crimson_cam.get_signature_id(),
		                 crimson_cam.get_data_age_ms());

		if (!crimson_cam.has_target()) {
			pros::lcd::print(1, "tx: --  ty: --");
			pros::lcd::print(2, "ta: --  ts: --");
			pros::lcd::print(3, "target: none");
			return;
		}

		const auto target = crimson_cam.get_raw_target();
		pros::lcd::print(1, "tx:%5.1f ty:%5.1f", crimson_cam.get_tx(), crimson_cam.get_ty());
		pros::lcd::print(2, "ta:%5.1f ts:%5.1f", crimson_cam.get_ta(), crimson_cam.get_ts());
		pros::lcd::print(3, "x:%d y:%d w:%d h:%d", target.x_middle_coord, target.y_middle_coord,
		                 target.width, target.height);
	}
}

void initialize() {
	pros::lcd::initialize();
	crimson_cam.configure_defaults(50, false);
	pros::lcd::set_text(0, "crimson-style vision ready");
	pros::lcd::set_text(1, "tv/tx/ty/ta/ts diagnostics");
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