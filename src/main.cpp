#include "main.h"
#include "llv.hpp"

namespace {
	constexpr std::uint8_t kLoopDelayMs = 20;
	llv::LimelightVision limelight(1, 1, VISION_ZERO_TOPLEFT);

	void print_limelight_status() {
		pros::lcd::print(0, "tv:%d sig:%u age:%ums", limelight.get_tv(), limelight.get_signature_id(),
		                 limelight.get_data_age_ms());

		if (!limelight.has_target()) {
			pros::lcd::print(1, "tx: --  ty: --");
			pros::lcd::print(2, "ta: --  ts: --");
			pros::lcd::print(3, "target: none");
			return;
		}

		const auto target = limelight.get_raw_target();
		pros::lcd::print(1, "tx:%5.1f ty:%5.1f", limelight.get_tx(), limelight.get_ty());
		pros::lcd::print(2, "ta:%5.1f ts:%5.1f", limelight.get_ta(), limelight.get_ts());
		pros::lcd::print(3, "x:%d y:%d w:%d h:%d", target.x_middle_coord, target.y_middle_coord,
		                 target.width, target.height);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	limelight.configure_defaults(50, false);
	pros::lcd::set_text(0, "limelight-style vision ready");
	pros::lcd::set_text(1, "tv/tx/ty/ta/ts diagnostics");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true) {
		limelight.update();
		print_limelight_status();
		pros::delay(kLoopDelayMs);
	}
}