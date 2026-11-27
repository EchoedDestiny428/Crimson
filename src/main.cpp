#include "main.h"
#include "autons/autons.hpp"
#include "config/camera.hpp"
#include "display/controller_hud.hpp"
#include "robotconfig.hpp"
#include "subsystems/drive.hpp"
#include "subsystems/elevator/claw.hpp"
#include "subsystems/elevator/extension.hpp"
#include "subsystems/elevator/macros.hpp"
#include "subsystems/elevator/arm_pivot.hpp"
#include "subsystems/screen.hpp"
#include "subsystems/side_toggle.hpp"
#include "util/driver_task.hpp"
#include "vision/localizer.hpp"

using namespace subsystems;

void initialize() {
    controller.clear();
    apply_motor_config();
    // Tracking wheels report every 5 ms instead of the default 10 (logs showed them changing only every 20 ms and a
    // few ms behind the IMU, which LemLib pairs them with: fast turns drifted).
    vertical_odom.set_data_rate(5);
    horizontal_odom.set_data_rate(5);
    dashboard.initialize();
    dashboard.show_status("CALIBRATING");

    drive::init();
    chassis.setPose(config::kStartXIn, config::kStartYIn, config::kStartHeadingDeg);
    elevator::extension::init();
    elevator::arm_pivot::init();
    side_toggle::init();
    logger.attach(&left_motors);
    logger.attach(&right_motors);
    logger.attach(&extension_motors);
    logger.attach(&arm_pivot_motor);
    logger.attach(&side_toggle_motors);
    logger.register_robot_fields();
    // How hard the arm is allowed to swing right now (it follows the lift height): pairs with imu_pitch/roll in the log.
    logger.register_field("arm_swing_accel_dps2", [](const void*) { return elevator::arm_pivot::swing_accel(); });
    // The elevator as the macros see it (the port columns' target_position is 0 for SmartMotor-driven motors): lift cm
    // above its start, arm degrees from straight down, and what each is commanded to (the arm's with its slack
    // adjustment); the claw (1 closed); settled as macros::settled() and wait_settled() see it; the arm's home still
    // on its way to the hardstop.
    logger.register_field("lift_cm", [](const void*) { return elevator::macros::pose_now().lift_cm; });
    logger.register_field("lift_target_cm", [](const void*) { return elevator::macros::target_now().lift_cm; });
    logger.register_field("arm_deg", [](const void*) { return elevator::macros::pose_now().arm_deg; });
    logger.register_field("arm_target_deg", [](const void*) { return elevator::macros::target_now().arm_deg; });
    logger.register_field("claw_closed", [](const void*) { return elevator::claw::is_closed() ? 1.0 : 0.0; });
    logger.register_field("elev_settled", [](const void*) { return elevator::macros::settled() ? 1.0 : 0.0; });
    logger.register_field("arm_homing", [](const void*) { return elevator::arm_pivot::homing() ? 1.0 : 0.0; });
    logger.start();

    screen::start();
    ::vision::localizer::start();
}

void disabled() {
    elevator::arm_pivot::set_hold(false);
    elevator::extension::hold();
    autons::select();
}

void competition_initialize() {
    autons::select();
}

void autonomous() {
    elevator::arm_pivot::set_hold(true);
    autons::run_selected();
}

void opcontrol() {
    logger.begin_run("driver", false); // a new log file for driver control
    chassis.cancelAllMotions();
    elevator::arm_pivot::set_hold(true);
    util::start_driver_task("Elevator stages", elevator::macros::update, elevator::macros::stop);
    util::start_driver_task("Claw", elevator::claw::update);
    util::start_driver_task("Side toggle", side_toggle::update, side_toggle::stop);
    util::start_driver_task("HUD", display::controller_hud::update);

    std::uint32_t now = pros::millis();
    while (true) {
        drive::update();
        pros::Task::delay_until(&now, util::kLoopMs);
    }
}
