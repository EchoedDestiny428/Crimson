#include "main.h"
#include "autons/autons.hpp"
#include "config/camera.hpp"
#include "display/controller_hud.hpp"
#include "robotconfig.hpp"
#include "subsystems/donut_tester.hpp"
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

// donut-testing branch: driver control is only the drive and the donut tuner (src/subsystems/donut_tester.cpp). The
// lift, arm, claw and side toggle are not on the controller; the arm still holds where it is.
void opcontrol() {
    chassis.cancelAllMotions();
    elevator::arm_pivot::set_hold(true);
    controller.clear();

    std::uint32_t now = pros::millis();
    while (true) {
        if (!donut_tester::update()) {
            drive::update();
        }
        pros::Task::delay_until(&now, util::kLoopMs);
    }
}
