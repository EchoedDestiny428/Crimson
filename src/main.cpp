#include "main.h"
#include "autons/autons.hpp"
#include "display/controller_hud.hpp"
#include "robotconfig.hpp"
#include "subsystems/drive.hpp"
#include "subsystems/elevator/claw.hpp"
#include "subsystems/elevator/extension.hpp"
#include "subsystems/elevator/macros.hpp"
#include "subsystems/elevator/arm_pivot.hpp"
#include "subsystems/screen.hpp"
#include "util/driver_task.hpp"
#include "vision/localizer.hpp"

using namespace subsystems;

void initialize() {
    controller.clear();
    apply_motor_config();
    dashboard.initialize();
    dashboard.show_status("CALIBRATING");

    drive::init();
    elevator::extension::init();
    elevator::arm_pivot::init();
    logger.attach(&left_motors);
    logger.attach(&right_motors);
    logger.attach(&extension_motors);
    logger.attach(&arm_pivot_motor);
    logger.register_robot_fields();
    logger.start();

    screen::start();
    vision::localizer::start();
}

void disabled() {
    elevator::extension::hold();
    autons::select();
}

void competition_initialize() {
    autons::select();
}

void autonomous() {
    autons::run_selected();
}

void opcontrol() {
    chassis.cancelAllMotions();
    util::start_driver_task("Elevator stages", elevator::macros::update, elevator::macros::stop);
    util::start_driver_task("Arm pivot", elevator::arm_pivot::update, elevator::arm_pivot::stop);
    util::start_driver_task("Claw", elevator::claw::update);
    util::start_driver_task("HUD", display::controller_hud::update);

    std::uint32_t now = pros::millis();
    while (true) {
        drive::update();
        pros::Task::delay_until(&now, util::kLoopMs);
    }
}
