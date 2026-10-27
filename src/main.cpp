#include "main.h"
#include "autons/autons.hpp"
#include "display/controller_hud.hpp"
#include "robotconfig.hpp"
#include "subsystems/drive.hpp"
#include "subsystems/vision.hpp"
#include "util/driver_task.hpp"

using namespace subsystems;

void initialize() {
    controller.clear();
    vision::init();
    dashboard.show_status("CALIBRATING");

    drive::init();
    logger.attach(&left_motors);
    logger.attach(&right_motors);
    logger.register_robot_fields();
    logger.start();

    vision::start();
}

void disabled() {
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
    util::start_driver_task("HUD", display::controller_hud::update);

    std::uint32_t now = pros::millis();
    while (true) {
        drive::update();
        pros::Task::delay_until(&now, util::kLoopMs);
    }
}
