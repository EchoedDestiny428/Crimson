#include "main.h"
#include "autons/autons.hpp"
#include "display/controller_hud.hpp"
#include "robotconfig.hpp"
#include "subsystems/drive.hpp"
#include "subsystems/elevator/claw.hpp"
#include "subsystems/elevator/extension.hpp"
#include "subsystems/elevator/macros.hpp"
#include "subsystems/elevator/arm_pivot.hpp"
#include "subsystems/vision.hpp"
#include "tuning/kg_tune.hpp"
#include "util/driver_task.hpp"

using namespace subsystems;

void initialize() {
    controller.clear();
    apply_motor_config();
    vision::init();
    dashboard.show_status("CALIBRATING");

    drive::init();
    elevator::extension::init();
    tuning::kg::init(); // kG tuning build: instead of elevator::arm_pivot::init()
    logger.attach(&left_motors);
    logger.attach(&right_motors);
    logger.attach(&extension_motors);
    logger.attach(&arm_pivot_motor);
    logger.register_robot_fields();
    logger.start();

    vision::start();
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
    // kG tuning build: only the drive and the tuner (A, Up, Down) run.
    util::start_driver_task("kG tune", tuning::kg::update);

    std::uint32_t now = pros::millis();
    while (true) {
        drive::update();
        pros::Task::delay_until(&now, util::kLoopMs);
    }
}
