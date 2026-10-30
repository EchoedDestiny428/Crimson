#include "main.h"
#include "autons/autons.hpp"
#include "macros.hpp"
#include "robotconfig.hpp"
#include "subsystems/claw.hpp"
#include "subsystems/drive.hpp"
#include "subsystems/elevator.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/pivot.hpp"
#include "subsystems/vision.hpp"
#include "telemetry/telemetry.hpp"
#include "util/driver_task.hpp"
#include "util/system_check.hpp"

using namespace subsystems;

void initialize() {
    controller.clear();
    vision::init();
    dashboard.show_status("CALIBRATING");

    drive::init();
    claw::init();
    pivot::init();
    elevator::init();
    pros::Task startup_home(macros::home, "Startup home");

    vision::start();
    telemetry::start();
}

void disabled() {
    elevator::hold();
    autons::select();
}

void competition_initialize() {
    autons::select();
}

void autonomous() {
    autons::run_selected();
}

void opcontrol() {
    if (!pros::competition::is_connected()) {
        autons::run_selected();
    }

    macros::cancel();
    chassis.cancelAllMotions();
    claw::automatic();
    util::start_driver_task("System check", util::system_check::update, util::system_check::stop);
    util::start_driver_task("Intake", intake::update, intake::stop);
    util::start_driver_task("Claw", claw::update, claw::automatic);
    util::start_driver_task("Pivot", pivot::update, pivot::stop);
    util::start_driver_task("Macros", macros::update, macros::stop);

    drive::reset();
    std::uint32_t now = pros::millis();
    while (true) {
        drive::update();
        pros::Task::delay_until(&now, util::kLoopMs);
    }
}
