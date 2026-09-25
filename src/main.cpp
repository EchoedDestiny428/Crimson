#include "main.h"
#include "macros.hpp"
#include "robotconfig.hpp"
#include "subsystems/claw.hpp"
#include "subsystems/drive.hpp"
#include "subsystems/elevator.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/pivot.hpp"
#include "subsystems/vision.hpp"
#include "util/driver_task.hpp"

using namespace subsystems;

void initialize() {
    vision::init();
    dashboard.show_status("CALIBRATING");

    drive::init();
    claw::init();
    pivot::init();
    elevator::init();

    vision::start();
}

void disabled() {
    elevator::hold();
}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    util::start_driver_task("Intake", intake::update, intake::stop);
    util::start_driver_task("Claw", claw::update, claw::stop);
    util::start_driver_task("Pivot", pivot::update, pivot::stop);
    util::start_driver_task("Macros", macros::update, macros::stop);

    drive::reset();
    std::uint32_t now = pros::millis();
    while (true) {
        drive::update();
        pros::Task::delay_until(&now, util::kLoopMs);
    }
}
