#include "main.h"
#include "autons/autons.hpp"
#include "robotconfig.hpp"
#include "subsystems/drive.hpp"
#include "subsystems/screen.hpp"
#include "telemetry/telemetry.hpp"

using namespace subsystems;

void initialize() {
    controller.clear();
    dashboard.initialize();
    dashboard.show_status("CALIBRATING");

    drive::init();

    screen::start();
    telemetry::start();
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
    if (!pros::competition::is_connected()) {
        autons::run_selected();
    }

    chassis.cancelAllMotions();

    drive::reset();
    std::uint32_t now = pros::millis();
    while (true) {
        drive::update();
        pros::Task::delay_until(&now, drive::kLoopMs);
    }
}
