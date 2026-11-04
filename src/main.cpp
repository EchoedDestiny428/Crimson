#include "main.h"
#include "autons/autons.hpp"
#include "config/drivetrain.hpp"
#include "robotconfig.hpp"
#include "macros.hpp"
#include "subsystems/claw.hpp"
#include "subsystems/drive.hpp"
#include "subsystems/elevator/extension.hpp"
#include "subsystems/imu_peaks.hpp"
#include "subsystems/pivot.hpp"
#include "subsystems/screen.hpp"
#include "subsystems/toggle.hpp"
#include "telemetry/telemetry.hpp"
#include "util/driver_task.hpp"

using namespace subsystems;

void initialize() {
    controller.clear();
    dashboard.initialize();
    dashboard.show_status("CALIBRATING");

    drive::init();
    elevator::extension::init();
    pivot::init();
    // Piston and flex wheels stay off until those tests. Macros stay off so they cannot move the bar.
    // claw::init();
    // toggle::init();

    screen::start();
    imu_peaks::start();
    telemetry::start();
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
    // macros::cancel();
    chassis.cancelAllMotions();
    chassis.setPose(config::kStartXIn, config::kStartYIn, config::kStartHeadingDeg);

    util::start_driver_task("Extension", elevator::extension::update, elevator::extension::stop);
    util::start_driver_task("Pivot", pivot::update, pivot::stop);
    // util::start_driver_task("Claw", claw::update, claw::stop);
    // util::start_driver_task("Toggle", toggle::update, toggle::stop);
    // util::start_driver_task("Macros", macros::update, macros::stop);

    drive::reset();
    std::uint32_t now = pros::millis();
    while (true) {
        drive::update();
        pros::Task::delay_until(&now, util::kLoopMs);
    }
}
