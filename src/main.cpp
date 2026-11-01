#include "main.h"
#include "display/dashboard.hpp"
#include "pros/rtos.hpp"
#include "vision/localizer.hpp"
#include <cstdint>

namespace {

constexpr std::uint32_t kScreenPeriodMs = 100;

display::Dashboard dashboard;

void draw_screen() {
    std::uint32_t now = pros::millis();
    while (true) {
        dashboard.update(vision::localizer::snapshot());
        pros::Task::delay_until(&now, kScreenPeriodMs);
    }
}

}

void initialize() {
    dashboard.initialize();
    vision::localizer::start();
    pros::Task task(draw_screen, "Screen");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {}
