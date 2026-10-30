#include "subsystems/screen.hpp"
#include "autons/autons.hpp"
#include "robotconfig.hpp"
#include <cstdint>

namespace subsystems::screen {

namespace {

constexpr std::uint32_t kPeriodMs = 20;
constexpr std::uint32_t kDashboardEveryTicks = 5;

void run() {
    std::uint32_t now = pros::millis();
    std::uint32_t tick = 0;

    while (true) {
        if (pros::competition::is_disabled()) {
            autons::draw_selector();
        } else if (++tick % kDashboardEveryTicks == 0) {
            dashboard.update(chassis.getPose());
        }

        pros::Task::delay_until(&now, kPeriodMs);
    }
}

}

void start() {
    pros::Task task(run, "Screen");
}

}
