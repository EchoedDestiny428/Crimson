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
        const bool refresh = ++tick % kDashboardEveryTicks == 0;
        if (pros::competition::is_disabled()) {
            autons::draw_selector();
        } else if (!pros::competition::is_autonomous() && refresh) {
            autons::match::mech_test::draw();
        } else if (refresh) {
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
