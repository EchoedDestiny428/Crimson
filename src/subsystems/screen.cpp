#include "subsystems/screen.hpp"
#include "autons/autons.hpp"
#include "robotconfig.hpp"
#include "vision/localizer.hpp"
#include <cstdint>

namespace subsystems::screen {

namespace {

constexpr std::uint32_t kPeriodMs = 20;
constexpr std::uint32_t kDashboardEveryTicks = 5;

void run() {
    std::uint32_t now = pros::millis();
    std::uint32_t tick = 0;
    bool results_were_up = false;

    while (true) {
        // latency-testing: the test's results stay up until the screen is touched, then the dashboard comes back.
        if (autons::latency_test::keep_results_on_screen()) {
            results_were_up = true;
        } else if (results_were_up) {
            results_were_up = false;
            dashboard.initialize();
        } else if (pros::competition::is_disabled()) {
            autons::draw_selector();
        } else if (++tick % kDashboardEveryTicks == 0) {
            dashboard.update(chassis.getPose(), vision::localizer::snapshot());
        }

        pros::Task::delay_until(&now, kPeriodMs);
    }
}

}

void start() {
    pros::Task task(run, "Screen");
}

}
