#include "util/driver_task.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace util {

bool driver_control_active() {
    return !pros::competition::is_disabled() && !pros::competition::is_autonomous();
}

void start_driver_task(const char* name, void (*update)(), void (*stop)()) {
    pros::Task task(
        [update, stop] {
            std::uint32_t now = pros::millis();
            while (driver_control_active()) {
                update();
                pros::Task::delay_until(&now, kLoopMs);
            }
            if (stop != nullptr) {
                stop();
            }
        },
        name);
}

}
