#include "display/motor_temps.hpp"
#include "robotconfig.hpp"
#include <cmath>
#include <vector>

namespace display::motor_temps {

namespace {

struct Group {
    pros::MotorGroup& motors;
    const char* name; // short, for the controller: DL2 = left drive motor 2
};

Group groups[] = {
    {left_motors, "DL"},
    {right_motors, "DR"},
    {extension_motors, "LIFT"},
    {arm_pivot_motor, "ARM"},
    {side_toggle_motors, "TOG"},
};

bool was_hot = false;

}

bool print_warning(int line) {
    const char* hot_name = nullptr;
    int hot_index = 0;
    double hot_c = 0.0;
    for (Group& group : groups) {
        const std::vector<double> temps = group.motors.get_temperature_all();
        for (std::size_t i = 0; i < temps.size(); ++i) {
            if (std::isfinite(temps[i]) && temps[i] >= kWarnC && temps[i] > hot_c) { // unplugged reads PROS_ERR_F
                hot_name = group.name;
                hot_index = static_cast<int>(i) + 1;
                hot_c = temps[i];
            }
        }
    }

    if (hot_name == nullptr) {
        was_hot = false;
        return false;
    }
    if (!was_hot) {
        was_hot = true;
        controller.rumble("---"); // uses this print slot; the text follows next time round
        return true;
    }
    controller.print(line, 0, "HOT %s%d %2.0fC       ", hot_name, hot_index, hot_c);
    return true;
}

}
