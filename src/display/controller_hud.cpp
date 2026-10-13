#include "display/controller_hud.hpp"
#include "robotconfig.hpp"

namespace display::controller_hud {

void update() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) &&
        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        logger.dump();
    }
}

}
