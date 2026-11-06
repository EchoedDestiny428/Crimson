#include "subsystems/drive.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"

namespace subsystems::drive {

void init() {
    chassis.calibrate();
}

void update() {
    const int throttle = controller.get_analog(controls::kThrottle);
    const int turn = controller.get_analog(controls::kTurn);
    left_motors.move(throttle + turn);
    right_motors.move(throttle - turn);
}

}
