#include "subsystems/drive.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"

namespace subsystems::drive {

void init() {
    chassis.calibrate();
}

// Plain arcade: no drive curve and no slew. Motor groups clamp to +/-127.
void update() {
    const int throttle = controller.get_analog(controls::kThrottle);
    const int turn = controller.get_analog(controls::kTurn);
    left_motors.move(throttle + turn);
    right_motors.move(throttle - turn);
}

}
